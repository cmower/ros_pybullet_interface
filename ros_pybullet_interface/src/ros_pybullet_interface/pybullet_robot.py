import tf_conversions
import numpy as np
from math import radians
from .pybullet_robot_joints import Joint
from .config import replace_package, ros_package_path
from .pybullet_robot_urdf import urdf_contains_ros_package_statements, replace_ros_package_statements
from .pybullet_object import PybulletObject
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from .utils import TimeoutExceeded
from cob_srvs.srv import SetString, SetStringResponse
from ros_pybullet_interface.srv import RobotInfo, RobotInfoResponse

class PybulletRobot(PybulletObject):

    snap_to_real_robot_timeout = 10.0  # secs

    def init(self):

        # Get urdf config
        config = self.config['loadURDF']
        self.use_fixed_base = config.get('useFixedBase', 0)  # pybullet defaults to 0

        # Get urdf filename and create new temp filename
        urdf_filename = replace_package(config['fileName'])

        # Check if urdf contains ros package:// statements
        if urdf_contains_ros_package_statements(urdf_filename):
            urdf_filename = replace_ros_package_statements(urdf_filename)

        # Setup input for loadURDF
        load_urdf_input = config.copy()
        load_urdf_input['fileName'] = urdf_filename
        self.urdf_filename = urdf_filename
        if 'flags' in load_urdf_input.keys():
            load_urdf_input['flags'] = eval('|'.join(['self.pb.' + f for f in load_urdf_input['flags'].split('|')]))

        # Load URDF
        self.body_unique_id = self.pb.loadURDF(**load_urdf_input)

        # Get joint information
        self.num_joints = self.pb.getNumJoints(self.body_unique_id)
        self.joints = [
            Joint(self.pb.getJointInfo(self.body_unique_id, jointIndex))
            for jointIndex in range(self.num_joints)
        ]
        self.ndof = sum([j.jointType != self.pb.JOINT_FIXED for j in self.joints])
        self.joint_names = [j.jointName for j in self.joints]
        self.joint_indices = [j.jointIndex for j in self.joints]
        self.link_names = [j.linkName for j in self.joints]

        # Set initial joint position
        initial_joint_state = self.config.get('initial_joint_position', {})
        joint_state = JointState()
        for joint_name, position in initial_joint_state.items():
            joint_state.name.append(joint_name)
            joint_type = self.joints[self.joint_names.index(joint_name)].jointType
            if joint_type == self.pb.JOINT_REVOLUTE:
                joint_state.position.append(radians(position))
            else:
                joint_state.position.append(position)
        self.target_joint_state_callback_reset_joint_state(joint_state)

        # Get frame offset
        self.get_frame_offset()

        # Set object base tf frame
        if self.use_fixed_base:
            self.setup_object_base_tf_frame()
        else:
            # Robot is not fixed base -> set initial base position/orientation

            # Get object base tf frame (optional, default to world frame)
            self.object_base_tf_frame_id = self.config.get('object_base_tf_frame_id', 'rpbi/world')

            if self.object_base_tf_frame_id != 'rpbi/world':
                # base frame is not world frame -> listen to tf frames

                # Get timer timeout if static
                if self.object_base_tf_frame_is_static:
                    self.object_base_tf_frame_listener_timeout = self.config.get('object_base_tf_frame_listener_timeout', 2)

                # Start looping: collect object tf
                got_tf = False
                self.object_base_tf_frame_listener_timer_start_time = self.node.time_now()
                while not got_tf:
                    pos_base, rot_base = self.node.tf.get_tf('rpbi/world', self.object_base_tf_frame_id)
                    if pos_base is not None:
                        got_tf = True
                    else:
                        # Compute time since the callback started
                        time_since_start = (self.node.time_now() - self.object_base_tf_frame_listener_timer_start_time).to_sec()

                        # Check if timeout exceeded
                        if (time_since_start > self.object_base_tf_frame_listener_timeout):
                            raise TimeoutExceeded(f'reached timeout ({self.object_base_tf_frame_listener_timeout} secs) to retrieve frame {self.object_base_tf_frame_id}!')

            else:

                # Base is world, so use zero transform
                pos_base = np.zeros(3)
                rot_base = np.array([0, 0, 0, 1])

            # Get base position/orientation
            T = self.offset_T @ self.node.tf.position_and_quaternion_to_matrix(pos_base, rot_base)
            base_position = T[:3,-1].flatten()
            base_orientation = tf_conversions.quaternion_from_matrix(T)

            self.pb.resetBasePositionAndOrientation(self.body_unique_id, base_position, base_orientation)

        # Check if robot is visual or not
        self.is_visual_robot = self.config.get('is_visual_robot', False)
        if not self.is_visual_robot:

            # Get control mode
            control_mode_str = self.config.get('controlMode', 'POSITION_CONTROL')
            self.control_mode = getattr(self.pb, control_mode_str)

            # Setup setJointMotorControlArray input
            self.set_joint_motor_control_array_input = {'bodyIndex': self.body_unique_id, 'controlMode': self.control_mode}

            # Set reset method
            if self.control_mode == self.pb.POSITION_CONTROL:
                self.reset_set_joint_motor_control_array_input = self.reset_set_joint_motor_control_array_input_position
            elif self.control_mode == self.pb.VELOCITY_CONTROL:
                self.reset_set_joint_motor_control_array_input = self.reset_set_joint_motor_control_array_input_velocity
            elif self.control_mode == self.pb.TORQUE_CONTROL:
                self.reset_set_joint_motor_control_array_input = self.reset_set_joint_motor_control_array_input_force
            else:
                raise NotImplementedError(f'control mode {control_mode_str} is not supported!')

            # Set callback method
            target_joint_state_callback = self.target_joint_state_callback_motor_control

        else:
            # False -> robot is a visual robot

            # Turn off collisions for robot
            for j in self.joints:
                self.pb.setCollisionFilterGroupMask(self.body_unique_id, j.jointIndex, 0, 0)

            # Set callback method
            target_joint_state_callback = self.target_joint_state_callback_reset_joint_state

        # Start target joint state subscriber
        topic_name = f'rpbi/{self.name}/joint_states/target'
        self.node.Subscriber(topic_name, JointState, target_joint_state_callback)

        # Retrieve list of joint names and enable those as f/t sensors (optional)
        # Note: if robot is visual then this data will not be published, even if this is set in config file
        self.enabled_joint_force_torque_sensors = self.config.get('enabled_joint_force_torque_sensors', []) # list of joint names
        self.ft_publishers = {}
        for joint_name in self.enabled_joint_force_torque_sensors:
            self.pb.enableJointForceTorqueSensor(self.body_unique_id, self.joint_names.index(joint_name), enableSensor=1)
            self.ft_publishers[joint_name] = self.node.Publisher(f'rpbi/{self.name}/{joint_name}/ft_sensor', WrenchStamped, queue_size=10)

        # Start current joint state publisher
        if not self.is_visual_robot:
            publish_joint_state_frequency = self.config.get('publish_joint_state_frequency', 50)
            publish_joint_state_dt = 1.0/float(publish_joint_state_frequency)
            self.joint_state_pub = self.node.Publisher(f'rpbi/{self.name}/joint_states', JointState, queue_size=10)
            self.node.Timer(self.node.Duration(publish_joint_state_dt), self.joint_state_publisher)

        # Start publishing link states
        if self.config.get('publish_link_states', False):

            # Get root link name
            # Note, to the best of my knowledge Pybullet doesn't have a way to return the root link name
            self.root_link_name = self.get_root_link_name()

            # Setup publish link state timer
            publish_link_states_frequency = self.config.get('publish_link_states_frequency', 50)
            publish_link_states_dt = 1.0/float(publish_link_states_frequency)
            self.node.Timer(self.node.Duration(publish_link_states_dt), self.publish_link_states)

        # Setup services
        self.node.Service(f'rpbi/{self.name}/robot_info', RobotInfo, self.service_robot_info)
        self.node.Service(f'rpbi/{self.name}/snap_to_real_robot', SetString, self.service_snap_to_real_robot)

    def get_root_link_name(self):
        # HACK: since I haven't been able to find how to retrieve the root link name, I had to use urdf_parser_py instead
        from urdf_parser_py import urdf
        with open(self.urdf_filename, 'r') as f:
            robot = urdf.Robot.from_xml_string(f.read())
        return robot.get_root()

    def reset_set_joint_motor_control_array_input_position(self, msg):
        self.set_joint_motor_control_array_input['jointIndices'] = [self.joint_names.index(joint_name) for joint_name in msg.name]
        self.set_joint_motor_control_array_input['targetPositions'] = msg.position


    def reset_set_joint_motor_control_array_input_velocity(self, msg):
        self.set_joint_motor_control_array_input['jointIndices'] = [self.joint_names.index(joint_name) for joint_name in msg.name]
        self.set_joint_motor_control_array_input['targetVelocities'] = msg.velocity


    def reset_set_joint_motor_control_array_input_force(self, msg):
        self.set_joint_motor_control_array_input['jointIndices'] = [self.joint_names.index(joint_name) for joint_name in msg.name]
        self.set_joint_motor_control_array_input['force'] = msg.effort


    def target_joint_state_callback_motor_control(self, msg):
        self.reset_set_joint_motor_control_array_input(msg)
        self.pb.setJointMotorControlArray(**self.set_joint_motor_control_array_input)


    def target_joint_state_callback_reset_joint_state(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.pb.resetJointState(self.body_unique_id, self.joint_names.index(name), position)


    def publish_wrench(self, name, joint_reaction_forces):
        msg = WrenchStamped()
        msg.header.stamp = self.node.time_now()
        msg.wrench.force.x = joint_reaction_forces[0]
        msg.wrench.force.y = joint_reaction_forces[1]
        msg.wrench.force.z = joint_reaction_forces[2]
        msg.wrench.torque.x = joint_reaction_forces[3]
        msg.wrench.torque.y = joint_reaction_forces[4]
        msg.wrench.torque.z = joint_reaction_forces[5]
        self.ft_publishers[name].publish(msg)


    def joint_state_publisher(self, event):

        # Get joint states from pybullet
        joint_states = self.pb.getJointStates(self.body_unique_id, self.joint_indices)

        # Get position, velocity, and effort
        position = []
        velocity = []
        effort = []
        for name, joint_state in zip(self.joint_names, joint_states):
            position.append(joint_state[0])
            velocity.append(joint_state[1])
            effort.append(joint_state[3])
            if name in self.enabled_joint_force_torque_sensors:
                self.publish_wrench(name, joint_state[2])

        # Create joint state message
        msg = JointState(name=self.joint_names, position=position, velocity=velocity, effort=effort)
        msg.header.stamp = self.node.time_now()
        self.joint_state_pub.publish(msg)


    def publish_link_states(self, event):

        # Get base position/orientation
        pos, rot = self.pb.getBasePositionAndOrientation(self.body_unique_id)
        self.node.tf.set_tf('rpbi/world', f'rpbi/{self.name}/{self.root_link_name}', pos, rot)

        # Iterate over joints
        for link_name, link_state in zip(self.link_names, self.pb.getLinkStates(self.body_unique_id, self.joint_indices, computeForwardKinematics=1)):
            self.node.tf.set_tf('rpbi/world', f'rpbi/{self.name}/{link_name}', link_state[0], link_state[1])


    def service_robot_info(self, req):
        return RobotInfoResponse(
            robot_name=self.name,
            root_link_name=self.root_link_name,
            bodyUniqueId=self.body_unique_id,
            numJoints=self.num_joints,
            numDof=self.ndof,
            joint_info=[j.as_ros_msg() for j in self.joints],
        )

    def service_snap_to_real_robot(self, req):

        # Setup
        success = True
        message = 'matched Pybullet robot to real robot'

        # Snap robot
        try:
            joint_state_topic = req.data
            msg = self.node.wait_for_message(joint_state_topic, JointState, timeout=self.snap_to_real_robot_timeout)
            self.target_joint_state_callback_reset_joint_state(msg)
        except Exception as e:
            success = False
            message = 'failed to match Pybullet robot to real robot, exception: %s' % str(e)

        # Log result
        if success:
            self.node.loginfo(message)
        else:
            self.node.logerr(message)

        return SetStringResponse(success=success, message=message)
