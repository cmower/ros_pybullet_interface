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
from ros_pybullet_interface.srv import ResetJointState, ResetJointStateResponse

class PybulletRobot(PybulletObject):

    """Interface for robots in Pybullet (simulated and visualizations). Note, visualized robots do not interact with objects in Pybullet."""

    snap_to_real_robot_timeout = 10.0  # secs

    def init(self):

        ##################################
        ## Extract configurations

        urdf_config = self.config['loadURDF']
        self.use_fixed_base = urdf_config.get('useFixedBase', 0)  # pybullet defaults to 0

        object_tf_config = self.config.get('object_tf', {})  # optional, can be given via basePosition/baseOrientation in loadURDF config

        self.is_visual_robot = self.config.get('is_visual_robot', False)
        set_joint_motor_control_array_config = self.config.get('setJointMotorControlArray', {})

        ##################################
        ## Setup object tf

        self.offset = self.get_object_offset_in_base_tf(object_tf_config)
        self.base = self.get_static_object_base_tf_in_world(object_tf_config)
        base_position, base_orientation = self.get_base_position_and_orientation(self.offset, self.base)

        ##################################
        ## Load URDF

        # Get urdf filename and create new temp filename
        urdf_filename = replace_package(urdf_config['fileName'])

        # Check if urdf contains ros package:// statements
        if urdf_contains_ros_package_statements(urdf_filename):
            urdf_filename = replace_ros_package_statements(urdf_filename)

        # Setup input for loadURDF
        load_urdf_input = urdf_config.copy()
        load_urdf_input['fileName'] = urdf_filename
        if 'basePosition' not in load_urdf_input:
            load_urdf_input['basePosition'] = base_position
        if 'baseOrientation' not in load_urdf_input:
            load_urdf_input['baseOrientation'] = base_orientation
        self.urdf_filename = urdf_filename
        if 'flags' in load_urdf_input:
            if isinstance(load_urdf_input['flags'], str):
                load_urdf_input['flags'] = eval('|'.join(['self.pb.' + f for f in load_urdf_input['flags'].split('|')]))

        # Load URDF
        self.body_unique_id = self.pb.loadURDF(**load_urdf_input)

        ##################################
        ## Setup joints interface

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

        ##################################
        ## Setup target joint states

        # Check if robot is visual or not
        if not self.is_visual_robot:
            # not visual robot (use motor control commands)

            # Get control mode
            cm = set_joint_motor_control_array_config.get('controlMode', 'POSITION_CONTROL')
            if isinstance(cm, str):
                self.control_mode = getattr(self.pb, cm)
            elif isinstance(cm, int):
                self.control_mode = cm
            else:
                raise ValueError(f'did not reconize type of control mode, given "{type(cm)}", expected either a "str" or "int".')

            # Setup setJointMotorControlArray input
            self.set_joint_motor_control_array_input = {'bodyIndex': self.body_unique_id, 'controlMode': self.control_mode}

            # Set gains (optional)
            for name in ('positionGains', 'velocityGains'):
                if name in set_joint_motor_control_array_config:
                    self.set_joint_motor_control_array_input[name] = set_joint_motor_control_array_config[name]

            # Set reset method
            if self.control_mode == self.pb.POSITION_CONTROL:
                self.reset_set_joint_motor_control_array_input = self.reset_set_joint_motor_control_array_input_position
            elif self.control_mode == self.pb.VELOCITY_CONTROL:
                self.reset_set_joint_motor_control_array_input = self.reset_set_joint_motor_control_array_input_velocity
            elif self.control_mode == self.pb.TORQUE_CONTROL:
                self.reset_set_joint_motor_control_array_input = self.reset_set_joint_motor_control_array_input_force
            else:
                raise NotImplementedError(f'control mode with id "{self.control_mode}" is not supported!')

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
        self.subs['target_joint_state'] = self.node.Subscriber(topic_name, JointState, target_joint_state_callback)

        ##################################
        ## Setup FT sensors

        if not self.is_visual_robot:

            # Retrieve list of joint names and enable those as f/t sensors (optional)
            # Note: if robot is visual then this data will not be published, even if this is set in config file
            self.enabled_joint_force_torque_sensors = self.config.get('enabled_joint_force_torque_sensors', []) # list of joint names
            for joint_name in self.enabled_joint_force_torque_sensors:
                self.pb.enableJointForceTorqueSensor(self.body_unique_id, self.joint_names.index(joint_name), enableSensor=1)
                self.pubs[f'{self.name}_{joint_name}_ft_sensor'] = self.node.Publisher(f'rpbi/{self.name}/{joint_name}/ft_sensor', WrenchStamped, queue_size=10)

        ##################################
        ## Setup current state publishers

        if not self.is_visual_robot:
            # Start current joint state publisher
            freq = self.config.get('publish_joint_state_frequency', 50)
            self.pubs['current_joint_states'] = self.node.Publisher(f'rpbi/{self.name}/joint_states', JointState, queue_size=10)
            self.node.Timer(self.node.Duration(1.0/float(freq)), self.joint_state_publisher)

        # Start publishing link states
        if self.config.get('publish_link_states', False):

            # Get root link name
            # Note, to the best of my knowledge Pybullet doesn't have a way to return the root link name
            self.root_link_name = self.get_root_link_name()

            # Setup publish link state timer
            freq = self.config.get('publish_link_states_frequency', 50)
            self.timers['publish_link_states'] = self.node.Timer(self.node.Duration(1.0/float(freq)), self.publish_link_states)

        # Show joint limits
        if self.is_visual_robot:
            if self.config.get('show_joint_limits', True):
                self.timers['show_joint_limits'] = self.node.Timer(self.node.Duration(1.0/50.0), self.show_joint_limits)

        ##################################
        ## Setup services
        self.srvs['robot_info'] = self.node.Service(f'rpbi/{self.name}/robot_info', RobotInfo, self.service_robot_info)
        self.srvs['snap_to_real_robot'] = self.node.Service(f'rpbi/{self.name}/snap_to_real_robot', SetString, self.service_snap_to_real_robot)
        self.srvs['reset_joint_state'] = self.node.Service(f'rpbi/{self.name}/reset_joint_state', ResetJointState, self.service_reset_joint_state)
        self.srvs['move_to_joint_state'] = self.node.Service(f'rpbi/{self.name}/move_to_joint_state', ResetJointState, self.service_move_to_joint_state)

    def get_root_link_name(self):
        """Return the root link name."""
        # HACK: since I haven't been able to find how to retrieve the root link name, I had to use urdf_parser_py instead
        from urdf_parser_py import urdf
        with open(self.urdf_filename, 'r') as f:
            robot = urdf.Robot.from_xml_string(f.read())
        return robot.get_root()

    def reset_set_joint_motor_control_array_input_position(self, msg):
        """Update the input parameters for setJointMotorControlArray using position control mode."""
        self.set_joint_motor_control_array_input['jointIndices'] = [self.joint_names.index(joint_name) for joint_name in msg.name]
        self.set_joint_motor_control_array_input['targetPositions'] = msg.position


    def reset_set_joint_motor_control_array_input_velocity(self, msg):
        """Update the input parameters for setJointMotorControlArray using velocity control mode."""
        self.set_joint_motor_control_array_input['jointIndices'] = [self.joint_names.index(joint_name) for joint_name in msg.name]
        self.set_joint_motor_control_array_input['targetVelocities'] = msg.velocity


    def reset_set_joint_motor_control_array_input_force(self, msg):
        """Update the input parameters for setJointMotorControlArray using torque control mode."""
        self.set_joint_motor_control_array_input['jointIndices'] = [self.joint_names.index(joint_name) for joint_name in msg.name]
        self.set_joint_motor_control_array_input['force'] = msg.effort


    def target_joint_state_callback_motor_control(self, msg):
        """Target joint state callback for simulated robots."""
        self.reset_set_joint_motor_control_array_input(msg)
        self.pb.setJointMotorControlArray(**self.set_joint_motor_control_array_input)


    def target_joint_state_callback_reset_joint_state(self, msg):
        """Target joint state callback for visualized robots."""
        for name, position in zip(msg.name, msg.position):
            self.pb.resetJointState(self.body_unique_id, self.joint_names.index(name), position)

    def publish_wrench(self, name, joint_reaction_forces):
        """Publish wrench forces."""
        msg = WrenchStamped()
        msg.header.stamp = self.node.time_now()
        msg.wrench.force.x = joint_reaction_forces[0]
        msg.wrench.force.y = joint_reaction_forces[1]
        msg.wrench.force.z = joint_reaction_forces[2]
        msg.wrench.torque.x = joint_reaction_forces[3]
        msg.wrench.torque.y = joint_reaction_forces[4]
        msg.wrench.torque.z = joint_reaction_forces[5]
        self.pubs[f'{self.name}_{name}_ft_sensor'].publish(msg)

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
        self.pubs['current_joint_states'].publish(msg)


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


    def service_reset_joint_state(self, req):

        success = True
        message = 'reset joint state'
        try:
            if self.is_visual_robot:
                self.target_joint_state_callback_reset_joint_state(req.joint_state)
            else:
                self.target_joint_state_callback_motor_control(req.joint_state)

        except Exception as e:
            success = False
            message = 'failed to reset joint state, exception: %s' % str(e)

        if success:
            self.node.loginfo(message)
        else:
            self.node.logerr(message)

        return ResetJointStateResponse(message=message, success=success)


    def service_move_to_joint_state(self, req):

        if self.control_mode not in {self.pb.POSITION_CONTROL, self.pb.VELOCITY_CONTROL}:
            success = False
            message = f'robot in  {self.control_mode}, currently this service only supports POSITION_CONTROL/VELOCITY_CONTROL modes!'
            self.node.logerr(message)
            return ResetJointStateResponse(message=message, success=success)

        # Setup
        success = True
        message = 'successfully moved robot to joint state'

        # Get joint states from pybullet
        joint_states = self.pb.getJointStates(self.body_unique_id, self.joint_indices)

        # Vectorize current and goal joint state
        qcurr = np.array([joint_states[self.joint_names.index(name)][0] for name in req.joint_state.name])
        qgoal = np.array(req.joint_state.position)

        # Move robot
        qprev = qcurr.copy()
        alpha = 0.0
        hz = 100
        dt = 1.0/float(hz)
        t0 = self.node.time_now().to_sec()
        rate = self.node.Rate(hz)
        while alpha < 1.0:
            alpha = (self.node.time_now().to_sec() - t0)/req.duration
            q = alpha*qgoal + (1.0-alpha)*qcurr
            dq = q - qprev
            joint_state = JointState(name=req.joint_state.name, position=q, velocity=dq/dt)
            if self.is_visual_robot:
                self.target_joint_state_callback_reset_joint_state(joint_state)
            else:
                self.target_joint_state_callback_motor_control(joint_state)
            qprev = q.copy()
            rate.sleep()

        return ResetJointStateResponse(message=message, success=success)

    def show_joint_limits(self, event):

        in_limit = [
            joint.in_limit(joint_states[0])
            for joint, joint_states in zip(self.joints, self.pb.getJointStates(self.body_unique_id, self.joint_indices))
            if joint.jointTypeStr != 'JOINT_FIXED'
        ]

        if not all(in_limit):
            print(self.node.time_now().to_sec(), "-------------%s joint states outside limit------------" % self.name)
