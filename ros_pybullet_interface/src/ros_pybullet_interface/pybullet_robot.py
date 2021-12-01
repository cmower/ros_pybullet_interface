import numpy
from .pybullet_object import PybulletObject
from .config import replace_package

class Joint:

    def __init__(self, robot_id, joint_index, pybullet):
        self.pb = pybullet
        self.active_joint_types = {self.pb.JOINT_REVOLUTE, self.pb.JOINT_PRISMATIC}
        self.joint_type_as_string = {
            self.pb.JOINT_REVOLUTE: 'revolute',
            self.pb.JOINT_PRISMATIC: 'prismatic',
            self.pb.JOINT_SPHERICAL: 'spherical',
            self.pb.JOINT_PLANAR: 'planar',
            self.pb.JOINT_FIXED: 'fixed',
        }
        self.info = self.pb.getJointInfo(robot_id, joint_index)

    def is_active(self):
        return self.type in self.active_joint_types

    @property
    def index(self):
        return self.info[0]

    @property
    def name(self):
        return self.info[1].decode('utf-8')

    @property
    def type(self):
        return self.info[2]

    @property
    def type_as_string(self):
        return self.joint_type_as_string[self.type]

    @property
    def link_name(self):
        return self.info[12].decode('utf-8')

    @staticmethod
    def position_from_state(state):
        return state[0]

    @staticmethod
    def velocity_from_state(state):
        return state[1]

    @staticmethod
    def reaction_from_state(state):
        return state[2]

    @staticmethod
    def effort_from_state(state):
        return state[3]


class JointForceTorqueSensor:

    def __init__(self, robot_id, robot_name, joint_index, ros_pub, to_ros_msg_method_handle, pybullet, ros_api):
        self.robot_id = robot_id
        self.joint_index = joint_index
        self.pb = pybullet
        self.pb.enableJointForceTorqueSensor(robot_id, joint_index, enableSensor=True)
        self.ros_pub = ros_pub
        self.ros_api = ros_api
        self.to_ros_msg_method_handle = to_ros_msg_method_handle

    def get_reading(self):
        state = self.pb.getJointState(self.robot_id, self.joint_index)
        return Joint.reaction_from_state(state)

    def publish_state_to_ros(self):
        reading = self.get_reading()
        msg = self.to_ros_msg_method_handle(reading)
        self.ros_pub.publish(msg)


class _PybulletRobotBase(PybulletObject):

    """Base class for robots"""


    # Config:
    #
    # urdf_filename: str
    #      A relative or absolute path to the URDF file on the file
    #      system of the physics server.
    #
    # tf_frame_id: str
    #      Transform frame unique id for the robot base frame in
    #      rpbi/world frame.
    #
    # alpha: float
    #      Resets alpha for all parts of robot. Has to be 0
    #      (invisible) or 1 (visible), if not given then visual
    #      properties defined in URDF is used.
    #


    def init_robot(self):

        # Load robot
        self.use_fixed_base = self.config.get('use_fixed_base', True)
        self.body_unique_id = self.pb.loadURDF(
            replace_package(self.config['urdf_filename']),
            useFixedBase=self.use_fixed_base,
            flags=self.pb.URDF_USE_MATERIAL_COLORS_FROM_MTL,
        )
        self.ndof = self.pb.getNumJoints(self.body_unique_id)

        # Init joints
        self.joints = [Joint(self.body_unique_id, jidx, self.pb) for jidx in range(self.ndof)]
        self.active_joints = [j for j in self.joints if j.is_active()]
        self.ndof_active = len(self.active_joints)

        # Get tf frame id (defines the robot base in rpbi/world frame)
        self.tf_frame_id = self.config['tf_frame_id']

        # Set alpha - i.e. user can adjust robot transparency without writing a whole new URDF
        alpha = self.config.get('alpha', None)  #
        if alpha is not None:
            visual_shape_data = self.pb.getVisualShapeData(self.body_unique_id)
            for data in visual_shape_data:
                link_index = data[1]
                rgba = data[7]
                new_rgba = (rgba[0], rgba[1], rgba[2], alpha)
                self.pb.changeVisualShape(self.body_unique_id, link_index, rgbaColor=new_rgba)

        # Initialize other class attributes
        self.target_joint_state = None

    def target_joint_state_callback(self, msg):
        if len(msg.name) == 0:
            # hope that positions are in correct order
            target_joint_state = msg.position
        else:
            # populate target joint states using names
            # TODO: consider giving user option to supply a namespace
            target_joint_state = []
            for joint in self.active_joints:
                idx = msg.name.index(joint.name)
                target_joint_state.append(msg.position[idx])
        self.target_joint_state = target_joint_state



class PybulletRobot(_PybulletRobotBase):

    """A simulated robot."""

    # Config:
    #
    # name: str
    #      Name of object.
    #
    # urdf_filename: str
    #      A relative or absolute path to the URDF file on the file
    #      system of the physics server.
    #
    # tf_frame_id: str
    #      Transform frame unique id for the robot base frame in
    #      rpbi/world frame.
    #
    # alpha: float
    #      Resets alpha for all parts of robot. Has to be 0
    #      (invisible) or 1 (visible), if not given then visual
    #      properties defined in URDF is used.
    #
    # init_position: float[NDOF]
    #      Initial joint configuration in degrees. Note, NDOF is the
    #      number of active joints. Default is 0.0 joint positions.
    #

    def init(self):

        # Initialize robot
        self.init_robot()

        # Set initial joint state
        init_position = numpy.deg2rad(self.config.get('init_position', [0.0]*len(self.active_joints))).tolist()
        for p, j in zip(init_position, self.active_joints):
            self.pb.resetJointState(self.body_unique_id, j.index, p)

        # Initialize sensor containers
        self.sensors = {}

    def add_joint_force_torque_sensor(self, sensor_name, joint_index, to_ros_msg_method_handle, ros_pub):
        self.sensors[sensor_name] = JointForceTorqueSensor(self.body_unique_id, self.name, joint_index, ros_pub, to_ros_msg_method_handle, self.pb, self.ros_api)

    def update(self):

        # Publish joint state
        joint_states = self.pb.getJointStates(self.body_unique_id, [j.index for j in self.active_joints])
        msg = self.ros_api['joint_states_to_ros_msg'](joint_states, self.active_joints, Joint)
        self.ros_api['joint_state_publisher'].publish(msg)

        # Publish link states
        link_states = self.pb.getLinkStates(self.body_unique_id, [j.index for j in self.joints])
        for link_state, joint in zip(link_states, self.joints):
            position = link_state[4]
            orientation = link_state[5]
            self.tf.set_tf('rpbi/world', f'rpbi/{self.name}/{joint.link_name}', position, orientation)

        # Publish sensor states
        for sensor in self.sensors.values():
            sensor.publish_state_to_ros()

        # Set base position/orientation
        if self.use_fixed_base:
            pos, rot = self.tf.get_tf('rpbi/world', self.tf_frame_id)
            if pos is not None:
                self.pb.resetBasePositionAndOrientation(self.body_unique_id, pos, rot)

        # Set joint motor targets
        if self.target_joint_state is not None:
            self.pb.setJointMotorControlArray(
                self.body_unique_id,
                [j.index for j in self.active_joints],
                self.pb.POSITION_CONTROL,
                targetPositions=self.target_joint_state
            )

class PybulletVisualRobot(_PybulletRobotBase):

    """Visual representation of a robot - no interaction in the pybullet world."""

    # Config:
    #
    # name: str
    #      Name of object.
    #
    # urdf_filename: str
    #      A relative or absolute path to the URDF file on the file
    #      system of the physics server.
    #
    # tf_frame_id: str
    #      Transform frame unique id for the robot base frame in
    #      rpbi/world frame.
    #
    # alpha: float
    #      Resets alpha for all parts of robot. Has to be 0
    #      (invisible) or 1 (visible), if not given then visual
    #      properties defined in URDF is used.
    #
    # init_position: float[NDOF]
    #      Initial joint configuration in degrees. Note, NDOF is the
    #      number of active joints. Default is 0.0 joint positions.
    #

    def init(self):

        # Initialize robot
        self.init_robot()

        # Turn off collisions for robot
        for j in self.joints:
            self.pb.setCollisionFilterGroupMask(self.body_unique_id, j.index, 0, 0)

        # Set initial joint state
        self.joint_state = numpy.deg2rad(self.config.get('init_position', [0.0]*len(self.active_joints))).tolist()

    def set_joint_state(self, position):
        for p, j in zip(position, self.active_joints):
            self.pb.resetJointState(self.body_unique_id, j.index, p)

    def update(self):

        # Set base position/orientation
        pos, rot = self.tf.get_tf('rpbi/world', self.tf_frame_id)
        if pos is not None:
            self.pb.resetBasePositionAndOrientation(self.body_unique_id, pos, rot)

        # Set joint state
        if self.target_joint_state is not None:
            self.set_joint_state(self.target_joint_state)
