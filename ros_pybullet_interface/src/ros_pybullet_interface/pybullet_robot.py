import numpy
import pybullet
from .pybullet_object import PybulletObject
from .config import replace_package

class Joint:

    ACTIVE_JOINT_TYPES = {pybullet.JOINT_REVOLUTE, pybullet.JOINT_PRISMATIC}

    def __init__(self, robot_id, joint_index):
        self.info = pybullet.getJointInfo(robot_id, joint_index)

    def is_active(self):
        return self.type in Joint.ACTIVE_JOINT_TYPES

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

    def __init__(self, robot_id, robot_name, joint_index, ros_pub, to_ros_msg_method_handle, ros_api):
        self.robot_id = robot_id
        self.joint_index = joint_index
        pybullet.enableJointForceTorqueSensor(robot_id, joint_index, enableSensor=True)
        self.ros_pub = ros_pub
        self.ros_api = ros_api
        self.to_ros_msg_method_handle = to_ros_msg_method_handle

    def get_reading(self):
        state = pybullet.getJointState(self.robot_id, self.joint_index)
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
        self.body_unique_id = pybullet.loadURDF(replace_package(self.config['urdf_filename']), useFixedBase=True)
        self.ndof = pybullet.getNumJoints(self.body_unique_id)

        # Init joints
        self.joints = [Joint(self.body_unique_id, jidx) for jidx in range(self.ndof)]
        self.active_joints = [j for j in self.joints if j.is_active()]

        # Get tf frame id (defines the robot base in rpbi/world frame)
        self.tf_frame_id = self.config['tf_frame_id']

        # Set alpha - i.e. user can adjust robot transparency without writing a whole new URDF
        alpha = self.config.get('alpha', None)  #
        if alpha is not None:
            visual_shape_data = pybullet.getVisualShapeData(self.body_unique_id)
            for data in visual_shape_data:
                link_index = data[1]
                rgba = data[7]
                new_rgba = (rgba[0], rgba[1], rgba[2], alpha)
                pybullet.changeVisualShape(self.body_unique_id, link_index, rgbaColor=new_rgba)

        # Initialize other class attributes
        self.target_joint_state = None

    def target_joint_state_callback(self, msg):
        self.target_joint_state = msg.position


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
        init_position = numpy.deg2rad(self.config['init_position']).tolist()
        for p, j in zip(init_position, self.active_joints):
            pybullet.resetJointState(self.body_unique_id, j.index, p)

        # Initialize sensor containers
        self.sensors = {}

    def add_joint_force_torque_sensor(self, sensor_name, joint_index, to_ros_msg_method_handle, ros_pub):
        self.sensors[sensor_name] = JointForceTorqueSensor(self.body_unique_id, self.name, joint_index, ros_pub, to_ros_msg_method_handle, self.ros_api)

    def update(self):

        # Publish joint state
        joint_states = pybullet.getJointStates(self.body_unique_id, [j.index for j in self.active_joints])
        msg = self.ros_api['joint_states_to_ros_msg'](joint_states, self.active_joints, Joint)
        self.ros_api['joint_state_publisher'].publish(msg)

        # Publish link states
        link_states = pybullet.getLinkStates(self.body_unique_id, [j.index for j in self.joints])
        for link_state, joint in zip(link_states, self.joints):
            position = link_state[4]
            orientation = link_state[5]
            self.tf.set_tf('rpbi/world', f'rpbi/{self.name}/{joint.link_name}', position, orientation)

        # Publish sensor states
        for sensor in self.sensors.values():
            sensor.publish_state_to_ros()

        # Set base position/orientation
        pos, rot = self.tf.get_tf('rpbi/world', self.tf_frame_id)
        if pos is not None:
            pybullet.resetBasePositionAndOrientation(self.body_unique_id, pos, rot)

        # Set joint motor targets
        if self.target_joint_state is not None:
            pybullet.setJointMotorControlArray(
                self.body_unique_id,
                [j.index for j in self.active_joints],
                pybullet.POSITION_CONTROL,
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
            pybullet.setCollisionFilterGroupMask(self.body_unique_id, j.index, 0, 0)

        # Set initial joint state
        self.joint_state = numpy.deg2rad(self.config.get('init_position', [0.0]*len(self.active_joints))).tolist()

    def set_joint_state(self, position):
        for p, j in zip(position, self.active_joints):
            pybullet.resetJointState(self.body_unique_id, j.index, p)

    def update(self):

        # Set base position/orientation
        pos, rot = self.tf.get_tf('rpbi/world', self.tf_frame_id)
        if pos is not None:
            pybullet.resetBasePositionAndOrientation(self.body_unique_id, pos, rot)

        # Set joint state
        if self.target_joint_state is not None:
            self.set_joint_state(self.target_joint_state)
