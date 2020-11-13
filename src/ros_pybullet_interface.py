#!/usr/bin/env python
import pybullet
import rospy
import yaml
import tf2_ros
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped

# ------------------------------------------------------
#
# Constants
# ------------------------------------------------------

FREQ = 100 # PyBullet sampling frequency
TARGET_JOINT_STATE_TOPIC = 'ros_pybullet_interface/joint_state/target' # listens for joint states on this topic
CURRENT_JOINT_STATE_TOPIC = 'ros_pybullet_interface/joint_state/current' # publishes joint states on this topic
WORLD_FRAME_ID = 'ros_pybullet_interface/world'

# ------------------------------------------------------
#
# Helpful functions
# ------------------------------------------------------

def loadYAMLConfig(file_name):
    with open(file_name, 'r') as configfile:
         config = yaml.load(configfile, Loader=yaml.FullLoader)
    return config

# ------------------------------------------------------
#
# PyBullet interface
# ------------------------------------------------------

def initPyBullet(time_step):
    pybullet.connect(pybullet.GUI)
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0) # this removes the side menus
    pybullet.resetSimulation()
    pybullet.setTimeStep(time_step)

def setupPyBulletCamera(distance, yaw, pitch, target_position):
    pybullet.resetDebugVisualizerCamera(
        cameraDistance=distance,
        cameraYaw=yaw,
        cameraPitch=pitch,
        cameraTargetPosition=target_position
    )

def stepPyBullet():
    pybullet.stepSimulation()

def closePyBullet():
    pybullet.disconnect()

class PyBulletObject:

    def loadFromURDF(self, urdf_file_name, base_position, use_fixed_base):
        self.ID = pybullet.loadURDF(urdf_file_name, basePosition=base_position, useFixedBase=use_fixed_base)

class PyBulletSceneObject(PyBulletObject):

    def __init__(self, file_name, config):
        if file_name.endswith('.urdf'):
            self.loadFromURDF(file_name, config['base_position'], config['use_fixed_base'])
        elif file_name.endswith('.obj'):
            self.loadFromOBJ(file_name, config)
        self.changeDynamics(config)

    def loadFromOBJ(self, file_name, config):
        # pybullet.createVisualShape()
        # pybullet.createCollisionShape()
        # pybullet.createMultiBody()
        raise NotImplementedError("")

    def changeDynamics(self, config):
        # pybullet.changeDynamics(bodyUniqueId=self.ID)
        raise NotImplementedError("")

class PyBulletRobot(PyBulletObject):

    def __init__(self, urdf_file_name, base_position, use_fixed_base):
        self.loadFromURDF(urdf_file_name, base_position, use_fixed_base)
        self.joint_id = []
        self.joint_name = []
        self.num_joints = pybullet.getNumJoints(self.ID)
        self.link_names = []
        for i in range(self.num_joints):
            info = pybullet.getJointInfo(self.ID, i)
            self.link_names.append(info[12].decode('utf-8'))
            if info[2] in {pybullet.JOINT_REVOLUTE, pybullet.JOINT_PRISMATIC}:
                self.joint_id.append(info[0])
                self.joint_name.append(info[1].decode("utf-8"))
        self.ndof = len(self.joint_id)

    def resetJointPosition(self, position):
        for i in range(self.ndof):
            pybullet.resetJointState(self.ID, i, position[i])

    def getJointName(self):
        return self.joint_name

    def getJointPosition(self):
        return [joint_state[0] for joint_state in pybullet.getJointStates(self.ID, self.joint_id)]

    def getJointVelocity(self):
        return [joint_state[1] for joint_state in pybullet.getJointStates(self.ID, self.joint_id)]

    def getJointMotorTorque(self):
        return [joint_state[3] for joint_state in pybullet.getJointStates(self.ID, self.joint_id)]

    def getLinkStates(self):
        states = [None]*self.num_joints
        for i in range(self.num_joints):
            state = pybullet.getLinkState(self.ID, i, computeLinkVelocity = 0, computeForwardKinematics = 1)
            states[i] = {
                'label': self.link_names[i],
                'position': state[4],
                'orientation': state[5],
            }
        return states

    def commandJointPosition(self, target_position):
        pybullet.setJointMotorControlArray(
            self.ID,
            self.joint_id,
            pybullet.POSITION_CONTROL,
            targetPositions = target_position,
        )

# ------------------------------------------------------
#
# ROS/PyBullet interface
# ------------------------------------------------------

class ROSPyBulletInterface:

    def __init__(self):

        # Setup constants
        self.dt = 1.0/float(FREQ)
        self.tfBroadcaster = tf2_ros.TransformBroadcaster()

        # Get ros parameters
        robot_config_file_name = rospy.get_param('~robot_config')
        camera_config_file_name = rospy.get_param('~camera_config')

        # Initialise pybullet
        initPyBullet(self.dt)

        # Setup pybullet
        self.setupPyBulletRobot(robot_config_file_name)
        self.setupPyBulletCamera(camera_config_file_name)

        # Setup ros publishers/tf broadcasters
        self.joint_state_publisher = rospy.Publisher(CURRENT_JOINT_STATE_TOPIC, JointState, queue_size=10)

        # Setup ros subscriber
        rospy.Subscriber(TARGET_JOINT_STATE_TOPIC, JointState, self.readTargetJointStateFromROS)

        # Main pybullet update
        rospy.Timer(self.dur, self.updatePyBullet)

    def setupPyBulletRobot(self, config_file_name):

        # Load robot configuration
        config = loadYAMLConfig(config_file_name)

        # Extract required data from configuration
        urdf_file_name = config['urdf']
        use_fixed_base = config['use_fixed_base']
        base_position = config['base_position']
        target_joint_position = config['init_position']

        # Create pybullet robot instance
        self.robot = PyBulletRobot(urdf_file_name, base_position, use_fixed_base)

        # Reset joint state
        self.robot.resetJointPosition(target_joint_position)
        self.target_joint_position = target_joint_position

        # Setup ros timers
        rospy.Timer(self.dur, self.publishPyBulletJointStateToROS)
        rospy.Timer(self.dur, self.publishPybulletLinkStatesToROS)
    def setupPyBulletCamera(self, config_file_name):

        # Load camera config
        config = loadYAMLConfig(config_file_name)

        # Extract data from configuration
        setupPyBulletCamera(
            config['distance'],
            config['yaw'],
            config['pitch'],
            config['target_position']
        )

    def readTargetJointStateFromROS(self, msg):
        self.target_joint_position = msg.position

    def publishPyBulletJointStateToROS(self, event):
        msg = JointState(
            name = self.robot.getJointName(),
            position = self.robot.getJointPosition(),
            velocity = self.robot.getJointVelocity(),
            effort = self.robot.getJointMotorTorque(),
        )
        msg.header.stamp = rospy.Time.now()
        self.joint_state_publisher.publish(msg)

        msg.header.stamp = rospy.Time.now()
    def publishPybulletLinkStatesToROS(self, event):

        for state in self.robot.getLinkStates():
            position = state['position']
            orientation = state['orientation']

            # Pack pose msg
            msg = TransformStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = WORLD_FRAME_ID
            msg.child_frame_id = 'ros_pybullet_interface/robot/'+state['label']
            msg.transform.translation.x = position[0]
            msg.transform.translation.y = position[1]
            msg.transform.translation.z = position[2]
            msg.transform.rotation.x = orientation[0]
            msg.transform.rotation.y = orientation[1]
            msg.transform.rotation.z = orientation[2]
            msg.transform.rotation.w = orientation[3] # NOTE: the ordering here may be wrong

            # Broadcast tf
            self.tfBroadcaster.sendTransform(msg)

    def updatePyBullet(self, event):
        self.robot.commandJointPosition(self.target_joint_position)
        stepPyBullet()

# ------------------------------------------------------
#
# Main program
# ------------------------------------------------------

if __name__=='__main__':
    rospy.init_node('ros_pybullet_interface', anonymous=True)
    ROSPyBulletInterface()
    rospy.on_shutdown(closePyBullet)
    rospy.spin()
