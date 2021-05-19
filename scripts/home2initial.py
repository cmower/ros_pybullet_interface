#!/usr/bin/env python3
import sys
import math
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import tf2_ros
import rospy
from geometry_msgs.msg import TransformStamped

r = 0.1 # radius
WORLD_FRAME_ID = 'ros_pybullet_interface/world'
END_EFFECTOR_TARGET_FRAME_ID = 'ros_pybullet_interface/end_effector/target' # publish for end-effector poses on this topic

# ------------------------------------------------------
#
# Constants
# ------------------------------------------------------

FREQ = 100 # IK sampling frequency
TARGET_JOINT_STATE_TOPIC = 'ros_pybullet_interface/joint_state/target' # listens for joint states on this topic
CURRENT_JOINT_STATE_TOPIC = 'ros_pybullet_interface/joint_state/current' # publishes joint states on this topic
WORLD_FRAME_ID = 'ros_pybullet_interface/world'
END_EFFECTOR_TARGET_FRAME_ID = 'ros_pybullet_interface/end_effector/target' # listens for end-effector poses on this topic
ROBOT_BASE_ID = "ros_pybullet_interface/robot/robot_base" # listen for the pose of the robot base

EEBodyPointPosition = np.array([0.0, 0.0, 0.0]) #np.zeros(3)

# ------------------------------------------------------------
#  REAL ROBOT
# ------------------------------------------------------------

CURRENT_JOINT_STATE_TOPIC = 'joint_states' # publishes joint states on this topic
REAL_ROBOT_TARGET_JOINT_STATE_TOPIC = 'PositionController/command' # commands joint states on this topic


class ROSdIKInterface(object):

    def __init__(self):

        # Setup constants
        self.dt = 1.0/float(FREQ)

        self.IK_listen_buff = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.IK_listen_buff)

        # Name of node
        self.name = rospy.get_name()
        # Initialization message
        rospy.loginfo("%s: Initializing class", self.name)

        # get the path to this catkin ws
        self.current_dir = utils.ROOT_DIR

        # Get ros parameters
        robot_config_file_name = utils.replacePackage(rospy.get_param('~robot_config'))

        #  PyRBDLRobot
        self.robot_name = self.setupPyRBDLRobot(robot_config_file_name)

        # Setup ros publishers
        publishers_topic_name = f"{self.robot_name}/{TARGET_JOINT_STATE_TOPIC}"
        self.target_joint_state_publisher = rospy.Publisher(publishers_topic_name, JointState, queue_size=1)

        # initialization of the Forward Kinematics
        self.target_EE_position = self.robotIK.robot.getCurEEPos()
        curOri = R.from_matrix(self.robotIK.robot.getCurEEOri())
        self.target_EE_orientation = np.transpose(curOri.as_matrix())

        # Setup ros publishers
        real_world_publishers_topic_name = f"{self.robot_name}/{REAL_ROBOT_TARGET_JOINT_STATE_TOPIC}"
        self.real_world_target_joint_state_publisher = rospy.Publisher(real_world_publishers_topic_name, Float64MultiArray, queue_size=1)



    def setupPyRBDLRobot(self, config_file_name):

        # Load robot configuration
        config = utils.loadYAMLConfig(config_file_name)

        # Extract data from configuration
        file_name = config['file_name']
        robot_name = config['name']
        end_effector_name = config['end_effector']
        use_fixed_base = config['use_fixed_base']
        ik_info = config['IK']

        # Establish connection with Robot in PyBullet/Real world via ROS
        rospy.loginfo(f"{self.name}: Waiting for {robot_name}/{CURRENT_JOINT_STATE_TOPIC} topic, to read the curent configuration of the robot.")
        msgRobotState = rospy.wait_for_message(f"{robot_name}/{CURRENT_JOINT_STATE_TOPIC}", JointState)
        # set robot to the curent configuration obtained from ros topic
        init_joint_position = list(msgRobotState.position)

        rospy.loginfo(f"{self.name}: Reading for /tf topic the position and orientation of the robot")
        # Loop till the position and orientation of the base the robots has been read.
        while 1:
            try:
                # Read the position and orientation of the robot from the /tf topic
                trans = self.IK_listen_buff.lookup_transform(WORLD_FRAME_ID, f"{robot_name}/{ROBOT_BASE_ID}", rospy.Time())
                break
            except:
                rospy.logwarn(f"{self.name}: /tf topic does NOT have {robot_name}/{ROBOT_BASE_ID}")
        # replaces base_position = config['base_position']
        base_position = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
        # replaces: base_orient_eulerXYZ = config['base_orient_eulerXYZ']
        base_orient_quat = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
        # base_orient_eulerXYZ = config['base_orient_eulerXYZ']

        # Create pybullet robot instance
        self.robotIK = PyRBDL4dIK(self.dt, file_name, end_effector_name, base_position, base_orient_quat, init_joint_position, ik_info)

        return robot_name

    def publishdIKJointStateToROS(self, event):
        msg = JointState(
            name = self.robotIK.robot.getJointName(),
            position =  self.robotIK.robot.getJointConfig(),
            # velocity = self.robotIK.robot.getJointConfig(),
            # effort = self.robotIK.robot.getJointConfig(),
        )
        msg.header.stamp = rospy.Time.now()
        self.target_joint_state_publisher.publish(msg)

    def publishdIKJointStateToROS2RealWorld(self, event):
        # Pack trajectory msg
        msg = Float64MultiArray()
        msg.layout.dim.append(MultiArrayDimension())
        # info for reconstruction of the 2D array
        msg.layout.dim[0].label  = "columns"
        msg.layout.dim[0].size   = 7

        position =  self.robotIK.robot.getJointConfig()

        # add data as flattened numpy array
        msg.data = position.flatten('C') # row major flattening

        # msg.header.stamp = rospy.Time.now()
        self.real_world_target_joint_state_publisher.publish(msg)

    def startListening2EETargets(self):
        # Subscribe target end-effector callback
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.Timer(rospy.Duration(self.dt), self.readTargetEEStateFromTF)

    def readTargetEEStateFromTF(self, event):
        try:
            tf = self.tfBuffer.lookup_transform(WORLD_FRAME_ID, f"{self.robot_name}/{END_EFFECTOR_TARGET_FRAME_ID}", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return
        self.target_EE_position = np.asarray([tf.transform.translation.x, tf.transform.translation.y,tf.transform.translation.z])
        target_EE_ori = R.from_quat(np.asarray([tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w]))
        self.target_EE_orientation = target_EE_ori.as_matrix()

    def updateRBDL(self, event):
        self.robotIK.fullDiffIKStep(self.target_EE_position, self.target_EE_orientation)

    def cleanShutdown(self):
        print('')
        rospy.loginfo("%s: Sending to safe configuration", self.name)
        # Shut down write callback
        self.writeCallbackTimer.shutdown()
        # Send to current configuration
        msg = JointState(
            name = self.robotIK.robot.getJointName(),
            position =  self.robotIK.robot.getJointConfig(),
            velocity = self.robotIK.robot.getJointConfig()*0,
            effort = self.robotIK.robot.getJointConfig()*0,
        )
        msg.header.stamp = rospy.Time.now()
        self.target_joint_state_publisher.publish(msg)
        rospy.sleep(1.0)



# ------------------------------------------------------------
#  REAL ROBOT
# ------------------------------------------------------------

CURRENT_JOINT_STATE_TOPIC = 'joint_states' # publishes joint states on this topic
REAL_ROBOT_TARGET_JOINT_STATE_TOPIC = 'PositionController/command' # commands joint states on this topic


if __name__ == '__main__':
    try:
        rospy.sleep(1.0)
        # Initialize node
        rospy.init_node("home2initial", anonymous=True)
        # Initialize node class
        ROSdIKinterface = ROSdIKInterface()

        # Establish connection with end-effector commander
        rospy.loginfo("%s: Setup target reader.", ROSdIKinterface.name)
        ROSdIKinterface.startListening2EETargets()

        # Create timer for periodic publisher
        dur = rospy.Duration(ROSdIKinterface.dt)
        rospy.Timer(dur, ROSdIKinterface.updateRBDL)
        ROSdIKinterface.writeCallbackTimer = rospy.Timer(dur, ROSdIKinterface.publishdIKJointStateToROS)

        # --------------------------------------------------
        # start the callback for the real world
        ROSdIKinterface.writeCallbackTimer = rospy.Timer(dur, ROSdIKinterface.publishdIKJointStateToROS2RealWorld)

        # Ctrl-C will stop the script
        rospy.on_shutdown(ROSdIKinterface.cleanShutdown)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
