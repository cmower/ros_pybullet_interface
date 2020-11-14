#!/usr/bin/env python
# license removed for brevity
import rospkg
import rospy
import rbdl

import numpy as np
from utils import loadYAMLConfig

# ROS message types
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
# from pyquaternion import Quaternion
# ## from std_msgs.msg import Float64
# from geometry_msgs.msg import WrenchStamped
# from std_msgs.msg import Float64MultiArray
# from ipab_lwr_msgs.msg import FriState
# from ipab_lwr_msgs.msg import FriCommandCartesianStiffness
# from ipab_lwr_msgs.msg import FriCommandJointPosition
# from ipab_lwr_msgs.msg import FriCommandMode


# ------------------------------------------------------
#
# Constants
# ------------------------------------------------------

FREQ = 100 # IK sampling frequency
TARGET_JOINT_STATE_TOPIC = 'ros_pybullet_interface/joint_state/target' # listens for joint states on this topic
CURRENT_JOINT_STATE_TOPIC = 'ros_pybullet_interface/joint_state/current' # publishes joint states on this topic
TARGET_END_EFFECTOR_TOPIC = 'ros_pybullet_interface/end_effector/target' # publishes end-effector poses on this topic
CURRENT_END_EFFECTOR_TOPIC = 'ros_pybullet_interface/end_effector/current' # publishes end-effector poses on this topic

EEBodyPointPosition = np.zeros(3)


class PyRBDLRobot:

    def __init__(self, urdf_file_name, end_effector_name, base_position, base_orient_mat, q0):

        # Load Robot rbdl model
        self.rbdlModel = rbdl.loadModel(urdf_file_name.encode('utf-8'), verbose = False, floating_base = True)

        # Get end-effector body for rbdl
        self.rbdlEndEffectorID = rbdl.Model.GetBodyId(self.rbdlModel, end_effector_name)

        # IK Variables initialization
        self.numJoints = self.rbdlModel.qdot_size
        self.rbdlEEBodyPointPosition = EEBodyPointPosition
        self.qBasePos = np.array(base_position)
        self.qBaseOrientQuat = rbdl.Quaternion.toNumpy(rbdl.Quaternion.fromPythonMatrix(np.asarray(base_orient_mat)))
        self.qInitial = np.array(q0)

        # ---- place robot to the base position and orientation

        # Create numpy arrays for the state
        self.q = np.zeros(self.rbdlModel.q_size)
        # Modify the state to place robot in appropriate position and orientation
        self.q[0:3] = self.qBasePos
        self.q[3:6] = self.qBaseOrientQuat[0:3]
        self.q[-1] = self.qBaseOrientQuat[3]
        self.q[6:self.numJoints] = self.qInitial

        def updateJointConfig(self, qNew):
            self.q[6:self.numJoints] = qNew

        def updateBasePos(self, posNew):
            self.q[0:3] = pos

        def updateBaseOrient(self, orient_matNew):
            qBaseOrientQuatNew = rbdl.Quaternion.toNumpy(rbdl.Quaternion.fromPythonMatrix(np.asarray(orient_matNew)))
            self.q[3:6] = qBaseOrientQuatNew[0:3]
            self.q[-1] = qBaseOrientQuatNew[3]

        # ---------------------------------------------------------------------#
        # Test funtion
        # ---------------------------------------------------------------------#
        def testFK4baseNewPoseOrient4LWR(self):
            ''' Test forward kinematics, when the the position, orientation and
            configuration of the robot is changed. '''
            # Get first link body of the LWR arm rbdl
            body_base_ID = rbdl.Model.GetBodyId(self.rbdlModel, "lwr_arm_0_link")

            # Transform coordinates from local to global coordinates
            # define a local point w.r.t to a body
            point_local = np.array([0, 0., 0.])
            print(" Local position of the point ", point_local)
            global_point_base = rbdl.CalcBodyToBaseCoordinates (self.rbdlModel, self.q, body_base, point_local)
            print(" Global position of the point ", global_point_base)


class PyRBDL4dIK(PyRBDLRobot):
    def __init__(self, time_step):
        self.dt = time_step

    def ComputeDelta(self, qprev):
        delta = numpy.concatenate((dpos,dori), axis=None)
        return delta

    def ComputeGlobalJacobian(self, qprev):
         # Compute Jacobian
        JG = np.zeros([6, self.numJoints])
        rbdl.CalcPointJacobian6D(self.rbdlModel, qprev, self.rbdlEndEffectorID, self.rbdlEEBodyPointPosition, JG, update_kinematics=True)
        return JG

    def FullDiffIK(self, J, delta, qprev):
         # Compute Jacobian pseudo-inverse
        Jpinv = np.linalg.pinv(J)
        # Compute differential kinematics
        dq = Jpinv_x.dot(delta)
        qnext = qprev + dq.dot(self.dt)#.flatten()
        return qnext

    def FullDiffIKstep(self, J, delta, qprev):
        a = s


            # a = rbdl.CalcBodyToBaseCoordinates(self.rbdlModel, np.array(q0), self.rbdlEndEffectorID, self.rbdlEEBodyPointPosition, update_kinematics=True)

        # JL = numpy.zeros([6, self.numJoints])
        # rbdl.CalcBodySpatialJacobian(self.rbdlModel, self.qprev, self.rbdlEndEffectorID, self.rbdlBodyPointPosition, JL, update_kinematics=False)

    # Compute position and orientation of end-effector and force sensor
            # G_pos_Contact = rbdl.CalcBodyToBaseCoordinates(self.rbdlModel, self.qprev, self.rbdlEndEffectorID, self.rbdlBodyPointPosition, update_kinematics=False)
            # G_R_Contact = rbdl.CalcBodyWorldOrientation(self.rbdlModel, self.qprev, self.rbdlEndEffectorID, update_kinematics=False)


class ROSdIKInterface(object):

    def __init__(self):

        # Setup constants
        self.dt = 1.0/float(FREQ)

        # Name of node
        self.name = rospy.get_name()
        # Initialization message
        rospy.loginfo("%s: Initializing class", self.name)

        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()

        # get the path to this catkin ws
        current_dir = rospack.get_path('ros_pybullet_interface')

        # Get ros parameters
        robot_config_file_name = rospy.get_param('~robot_config')

        #  PyRBDLRobot
        self.setupPyRBDLRobot(current_dir, robot_config_file_name)


        # Setup ros publishers
        self.target_joint_state_publisher = rospy.Publisher(TARGET_JOINT_STATE_TOPIC, JointState, queue_size=1)

        # Setup ros subscriber
        rospy.Subscriber(TARGET_END_EFFECTOR_TOPIC, PoseStamped, self.readTargetEEStateFromROS)
        rospy.Subscriber(CURRENT_JOINT_STATE_TOPIC, JointState, self.readCurrentJointStateFromROS)


         # Initialize command message
        self.msg_out = JointState()
        self.msg_out.jointPosition = np.zeros(self.ndof)

        # # Maximum end effector velocities
        # self.drMAX = 0.2
        # self.dfzMAX = 0.01
        # self.doriMAX = 1.5
        #
        # # Initialize task space velocity vector
        # self.vel = np.zeros(2)
        #
        # # Flag for running in task space global
        # self.globalTaskSpace = True


    def setupPyRBDLRobot(self, current_dir, config_file_name):

        # Load robot configuration
        config = loadYAMLConfig(current_dir + config_file_name)

        # Extract data from configuration
        urdf_file_name = current_dir + config['urdf']
        end_effector_name = config['end_effector']
        use_fixed_base = config['use_fixed_base']
        base_position = config['base_position']
        base_orient_mat = config['base_orient_mat']
        init_joint_position = config['init_position']

        # Create pybullet robot instance
        self.robot = PyRBDLRobot(urdf_file_name, end_effector_name, base_position, base_orient_mat, init_joint_position)



    def publishdIKJointStateToROS(self, event):
        # msg = JointState(
        #     name = self.robot.getJointName(),
        #     position =  self.robot.getJointPosition(),
        #     velocity = self.robot.getJointVelocity(),
        #     effort = self.robot.getJointMotorTorque(),
        # )
        msg.header.stamp = rospy.Time.now()
        self.joint_state_publisher.publish(msg)

    def readTargetEEStateFromROS(self, msg):
        self.target_EE_position = msg.pose.position
        self.target_EE_orientation = msg.pose.orientation

    def readCurrentJointStateFromROS(self, msg):
        self.current_joint_position = msg.position

    def updateRBDL(self, event):
        self.robot.compute(self.target_EE_position, self.target_EE_orientation)


    # def cleanShutdown(self):
    #     print('')
    #     rospy.loginfo("%s: Sending to safe configuration", self.name)
    #     # Shut down write callback
    #     self.writeCallbackTimer.shutdown()
    #     # Send to safe configuration
    #     # TODO: Actually here the safe configuration should depend on the current configuration
    #     qSafe = self.q + np.asarray([0.0, 0.0873, 0.0, 0.0, 0.0, 0.0, 0.0])
    #     self.msg_out.jointPosition = qSafe
    #     self.msg_out.header.stamp = rospy.get_rostime()
    #     # Publish configuration
    #     self.pubState.publish(self.msg_out)
    #     rospy.sleep(1.0)


if __name__ == '__main__':
    try:
        # Initialize node
        rospy.init_node("ros_rbdl_IK_interface", anonymous=True)
        # Initialize node class
        ROSdIKinterface = ROSdIKInterface()

        # Establish connection with Robot in PyBullet environment
        rospy.loginfo("%s: Waiting for /kuka_lwr_state topic", ROSdIKinterface.name)
        # change.....
        msgRobotState = rospy.wait_for_message("/kuka_lwr_state", FriState)
        # ROSdIKinterface.changeMode(msgRobotState)

        # Establish connection with end-effector commander
        rospy.loginfo("%s: Waiting for /operator_signal/velocity", ROSdIKinterface.name)
        # change.....
        msgJoystick = rospy.wait_for_message("/operator_signal/velocity", Float64MultiArray)
        # ROSdIKinterface.startJoystick(msgJoystick)

        # Create timer for periodic publisher
        dur = rospy.Duration(ROSdIKinterface.dt)
        rospy.Timer(dur, ROSdIKinterface.updateRBDL)
        ROSdIKinterface.writeCallbackTimer = rospy.Timer(dur, ROSdIKinterface.publishdIKJointStateToROS)
        # ROSdIKinterface.writeCallbackTimer = rospy.Timer(dur, my_node.writePosCallback)
        # Ctrl-C will stop the script
        rospy.on_shutdown(my_node.cleanShutdown)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
