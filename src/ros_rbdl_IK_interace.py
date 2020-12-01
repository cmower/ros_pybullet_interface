#!/usr/bin/env python3
# license removed for brevity
import rospkg
import rospy
import sys
import os

import rbdl
import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation as R

# ROS message types
from sensor_msgs.msg import JointState

sys.path.append(
    os.path.join(
        rospkg.RosPack().get_path('ros_pybullet_interface'),
        'include'
    )
)
from ros_pybullet_interface_utils import loadYAMLConfig

# ------------------------------------------------------
#
# Constants
# ------------------------------------------------------

FREQ = 100 # IK sampling frequency
TARGET_JOINT_STATE_TOPIC = 'ros_pybullet_interface/joint_state/target' # listens for joint states on this topic
CURRENT_JOINT_STATE_TOPIC = 'ros_pybullet_interface/joint_state/current' # publishes joint states on this topic
CURRENT_END_EFFECTOR_TOPIC = 'ros_pybullet_interface/end_effector/current' # publishes end-effector poses on this topic
WORLD_FRAME_ID = 'ros_pybullet_interface/world'
END_EFFECTOR_TARGET_FRAME_ID = 'ros_pybullet_interface/end_effector/target'
EEBodyPointPosition = np.zeros(3)

scale_dOri = 0.1 # this parameter needs to go to a yaml file

class PyRBDLRobot:

    def __init__(self, file_name, end_effector_name, base_position, base_orient_eulerXYZ, q0):

        # Load Robot rbdl model
        self.rbdlModel = rbdl.loadModel(file_name.encode('utf-8'), verbose = False, floating_base = True)

        # Get end-effector body for rbdl
        self.rbdlEndEffectorID = rbdl.Model.GetBodyId(self.rbdlModel, end_effector_name)

        # IK Variables initialization
        self.numJoints = self.rbdlModel.qdot_size
        self.rbdlEEBodyPointPosition = EEBodyPointPosition
        self.qBasePos = np.array(base_position)
        ori = R.from_euler('xyz', base_orient_eulerXYZ, degrees=True)
        self.qBaseOrientQuat = rbdl.Quaternion.toNumpy(rbdl.Quaternion.fromPythonMatrix(ori.as_matrix()))
        self.qInitial = np.array(q0)

        # ---- place robot to the base position and orientation

        # Create numpy arrays for the state
        self.q = np.zeros(self.rbdlModel.q_size)
        # Modify the state to place robot in appropriate position and orientation
        self.q[0:3] = self.qBasePos
        self.q[3:6] = self.qBaseOrientQuat[0:3]
        self.q[-1] = self.qBaseOrientQuat[3]
        self.q[6:self.numJoints] = self.qInitial

        # build the name list of joints
        self.joint_name = []
        for i in range(len(self.rbdlModel.mJoints)):
            # JointTypeRevolute
            index = self.rbdlModel.mJoints[i].q_index
            # the index 3 indicates the floating base
            if index > 3:
                self.joint_name.append("Joint_"+str(index)+"_"+self.rbdlModel.mJoints[i].mJointType)

    def updateJointConfig(self, qNew):
        self.q[6:self.numJoints] = qNew

    def updateBasePos(self, posNew):
        self.q[0:3] = pos

    def updateBaseOrient(self, orient_eulerXYZNew):
        ori_mat = np.asarray(XYZeuler2RotationMat(orient_eulerXYZNew))
        qBaseOrientQuatNew = rbdl.Quaternion.toNumpy(rbdl.Quaternion.fromPythonMatrix(ori_mat))
        self.q[3:6] = qBaseOrientQuatNew[0:3]
        self.q[-1] = qBaseOrientQuatNew[3]

    def getJointConfig(self):
        return self.q[6:self.numJoints]

    def getJointName(self):
        return self.joint_name

    def getCurEEPos(self):
        return rbdl.CalcBodyToBaseCoordinates(self.rbdlModel, self.q, self.rbdlEndEffectorID, np.array([0., 0., 0.]))

    def getCurEEOri(self):
        return rbdl.CalcBodyWorldOrientation(self.rbdlModel, self.q, self.rbdlEndEffectorID)

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

class PyRBDL4dIK:

    def __init__(self, time_step, file_name, end_effector_name, base_position, base_orient_eulerXYZ, q0):
        self.robot = PyRBDLRobot(file_name, end_effector_name, base_position, base_orient_eulerXYZ, q0)
        self.dt = time_step

    def FullDiffIKstep(self, globalTargetPos3D, globalTargetOri3D):
        delta = self.ComputeDelta(globalTargetPos3D, globalTargetOri3D)
        JG = self.ComputeGlobalJacobian()
        self.FullDiffIK(JG, delta)

    """ --------------- Functions made for self use -------------------------"""

    def ComputeDelta(self, globalTargetPos3D, globalTargetOri3D):
        # retrieve global position of the end-effector
        global_eePos = self.robot.getCurEEPos()
        # position error
        dpos = globalTargetPos3D - global_eePos

        # compute global orientation of the end-effector
        global_eeOri_mat = self.robot.getCurEEOri()
        global_eeOri = R.from_matrix(global_eeOri_mat)
        # Inv_global_eeOri = global_eeOri.inv()

        # globalTargetOri3D is a quaternion in the form x,y,z,w
        global_eeOriTarget = R.from_quat(globalTargetOri3D)
        # Inv_global_eeOriTarget = global_eeOriTarget.inv()

        # orientation error as in quaternions --- it did not work!
        # diff * q1 = q2  --->  diff = q2 * inverse(q1)
        # dori = global_eeOriTarget * Inv_global_eeOri
        # dori = global_eeOri * Inv_global_eeOriTarget

        # least square error between two frames
        dori = R.align_vectors(global_eeOriTarget.as_matrix(), global_eeOri.as_matrix())[0]

        # get delta orientation as a vector + also scale it
        doriVec = dori.as_rotvec() * scale_dOri
        # print(np.linalg.norm(doriVec))

        # position and orientation (in euler angles) error
        #  we stuck first angular error and then position error because of the RBDL jacobian form (see below)
        # Computes the 6-D Jacobian $G(q)$ that when multiplied with $\dot{q}$ gives a 6-D vector
        # that has the angular velocity as the first three entries and the linear velocity as the last three entries.
        delta = np.concatenate((doriVec, dpos), axis=None)

        return delta

    def ComputeGlobalJacobian(self):
         # Compute Jacobian
        JG = np.zeros([6, self.robot.numJoints])
        rbdl.CalcPointJacobian6D(self.robot.rbdlModel, self.robot.q, self.robot.rbdlEndEffectorID, self.robot.rbdlEEBodyPointPosition, JG, update_kinematics=True)
        return JG

    def FullDiffIK(self, J, delta):
         # Compute Jacobian pseudo-inverse
        Jpinv = np.linalg.pinv(J)
        Jpinv_pos = Jpinv[6:self.robot.numJoints,:] # select joints and dimensions
        # Compute differential kinematics
        dq = Jpinv_pos.dot(delta)

        # retrieve current configuration
        qprev = self.robot.getJointConfig()
        # compute new configuration
        qnext = qprev + dq #dot(self.dt)#.flatten()
        #  update configuration
        self.robot.updateJointConfig(qnext)



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
        self.current_dir = rospack.get_path('ros_pybullet_interface')

        # Get ros parameters
        robot_config_file_name = rospy.get_param('~robot_config')

        #  PyRBDLRobot
        self.setupPyRBDLRobot(robot_config_file_name)

        # Setup ros publishers
        self.target_joint_state_publisher = rospy.Publisher(TARGET_JOINT_STATE_TOPIC, JointState, queue_size=1)

        # initialization
        self.target_EE_position = self.robotIK.robot.getCurEEPos()
        curOri = R.from_matrix(self.robotIK.robot.getCurEEOri())
        self.target_EE_orientation = curOri.as_quat()

        # print(self.robotIK.robot.getCurEEPos())
        # riq = R.from_matrix(self.robotIK.robot.getCurEEOri())
        # print(riq.as_rotvec())

    def setupPyRBDLRobot(self, config_file_name):

        # Load robot configuration
        config = loadYAMLConfig(os.path.join(self.current_dir,config_file_name))

        # Extract data from configuration
        file_name = os.path.join(self.current_dir,config['file_name'])
        end_effector_name = config['end_effector']
        use_fixed_base = config['use_fixed_base']
        base_position = config['base_position']
        base_orient_eulerXYZ = config['base_orient_eulerXYZ']
        init_joint_position = config['init_position']

        # Establish connection with Robot in PyBullet environment
        rospy.loginfo("%s: Waiting for "+CURRENT_JOINT_STATE_TOPIC +" topic", self.name)
        msgRobotState = rospy.wait_for_message(CURRENT_JOINT_STATE_TOPIC, JointState)

        # set robot to the curent configuration obtained from pybullet env
        init_joint_position = list(msgRobotState.position)
        self.startListening2JointState(msgRobotState)

        # Create pybullet robot instance
        self.robotIK = PyRBDL4dIK(self.dt, file_name, end_effector_name, base_position, base_orient_eulerXYZ, init_joint_position)

    def publishdIKJointStateToROS(self, event):
        msg = JointState(
            name = self.robotIK.robot.getJointName(),
            position =  self.robotIK.robot.getJointConfig(),
            # velocity = self.robotIK.robot.getJointConfig(),
            # effort = self.robotIK.robot.getJointConfig(),
        )
        msg.header.stamp = rospy.Time.now()
        self.target_joint_state_publisher.publish(msg)

    def startListening2EETargets(self):
        # Subscribe target end-effector callback
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.Timer(rospy.Duration(self.dt), self.readTargetEEStateFromTF)

    def readTargetEEStateFromTF(self, event):
        try:
            tf = self.tfBuffer.lookup_transform(WORLD_FRAME_ID, END_EFFECTOR_TARGET_FRAME_ID, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return
        self.target_EE_position = np.asarray([tf.transform.translation.x, tf.transform.translation.y,tf.transform.translation.z])
        self.target_EE_orientation = np.asarray([tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w])

    def startListening2JointState(self, msg):
        # Setup ros subscriber
        rospy.Subscriber(CURRENT_JOINT_STATE_TOPIC, JointState, self.readCurrentJointStateFromROS)

    def readCurrentJointStateFromROS(self, msg):
        """ PENDING - not used at the moment"""
        # this can be modified to be used as closed loop
        self.current_joint_position = msg.position
        # print(self.robotIK.robot.getJointConfig() - np.asarray(msg.position))

    def updateRBDL(self, event):
        self.robotIK.FullDiffIKstep(self.target_EE_position, self.target_EE_orientation)

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


if __name__ == '__main__':
    try:
        # Initialize node
        rospy.init_node("ros_rbdl_IK_interface", anonymous=True)
        # Initialize node class
        ROSdIKinterface = ROSdIKInterface()

        # Establish connection with end-effector commander
        rospy.loginfo("%s: Setup target reader.", ROSdIKinterface.name)
        ROSdIKinterface.startListening2EETargets()

        # Create timer for periodic publisher
        dur = rospy.Duration(ROSdIKinterface.dt)
        rospy.Timer(dur, ROSdIKinterface.updateRBDL)
        ROSdIKinterface.writeCallbackTimer = rospy.Timer(dur, ROSdIKinterface.publishdIKJointStateToROS)

        # Ctrl-C will stop the script
        rospy.on_shutdown(ROSdIKinterface.cleanShutdown)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
