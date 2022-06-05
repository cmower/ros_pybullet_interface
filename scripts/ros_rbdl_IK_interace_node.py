#!/usr/bin/env python3
# license removed for brevity
import rospkg
import rospy
import os

import rbdl
import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation as R

# ROS message types
from sensor_msgs.msg import JointState


# ROS messages types of the real robot
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension


# from ros_pybullet_interface.utils import loadYAMLConfig
import ros_pybullet_interface.utils as utils

# ------------------------------------------------------
#
# Constants
# ------------------------------------------------------

FREQ = 50 # IK sampling frequency
TARGET_JOINT_STATE_TOPIC = 'ros_pybullet_interface/joint_state/target' # publishes  for joint states on this topic
CURRENT_JOINT_STATE_TOPIC = 'ros_pybullet_interface/joint_state/current' # listens joint states on this topic
WORLD_FRAME_ID = 'ros_pybullet_interface/world'
END_EFFECTOR_TARGET_FRAME_ID = 'ros_pybullet_interface/end_effector/target' # listens for end-effector poses on this tf id
ROBOT_BASE_ID = "ros_pybullet_interface/robot/robot_base" # listen for the pose of the robot base


EEBodyPointPosition = np.array([0.0, 0.0, -0.0]) #np.zeros(3)

# ------------------------------------------------------------
#  REAL ROBOT
# ------------------------------------------------------------
# CURRENT_JOINT_STATE_TOPIC = 'joint_states' # publishes joint states on this topic
# test on robot and compare with commit:
# https://github.com/cmower/ros_pybullet_interface/commit/e3ce90b58998ea3078abda3f5733db883448bb6e
# NDOF = 7
# REAL_ROBOT_TARGET_JOINT_STATE_TOPIC = 'sim/PositionController/command' # commands joint states on this topic


class PyRBDLRobot:

    def __init__(self, file_name, end_effector_name, base_position, base_orientation_quat, q0):

        # Load Robot rbdl model
        file_name = utils.replacePackage(file_name)
        self.rbdlModel = rbdl.loadModel(file_name.encode('utf-8'), verbose = False, floating_base = True)

        # Get end-effector body for rbdl
        self.rbdlEndEffectorID = rbdl.Model.GetBodyId(self.rbdlModel, end_effector_name)

        # IK Variables initialization
        self.numJoints = self.rbdlModel.qdot_size
        self.rbdlEEBodyPointPosition = EEBodyPointPosition
        self.qBasePos = np.array(base_position)
        self.qBaseOrientQuat = np.array(base_orientation_quat)
        # old implementation with Euler angles loaded from file
        # ori = R.from_euler('xyz', base_orient_eulerXYZ, degrees=True)
        # self.qBaseOrientQuat = rbdl.Quaternion.toNumpy(rbdl.Quaternion.fromPythonMatrix(ori.as_matrix()))
        self.qInitial = np.array(q0)

        # ---- place robot to the base position and orientation

        # Create numpy arrays for the state
        self.q = np.zeros(self.rbdlModel.q_size)
        # Modify the state to place robot in appropriate position and orientation
        self.q[0:3] = self.qBasePos
        # following the convection of RBDL, the vector of the quaternion is in
        # slide 3:6 of the configuration (for floating base)
        # and the scalar part of the quaternion is last after all the joints
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
        self.q[0:3] = posNew

    def updateBaseOrientationEuler(self, orient_eulerXYZNew):
        ori_mat = np.asarray(XYZeuler2RotationMat(orient_eulerXYZNew))
        qBaseOrientQuatNew = rbdl.Quaternion.toNumpy(rbdl.Quaternion.fromPythonMatrix(ori_mat))
        self.q[3:6] = qBaseOrientQuatNew[0:3]
        self.q[-1] = qBaseOrientQuatNew[3]

    def updateBaseOrientationQuat(self, orient_quat):
        self.q[3:6] = orient_quat[0:3]
        self.q[-1] = orient_quat[3]

    def getJointConfig(self):
        return self.q[6:self.numJoints]

    def getJointName(self):
        return self.joint_name


    ### --- why use two different function for end-effector position and orientation
    def getCurEEPos(self):
        return rbdl.CalcBodyToBaseCoordinates(self.rbdlModel, self.q, self.rbdlEndEffectorID, EEBodyPointPosition)

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

    def __init__(self, time_step, file_name, end_effector_name, base_position, base_orientation_quat, q0, ik_info):
        self.robot = PyRBDLRobot(file_name, end_effector_name, base_position, base_orientation_quat, q0)
        self.dt = time_step

        # task indexes to switch from full 6D to only position or orienation
        self.f_indx = ik_info['f_indx']
        self.l_indx = ik_info['l_indx']

        # weights for tracking between orientation axes
        self.ori_axis_weights = ik_info['ori_axis_weights']
        # Find the unique elements of an array
        unique, index, counts = np.unique(np.array(self.ori_axis_weights), return_index=True, return_counts=True)
        # get the index of the elemnt that is 1, if it exist
        self.axis_idx = -1
        if (counts[unique==1.0]==1.0):
            self.axis_idx = index[unique==1.0][0]

        # set the function that computes the orienation error
        # error based on frames matching
        self.computeOrientationDelta = self.computeOrientationDelta3D
        # error based on vector matching
        # (match Z axis of robot with axis with index 'axis_idx')
        if self.axis_idx != -1:
            self.computeOrientationDelta = self.computeOrientationDelta1axis

        # scaling parameter between orientation and position
        self.scale_pos_orient = ik_info['scale_pos_orient']

        # parameters of the nullspace motion
        self.alpha_null = np.array(ik_info['alpha_null'])
        self.h_delta = np.deg2rad(np.array(ik_info['h_delta']))
        self.h_norm = np.deg2rad(np.array(ik_info['h_norm']))


    def fullDiffIKStep(self, globalTargetPos3D, globalTargetOri3D):
        delta = self.computeDelta(globalTargetPos3D, globalTargetOri3D)
        JG = self.computeGlobalJacobian()
        self.fullDiffIK(JG, delta)

    """ --------------- Functions made for self use -------------------------"""
    def computeOrientationDelta3D(self, orientA, orientB):
        # least square error between two frames
        error = R.align_vectors(np.transpose(orientA.as_matrix()), orientB.as_matrix(), self.ori_axis_weights)[0]
        return error.as_rotvec()

    def computeOrientationDelta1axis(self, orientA, orientB):
        # compute error only between a pair fo vectors
        zAxisTarget = orientA.as_matrix()[:,self.axis_idx] # get target axis vector
        zAxisCurrent = orientB.as_matrix()[2, :]           # get current Z axis vector
        axis = np.cross(zAxisCurrent, zAxisTarget)
        vectdot_align = zAxisTarget.dot(zAxisCurrent)
        # we clip the value of the dot product to ensure that it's between -1 to 1
        # as the vectors fo the dot product are normalized
        angle = np.arccos(np.clip(vectdot_align, -1.0, 1.0))

        if np.isnan(angle).any():
            rospy.logerr(" The is something wrong with orientantion of the IK.")

        return  axis*angle

    def computeDelta(self, globalTargetPos3D, globalTargetOri3D):
        # retrieve global position of the end-effector
        global_eePos = self.robot.getCurEEPos()
        # position error
        dpos = globalTargetPos3D - global_eePos

        # compute global orientation of the end-effector
        global_eeOri_mat = self.robot.getCurEEOri()
        global_eeOri = R.from_matrix(global_eeOri_mat)

        # globalTargetOri3D is a matrix form
        global_eeOriTarget = R.from_matrix(globalTargetOri3D)

        # compute orientation error
        dori = self.computeOrientationDelta(global_eeOriTarget, global_eeOri)

        # get delta orientation as a vector + also scale it
        doriVec = dori * self.scale_pos_orient

        # position and orientation (in euler angles) error
        # we stack first angular error and then position error because of the RBDL jacobian form (see below)
        # Computes the 6-D Jacobian $G(q)$ that when multiplied with $\dot{q}$ gives a 6-D vector
        # that has the angular velocity as the first three entries and the linear velocity as the last three entries.
        delta = np.concatenate((doriVec, dpos), axis=None)

        return delta

    def computeGlobalJacobian(self):
         # Compute Jacobian
        JG = np.zeros([6, self.robot.numJoints])
        rbdl.CalcPointJacobian6D(self.robot.rbdlModel, self.robot.q, self.robot.rbdlEndEffectorID, self.robot.rbdlEEBodyPointPosition, JG, update_kinematics=True)
        return JG

    def fullDiffIK(self, J, delta):

         # Compute Jacobian pseudo-inverse
        Jpinv = np.linalg.pinv(J)

        # select joints and dimensions
        J_arm = J[self.f_indx:self.l_indx,6:self.robot.numJoints]
        Jpinv_arm = Jpinv[6:self.robot.numJoints,self.f_indx:self.l_indx]
        # Compute differential kinematics
        dq = Jpinv_arm.dot(delta[self.f_indx:self.l_indx])

        _, c = J_arm.shape
        # retrieve current configuration
        qprev = self.robot.getJointConfig()

        # compute null-space component
        dq_nullspace_motion = (np.eye(c)-Jpinv_arm @ J_arm) @ ((self.h_delta-qprev)*(1/self.h_norm**2))

        if (abs(dq) > np.deg2rad(5)).any():
            print("---------------------------------------------------------------")
            print(" BIG STEP IN IK ")
            print("---------------------------------------------------------------")

        # compute new configuration
        qnext = qprev + dq + self.alpha_null*dq_nullspace_motion

        #  update configuration
        self.robot.updateJointConfig(qnext)


class ROSdIKInterface(object):

    def __init__(self):

        # Setup constants
        self.dt = 1.0/float(FREQ)

        # listen to tf topic for end effector targets
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
        # real_world_publishers_topic_name = f"{self.robot_name}/{REAL_ROBOT_TARGET_JOINT_STATE_TOPIC}"
        # self.real_world_target_joint_state_publisher = rospy.Publisher(real_world_publishers_topic_name, Float64MultiArray, queue_size=1)



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

        
    # def publishdIKJointStateToROS2RealWorld(self, event):
    #     # Pack trajectory msg
    #     msg = Float64MultiArray()
    #     msg.layout.dim.append(MultiArrayDimension())
    #     # info for reconstruction of the 2D array
    #     msg.layout.dim[0].label  = "columns"
    #     msg.layout.dim[0].size   = NDOF
    #
    #     position =  self.robotIK.robot.getJointConfig()
    #
    #     # add data as flattened numpy array
    #     msg.data = position.flatten('C') # row major flattening
    #
    #     # msg.header.stamp = rospy.Time.now()
    #     self.real_world_target_joint_state_publisher.publish(msg)

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

        # --------------------------------------------------
        # start the callback for the real world
        # ROSdIKinterface.writeCallbackTimer = rospy.Timer(dur, ROSdIKinterface.publishdIKJointStateToROS2RealWorld)

        # Ctrl-C will stop the script
        rospy.on_shutdown(ROSdIKinterface.cleanShutdown)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
