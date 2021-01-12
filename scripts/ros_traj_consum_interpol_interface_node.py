#!/usr/bin/env python3
# license removed for brevity
import rospkg
import rospy
import os

import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation as R

# ROS message types
from geometry_msgs.msg import TransformStamped

import ros_pybullet_interface.utils as utils

import ros_pybullet_interface.interpolation as interpol

# ------------------------------------------------------
#
# Constants
# ------------------------------------------------------

FREQ = 100 # Resolution of trajectory knots --- sampling frequency

class TrajManager:

    def __init__(self, mot_dim, interpol):


        self.horizonLength  = interpol['horizon_Length']
        self.interFreq = 1.0/interpol['interDt']
        self.use_interp = interpol['use_interpolation']

        # number of dimentions of the trajectory plan
        trajPlanDim = mot_dim["number"]

        # information about the fixed and planned dimensions of the motion
        self.mot_dim = mot_dim

        # init struct for trajectory plan received by a planner
        self.trajPlan = np.zeros((trajPlanDim, 1))

        if self.use_interp:
            self.dTrajPlan = np.zeros((trajPlanDim, 1))
            self.time = np.zeros((1, 1))

        # init struct for interpolated motion plan
        self.motionInterpPlan = np.array([])


    def getNextWayPt(self):

        traj_waypt = self.popFirstTrajElem()

        return self.transTraj2Motion6D(traj_waypt)

    def transTraj2Motion6D(self, way_pt):
        """ We need a function that maps dimensions of the Traj to 6D"""


        # translation
        if self.mot_dim['trans']['translationX'] is not None:
            x = self.mot_dim['trans']['translationX']
        else:
            x = way_pt[self.mot_dim['trans']['translationX_index']]

        if self.mot_dim['trans']['translationY'] is not None:
            y = self.mot_dim['trans']['translationY']
        else:
            y = way_pt[self.mot_dim['trans']['translationY_index']]


        if self.mot_dim['trans']['translationZ'] is not None:
            z = self.mot_dim['trans']['translationZ']
        else:
            z = way_pt[self.mot_dim['trans']['translationZ_index']]

        pos = np.array([x, y, z])

        # rotation
        if self.mot_dim['rotation']['rotationTheta'] == True:
            rot_mat = utils.rotation_matrix_from_axis_angle(np.array(self.mot_dim['rotation']['rotationvec']), way_pt[self.mot_dim['rotation']['rotationTheta_index']])
            Ori_Rot = R.from_matrix(rot_mat)

        else:
            if self.mot_dim['rotation']['rotationvec'] is not None:
                Ori_Rot = R.from_rotvec(np.array(self.mot_dim['rotation']['rotationvec']))
            else:
                idx = self.mot_dim['rotation']['rotationvec_index']
                Ori_Rot = R.from_quat(np.array(way_pt[idx[0]:idx[1]]))

        # eeOri_Rot_rob = R.from_quat(np.array([1.0, 0.0, 0.0,  -0.0]))
        Ori = Ori_Rot.as_quat()

        return np.hstack((pos, Ori))


    # --------------------------------------------------------------------
    #   ATTENTION : popFirstTrajElem and updateTraj need to be thread safe!!!
    #             At the moment they are NOT!!!!!
    #
    #    READ more ...
    #       maybe multithreading
    #    maybe threading
    # use lock:
    # https://www.bogotobogo.com/python/Multithread/python_multithreading_Synchronization_Lock_Objects_Acquire_Release.php
    # --------------------------------------------------------------------

    def popFirstTrajElem(self):
        """ Extract the 1st element of the motion struct to
        send to the simulation"""

        # nextWaypt = self.trajPlan[:,0]
        # self.trajPlan = np.delete(self.trajPlan, 0, 1)
        # self.dTrajPlan = np.delete(self.dTrajPlan, 0, 1)

        # if we use interpolation, we use the interpolated one
        if self.use_interp:

            if self.motionInterpPlan.shape[1] == 0:
                rospy.logerr("All the trajectory data has been consumerd")
                return None

            nextWaypt = self.motionInterpPlan[:,0]
            self.motionInterpPlan = np.delete(self.motionInterpPlan, 0, 1)


        return nextWaypt


    def updateTraj(self):
        # read from file at the moment
        with open('/home/theo/software/OC/Hybrid_MPC/testData1.npy', 'rb') as f:
            data = np.load(f)

        # update the trajectory plan#
        # self.trajPlan = np.hstack((self.trajPlan, np.vstack( (data[0:3,:], data[6:8,:]))))
        # self.dTrajPlan = np.hstack((self.dTrajPlan, np.vstack((data[3:6,:], data[8:10,:]))))
        # self.time = np.hstack((self.time, 0.0001 + data[10,:].reshape(1,data[10,:].shape[0])))

        if self.horizonLength==20:
            self.trajPlan = np.hstack((self.trajPlan,  data[6:8,:]))
            self.dTrajPlan = np.hstack((self.dTrajPlan, data[8:10,:]))
            self.time = np.hstack((self.time, 0.0001 + data[10,:].reshape(1,data[10,:].shape[0])))

        if self.horizonLength==10:
            self.trajPlan = np.hstack((self.trajPlan,  data[0:3,:]))
            self.dTrajPlan = np.hstack((self.dTrajPlan, data[3:6,:]))
            self.time = np.hstack((self.time, 0.0001 + data[10,:].reshape(1,data[10,:].shape[0])))


        if self.use_interp:
            self.motionInterpPlan = self.computeInterpTraj()

        # throw first away
        self.popFirstTrajElem()
        # print(self.motionInterpPlan[:,0:2])
        # assa

    def computeInterpTraj(self):
        """ Compute the interpolated trajectory from the planning traj"""

        tempMotionInterpPlan = np.empty((0))
        trajDim = self.trajPlan.shape[0]

        for i in range(trajDim):
            _, interSeq_I = interpol.interpolateCubicHermiteSplineSourceCode(self.time[0,:], self.trajPlan[i,:], self.dTrajPlan[i,:], sampleFreq=self.interFreq, plotFlag=False, plotTitle="PositionVsTime")
            tempMotionInterpPlan = np.append(tempMotionInterpPlan, interSeq_I, axis=0)

        # reshape to have a dimension per row
        col_len = interSeq_I.shape[0]
        tempMotionInterpPlan = tempMotionInterpPlan.reshape(trajDim, col_len)

        return tempMotionInterpPlan



class ROSTrajInterface(object):

    def __init__(self):

        # Setup constants
        self.dt = 1.0/float(FREQ)

        # default values for TF
        self.header_frame_id = None
        self.msg_child_frame_id = None

        # Name of node
        self.name = rospy.get_name()
        # Initialization message
        rospy.loginfo("%s: Initializing class", self.name)

        # get the path to this catkin ws
        self.current_dir = utils.ROOT_DIR

        # Get ros parameters
        traj_config_file_name = rospy.get_param('~traj_config')

        #  TrajManager
        self.setupTrajManager(traj_config_file_name)

        self.tfBroadcaster = tf2_ros.TransformBroadcaster()
        rospy.sleep(1.0)


    def setupTrajManager(self, config_file_name):

        # Load robot configuration
        config = utils.loadYAMLConfig(os.path.join(self.current_dir,config_file_name))

        # Extract data from configuration
        mot_dim = config['motion_dimensions']

        # Extract data from configuration
        interpol = config['interpolation']

        # set info about TF
        self.header_frame_id = config['communication']['header_frame_id']
        self.msg_child_frame_id = config['communication']['msg_child_frame_id']

        # Establish connection with planning node
        # rospy.loginfo("%s: Waiting for "+CURRENT_JOINT_STATE_TOPIC +" topic", self.name)
        # msgRobotState = rospy.wait_for_message(CURRENT_JOINT_STATE_TOPIC, JointState)
        # self.startListening2Trajectories(msgRobotState)

        # Create trajectory manager instance
        self.trajManag = TrajManager(mot_dim, interpol)

        #  temp call to load trajectory data
        self.trajManag.updateTraj()


    def startListening2Trajectories(self):
        # Subscribe target trajectory callback
        # write a listener, that receives the new trajectories and update
        # the structure
        # call self.trajManag.updateTraj()
        pass


    def publishdNextWayPtToROS(self, event):

            motion = self.trajManag.getNextWayPt()

            # Pack pose msg
            msg = TransformStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.header_frame_id
            msg.child_frame_id = self.msg_child_frame_id
            msg.transform.translation.x = motion[0]
            msg.transform.translation.y = motion[1]
            msg.transform.translation.z = motion[2]
            msg.transform.rotation.x = motion[3]
            msg.transform.rotation.y = motion[4]
            msg.transform.rotation.z = motion[5]
            msg.transform.rotation.w = motion[6] # NOTE: the ordering here may be wrong

            # Publish msg
            self.tfBroadcaster.sendTransform(msg)


    def cleanShutdown(self):
        print('')
        rospy.loginfo("%s: Sending to safe configuration", self.name)
        # Shut down write callback
        self.writeCallbackTimer.shutdown()
        rospy.sleep(1.0)



if __name__ == '__main__':
    try:
        # Initialize node
        rospy.init_node("ros_Traj_interface", anonymous=True)
        # Initialize node class
        ROSTrajInterface = ROSTrajInterface()

        rospy.loginfo("%s: node started.", ROSTrajInterface.name)

        # Create timer for periodic publisher
        dur = rospy.Duration(ROSTrajInterface.dt)
        ROSTrajInterface.writeCallbackTimer = rospy.Timer(dur, ROSTrajInterface.publishdNextWayPtToROS)

        # Ctrl-C will stop the script
        rospy.on_shutdown(ROSTrajInterface.cleanShutdown)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
