#!/usr/bin/env python3
# license removed for brevity
import rospkg
import rospy
import os

import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

# ROS message types
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32


import ros_pybullet_interface.utils as utils

import ros_pybullet_interface.interpolation as interpol

# ------------------------------------------------------
#
# Constants
# ------------------------------------------------------

FREQ = 200 # Resolution of trajectory knots --- sampling frequency

class TrajManager:

    def __init__(self, mot_dim, interpol):


        self.nochange_win_len  = interpol['nochange_window_length']
        self.interFreq = 1.0/interpol['interDt']
        self.use_interp = interpol['use_interpolation']

        # number of dimentions of the trajectory plan
        # trajPlanDim = mot_dim["number"]

        # information about the fixed and planned dimensions of the motion
        self.mot_dim = mot_dim

        # init struct for interpolated motion plan
        self.motionInterpPlan = np.array([])
        self.stiffnessInterpPlan = np.array([])


    def getNextWayPt(self):
        """ Get function to access data from the trajectory class """

        traj_waypt = self.popFirstTrajElem()

        if traj_waypt is not None:
            return self.transTraj2Motion6D(traj_waypt)
        else:
            return None

    def getNextStiffness(self):
        """ Get function to access data from the trajectory class """

        stiffness_waypt = self.popFirstStiffnessElem()

        if stiffness_waypt is not None:
            return stiffness_waypt
        else:
            return None


    def transTraj2Motion6D(self, way_pt):
        """A function that maps dimensions of the Traj to 6D"""

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
        # specify manually axis and angle --- used for fixed orientation
        if self.mot_dim['rotation']['rotation_repr'] == 'None':
            Ori_Rot = R.from_rotvec(np.deg2rad(self.mot_dim['rotation']['rotationangle'])*np.array(self.mot_dim['rotation']['rotationvec']))
        # specify manually axis and take angle from the planner  --- used for rotation around fixed axis
        elif self.mot_dim['rotation']['rotation_repr'] == 'Theta':
            Ori_Rot = R.from_rotvec(np.array(self.mot_dim['rotation']['rotationvec'])*way_pt[self.mot_dim['rotation']['rotationvec_index'][0]])
        elif self.mot_dim['rotation']['rotation_repr'] == 'Euler':
            # take euler angles directly from the planner
            idx = self.mot_dim['rotation']['rotationvec_index']
            Ori_Rot = R.from_euler('ZYX', np.array(way_pt[idx[0]:idx[1]]))
        elif self.mot_dim['rotation']['rotation_repr'] == 'Quat':
            # take quaternion directly from the planner
            idx = self.mot_dim['rotation']['rotationvec_index']
            Ori_Rot = R.from_quat(np.array(way_pt[idx[0]:idx[1]]))

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


        # if we use interpolation, we use the interpolated one
        # if self.use_interp:

        if self.motionInterpPlan.shape[1] == 0:
            rospy.logerr("All the trajectory data has been consumed")
            return None

        nextWaypt = self.motionInterpPlan[:,0]
        self.motionInterpPlan = np.delete(self.motionInterpPlan, 0, 1)
        self.timeInterpPlan = np.delete(self.timeInterpPlan, 0)

        return nextWaypt

    def popFirstStiffnessElem(self):
        """ Extract the 1st element of the motion struct to
        send to the simulation"""


        # if we use interpolation, we use the interpolated one
        # if self.use_interp:

        if self.stiffnessInterpPlan.shape[0] == 0:
            rospy.logerr("All the trajectory data has been consumed")
            return None

        nextWaypt = self.stiffnessInterpPlan[0]
        self.stiffnessInterpPlan = np.delete(self.stiffnessInterpPlan, 0)

        return nextWaypt


    def updateTraj(self, new_traj):
        """ Implemented for receiding horizon and MPC loops
            Needs to be extensively tested, when a receiding horizon or MPC
            motion planner is available                    """

        # from which knot and on of the new traj, do we what to use?
        # ATTENTION: This should be become a parameter, when receiding horizon/MPC can be tested
        index_of_1st_knot = 1

        # # time is always the first row
        timeVec = new_traj[0,:]

        # get the index where to new trajectory data should be inserted
        insertIndex = self.findInsertIndex(timeVec, index_of_1st_knot)

        # create new time vector
        timevector = np.append(self.timeInterpPlan[:insertIndex], self.timeInterpPlan[insertIndex] + timeVec[index_of_1st_knot:].reshape(1,timeVec[index_of_1st_knot:].shape[0]))
        timevector = timevector.reshape(1,timevector.shape[0])

        numRows, _ = new_traj.shape
        midRow = self.mot_dim['number'] + 1

        # first half rows denote position
        trajPlan = np.hstack((self.motionInterpPlan[:,:insertIndex],  new_traj[1:midRow,index_of_1st_knot:]))
        # second half rows denote velocity
        dtrajPlan_noaction_window = np.diff(self.motionInterpPlan[:,:insertIndex+1])
        dtrajPlan = np.hstack((dtrajPlan_noaction_window,  new_traj[midRow:,index_of_1st_knot:]))

        # interpolate
        if self.use_interp:
            self.timeInterpPlan, self.motionInterpPlan = self.computeInterpTraj(timevector, trajPlan, dtrajPlan )
        else:
            self.timeInterpPlan = timevector
            self.motionInterpPlan = trajPlan

    def findInsertIndex(self, time_vector, index_1knot):

        # find where along the time axis should the data be added
        insertionIndex = np.where(self.timeInterpPlan > time_vector[index_1knot])
        #  if first knot of new traj is after the duration of the current trajectory
        if insertionIndex[0].size == 0:
            insertionIndex = -1
        else:
            #  if first knot of new traj is within the duration of the current trajectory
            insertionIndex = insertionIndex[0][0]
            if insertionIndex < self.nochange_win_len:
                #  if first knot of new traj is within the nochange_window of the current trajectory
                rospy.logerr("The first new knot of the trajectory is timed to be within the nochange_window region! It will be overriden.")
                insertionIndex = self.nochange_win_len

        return insertionIndex


    def setInitTraj(self, new_traj):
        """ More comments are needed """

        # # time is always the first row
        timeVec = new_traj[0,:]
        timevector = timeVec.reshape(1,timeVec.shape[0])

        numRows, _ = new_traj.shape
        midRow = self.mot_dim['number'] + 1

        # first half rows denote position
        trajPlan = new_traj[1:midRow,:]

        # second half rows denote velocity
        dtrajPlan = new_traj[midRow:,:]

        # interpolate
        if self.use_interp:
            self.timeInterpPlan, self.motionInterpPlan = self.computeInterpTraj(timevector, trajPlan, dtrajPlan)
        else:
            self.timeInterpPlan = timevector
            self.motionInterpPlan = trajPlan


    def setInitStiffness(self, new_traj):
        """ More comments are needed """

        # # time is always the first row
        timePlan = new_traj[0,:]
        # timePlan = timePlan.reshape(1,timePlan.shape[0])
        # second row denote stiffness
        trajPlan = new_traj[1,:]
        # trajPlan = trajPlan.reshape(1,trajPlan.shape[0])

        # interpolate
        if self.use_interp:
            # self.timeInterpPlan, self.motionInterpPlan = self.computeInterpTraj(timevector, trajPlan, dtrajPlan)
            _, self.stiffnessInterpPlan = self.computeInterpStiffness(timePlan, trajPlan)
        else:
            self.stiffnessInterpPlan = trajPlan


    def computeInterpStiffness(self, time_vector, traj_plan):
        tempMotionInterpPlan = np.empty((0))
        trajDim = traj_plan.shape[0]
        row_len = 0
        interSeqTime, interSeq_I = interpol.interpolateInterp1d(time_vector, traj_plan, kind='linear', sampleFreq = self.interFreq, plotFlag=False, plotTitle="None")

        return interSeqTime, interSeq_I


    def computeInterpTraj(self, time_vector, traj_plan, dtraj_plan):
        """ Compute the interpolated trajectory from the planning traj

            ATTENTION: if the angular motion is provide in 3D, it should be quaternions
            to used slerp... Implementation pending....
                                                                             """
        tempMotionInterpPlan = np.empty((0))
        trajDim = traj_plan.shape[0]
        row_len = 0

        rot_repr = self.mot_dim['rotation']['rotation_repr']
        if rot_repr == 'None' or rot_repr == 'Theta':
            # for each dimension of the motion compute the interpolated trajectory
            for i in range(trajDim):
                interSeqTime, interSeq_I = interpol.interpolateCubicHermiteSpline(time_vector[0,:], traj_plan[i,:], dtraj_plan[i,:], sampleFreq=self.interFreq, plotFlag=False, plotTitle="DimVsTime")
                tempMotionInterpPlan = np.append(tempMotionInterpPlan, interSeq_I, axis=0)

            row_len = trajDim
        else:
            # for each translational dimension of the motion compute the interpolated trajectory
            # using position and derivatives
            if self.mot_dim['trans']['translationX'] is None:
                i = self.mot_dim['trans']['translationX_index']
                interSeqTime, interSeq_I = interpol.interpolateCubicHermiteSpline(time_vector[0,:], traj_plan[i,:], dtraj_plan[i,:], sampleFreq=self.interFreq, plotFlag=False, plotTitle="XVsTime")
                tempMotionInterpPlan = np.append(tempMotionInterpPlan, interSeq_I, axis=0)
                row_len += 1

            if self.mot_dim['trans']['translationY'] is None:
                i = self.mot_dim['trans']['translationY_index']
                interSeqTime, interSeq_I = interpol.interpolateCubicHermiteSpline(time_vector[0,:], traj_plan[i,:], dtraj_plan[i,:], sampleFreq=self.interFreq, plotFlag=False, plotTitle="YVsTime")
                tempMotionInterpPlan = np.append(tempMotionInterpPlan, interSeq_I, axis=0)
                row_len += 1

            if self.mot_dim['trans']['translationZ'] is None:
                i = self.mot_dim['trans']['translationZ_index']
                interSeqTime, interSeq_I = interpol.interpolateCubicHermiteSpline(time_vector[0,:], traj_plan[i,:], dtraj_plan[i,:], sampleFreq=self.interFreq, plotFlag=False, plotTitle="ZVsTime")
                tempMotionInterpPlan = np.append(tempMotionInterpPlan, interSeq_I, axis=0)
                row_len += 1

            # for each rotational dimension of the motion compute the interpolated trajectory
            # using only the value of the Euler angles and neglect angular velocity
            if rot_repr == 'Euler':
                rot_idx = self.mot_dim['rotation']['rotationvec_index']
                for i in range(rot_idx[0],rot_idx[1]):
                    # interSeqTime, interSeq_I = interpol.interpolatePolyfit(time_vector[0,:],  traj_plan[i,:], polyOrder = 3, sampleFreq=self.interFreq, plotFlag=True, plotTitle="EulerVsTime")
                    interSeqTime, interSeq_I = interpol.interpolateCubicSpline(time_vector[0,:],  traj_plan[i,:], sampleFreq=self.interFreq, plotFlag=False, plotTitle="EulerVsTime")
                    tempMotionInterpPlan = np.append(tempMotionInterpPlan, interSeq_I, axis=0)
                row_len += 3

            # for each rotational dimension of the motion compute the interpolated trajectory
            # using slerp for quaternions
            elif rot_repr == 'Quat':
                # we need to do slerp
                rot_idx = self.mot_dim['rotation']['rotationvec_index']
                # interSeqTime, interSeq_I = interpol.interpolateLinearlyQuaternions(time_vector[0,:],  traj_plan[rot_idx[0]:rot_idx[1],:], sampleFreq=self.interFreq)
                interSeqTime, interSeq_I = interpol.interpolateCubiQuaternions(time_vector[0,:],  traj_plan[rot_idx[0]:rot_idx[1],:], sampleFreq=self.interFreq)
                for i in range(interSeq_I.shape[1]):
                    tempMotionInterpPlan = np.append(tempMotionInterpPlan, interSeq_I[:,i], axis=0)
                row_len += 4


        # reshape to have a dimension per row
        col_len = interSeq_I.shape[0]
        tempMotionInterpPlan = tempMotionInterpPlan.reshape(row_len, col_len)

        return interSeqTime, tempMotionInterpPlan



class ROSTrajInterface(object):

    def __init__(self):

        # Name of node
        self.name = rospy.get_name()
        # Initialization message
        rospy.loginfo("%s: Initializing class", self.name)

        # safty check
        if FREQ > 200:
            rospy.loginfo("%s: Shutting down to high FREQUENCY of consuming in node ", self.name)
            self.cleanShutdown()

        # Setup constants
        self.dt = 1.0/float(FREQ)

        # get the path to this catkin ws
        self.current_dir = utils.ROOT_DIR

        # Get ros parameters
        self.traj_config_file_name = rospy.get_param('~traj_config')

        # if interpolation is for a robot
        self.robot_name = rospy.get_param('~robot_name','')

        # flag for stiffness interpolation/publishing
        self.variable_stiffness = rospy.get_param('~variable_stiffness', False)

        # stream the interpolated data or not
        rospy.set_param('/stream_interpolated_motion_flag', False)

        #  TrajManager
        self.setupTrajManager(self.traj_config_file_name, self.robot_name)

        # Establish connection with planning node
        rospy.loginfo(f"{self.name}: Waiting for self.current_traj_topic topic")
        msgTraj = rospy.wait_for_message(self.current_traj_topic , Float64MultiArray)

        # single update of trajectory
        # To be used for play-back motion plans
        self.readInitialTrajFromROS(msgTraj)

        if self.variable_stiffness:
            self.setupStiffnessManager(self.traj_config_file_name, self.robot_name)
            msgStiffness = rospy.wait_for_message(self.current_stiffness_listener , Float64MultiArray)
            self.readInitialStiffnessFromROS(msgStiffness)

        # Subscribe target trajectory callback
        # repetitive update of trajectory
        # To be used for receiding horizon and/or MPC motion plans
        # rospy.Subscriber(self.current_traj_topic, Float64MultiArray, self.readCurrentTrajUpdateFromROS)

        self.tfBroadcaster = tf2_ros.TransformBroadcaster()

        # rospy.sleep(3.0)


    def setupTrajManager(self, config_file_name, robot):

        # Load robot configuration
        config = utils.loadYAMLConfig(config_file_name)

        # Extract data from configuration
        mot_dim = config['motion_dimensions']

        # Extract data from configuration
        interpol = config['interpolation']

        # set info about TF publisher
        self.msg_header_frame_id = config['communication']['publisher']['header_frame_id']
        self.msg_child_frame_id = f"{robot}/{config['communication']['publisher']['msg_child_frame_id']}"

        # set info for listener
        self.current_traj_topic = f"{robot}/{config['communication']['listener']['topic']}"

        # Create trajectory manager instance
        self.trajManag = TrajManager(mot_dim, interpol)


    def setupStiffnessManager(self, config_file_name, robot):

        # Load robot configuration
        config = utils.loadYAMLConfig(config_file_name)

        # Extract data from configuration
        interpol = config['interpolation']

        # set info for listener
        self.current_stiffness_listener = f"{robot}/{config['stiffnessComm']['listener']['topic']}"
        # set info for stiffness publisher
        self.stiffness_publisher_name = f"{robot}/{config['stiffnessComm']['publisher']['topic']}"
        self.current_stiffness_publisher = rospy.Publisher(self.stiffness_publisher_name, Float32, queue_size=1)

        # Create trajectory manager instance
        self.stiffnessManag = TrajManager(1, interpol)



    def readInitialTrajFromROS(self, msg):
        # listener, that receives the initial trajectory
        # decode msg
        msg_data = self.decodeROStrajmsg(msg)

        #  call to initial setup of trajectory data
        self.trajManag.setInitTraj(msg_data)

    def readInitialStiffnessFromROS(self, msg):
        # listener, that receives the initial trajectory
        # decode msg
        msg_data = self.decodeROStrajmsg(msg)

        #  call to initial setup of trajectory data
        self.stiffnessManag.setInitStiffness(msg_data)


    def readCurrentTrajUpdateFromROS(self, msg):
        # listener, that receives the new trajectories and update the structure
        # decode msg
        msg_data = self.decodeROStrajmsg(msg)

        #  call to update the trajectory data
        self.trajManag.updateTraj(msg_data)


    def decodeROStrajmsg(self, msg):
        """ From Float64MultiArray type msg to numpy 2D array"""

        if msg.layout.dim[0].label  == "rows":
            rows = msg.layout.dim[0].size
        if msg.layout.dim[1].label  == "columns":
            columns = msg.layout.dim[1].size

        data = np.array(msg.data).reshape(rows, columns)

        return data

    def publishdNextWayPtToROS(self, event):
        """ Publish 6D information for the respective rigid body """

        # check if stream flag is active
        if rospy.get_param('/stream_interpolated_motion_flag')!= True:
            return


        motion = self.trajManag.getNextWayPt()

        # if the motion plan is not empty
        if motion is not None:

            # Pack pose msg
            msg = TransformStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.msg_header_frame_id
            msg.child_frame_id = self.msg_child_frame_id
            msg.transform.translation.x = motion[0]
            msg.transform.translation.y = motion[1]
            msg.transform.translation.z = motion[2]
            msg.transform.rotation.x = motion[3]
            msg.transform.rotation.y = motion[4]
            msg.transform.rotation.z = motion[5]
            msg.transform.rotation.w = motion[6] # NOTE: the ordering here may be wrong

            # Publish tf msg
            self.tfBroadcaster.sendTransform(msg)

            if self.variable_stiffness:

                # Publish stifness msg
                stiffness = self.stiffnessManag.getNextStiffness()
                msg = Float32()
                msg.data = stiffness
                self.current_stiffness_publisher.publish(msg)

        else:
            self.cleanShutdown()

    def publishdNextStiffness(self, event):
        stiffness = self.stiffnessManag.getNextStiffness()

        msg = Float32()
        msg.data = stiffness

        self.current_stiffness_publisher.publish(msg)

    def cleanShutdown(self):
        print('')
        rospy.loginfo("%s: Shutting down interpolation node ", self.name)
        # Shut down write callback
        self.writeCallbackTimer.shutdown()
        rospy.sleep(1.0)



if __name__ == '__main__':
    try:
        rospy.sleep(1.0)
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
