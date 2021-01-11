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
WORLD_FRAME_ID = 'ros_pybullet_interface/world'
END_EFFECTOR_TARGET_FRAME_ID = 'ros_pybullet_interface/end_effector/target'


class TrajManager:

    def __init__(self, hor_len, use_interpolation, inter_dt,\
                    rob_traj_dim, def_rob_mot_axes, def_rob_mot_axes_val, \
                    obj_traj_dim, def_obj_mot_dim, def_obj_mot_axes_val):

        consumerCounter = 0
        self.horizonLength = hor_len
        self.interFreq = 1/inter_dt
        self.use_interp = use_interpolation

        self.rob_traj_dim = rob_traj_dim
        self.obj_traj_dim = obj_traj_dim

        self.def_rob_mot_axes_val = def_rob_mot_axes_val
        self.def_obj_mot_axes_val = def_obj_mot_axes_val

        # number of dimentions of the trajectory plan
        trajPlanDim = len(rob_traj_dim) + len(obj_traj_dim)

        # init struct for trajectory plan received by a planner
        self.trajPlan = np.zeros((trajPlanDim, 1))

        if use_interpolation:
            self.dTrajPlan = np.zeros((trajPlanDim, 1))
            self.time = np.zeros((1, 1))

        # init struct for interpolated motion plan
        self.motionInterpPlan = np.array([])


    def getNextWayPt(self):

        traj_waypt = self.popFirstTrajElem()

        return self.transTraj2Motion6D(traj_waypt)

    def transTraj2Motion6D(self, way_pt):
        """ We need a function that maps dimensions of the Traj to 6D"""

        # Robot end-effector
        pos_rob = np.array([way_pt[3], way_pt[4], self.def_rob_mot_axes_val[0]])
        eeOri_Rot_rob = R.from_rotvec(self.def_rob_mot_axes_val[1:])

        eeOri_Rot_rob = R.from_quat(np.array([1.0, 0.0, 0.0,  -0.0]))
        # eeOri_Rot_rob = R.from_quat(np.array([0.991629064083, 0.00588622735813, 0.128958553076,  -0.0026147547178]))
        eeOri_rob = eeOri_Rot_rob.as_quat()

        # object
        pos_obj = np.array([way_pt[0], way_pt[1], self.def_obj_mot_axes_val[0]])

        # requires scipy 1.6.0 --- not available yet in my pip
        # source : https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.from_mrp.html#scipy.spatial.transform.Rotation.from_mrp
        # mrp_vec = np.array(self.def_obj_mot_axes_val[1:])*np.tan(way_pt[2]/4)
        # eeOri_Rot_obj = R.from_mrp(mrp_vec)

        rot_mat = utils.rotation_matrix_from_axis_angle(np.array(self.def_obj_mot_axes_val[1:]), way_pt[2])
        eeOri_Rot_obj = R.from_matrix(rot_mat)
        eeOri_obj = eeOri_Rot_obj.as_quat()

        return np.hstack((pos_rob,eeOri_rob)), np.hstack((pos_obj, eeOri_obj))


    # --------------------------------------------------------------------
    #   ATTENTION : popFirstTrajElem and updateTraj need to be thread safe!!!
    #             At the moment they are NOT!!!!!
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
        self.trajPlan = np.hstack((self.trajPlan, np.vstack( (data[0:3,:], data[6:8,:]))))
        self.dTrajPlan = np.hstack((self.dTrajPlan, np.vstack((data[3:6,:], data[8:10,:]))))
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
        rospy.sleep(5.0)

        self.flag = 0


    def setupTrajManager(self, config_file_name):

        # Load robot configuration
        config = utils.loadYAMLConfig(os.path.join(self.current_dir,config_file_name))

        # Extract data from configuration
        rob_traj_dim = config['robot_trajectory_dimensions']
        def_rob_mot_axes = config['default_robot_motion_axes']
        def_rob_mot_axes_val = config['default_robot_motion_axes_values']

        if config['object_properties']:
            obj_traj_dim = config['object_trajectory_dimensions']
            def_obj_mot_dim = config['default_object_motion_dimensions']
            def_obj_mot_axes_val = config['default_object_motion_axes_values']
        else:
            obj_traj_dim = None
            def_obj_mot_dim = None
            def_obj_mot_axes_val = None

        hor_len = config['horizon_Length']
        use_interp = config['use_interpolation']
        interDt = config['interDt']

        # Establish connection with planning node
        # rospy.loginfo("%s: Waiting for "+CURRENT_JOINT_STATE_TOPIC +" topic", self.name)
        # msgRobotState = rospy.wait_for_message(CURRENT_JOINT_STATE_TOPIC, JointState)
        # self.startListening2Trajectories(msgRobotState)

        # Create trajectory manager instance
        self.trajManag = TrajManager(hor_len, use_interp, interDt,\
                                    rob_traj_dim, def_rob_mot_axes, def_rob_mot_axes_val, \
                                    obj_traj_dim, def_obj_mot_dim, def_obj_mot_axes_val)

        #  temp call to load trajectory data
        self.trajManag.updateTraj()


    def startListening2Trajectories(self):
        # Subscribe target trajectory callback
        # write a listener, that receives the new trajectories and update
        # the structure
        # call self.trajManag.updateTraj()
        pass


    def publishdNextWayPtToROS(self, event):

            robot, obj = self.trajManag.getNextWayPt()

            # Pack pose msg
            msg = TransformStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'ros_pybullet_interface/world'
            msg.child_frame_id = 'ros_pybullet_interface/end_effector/target'
            msg.transform.translation.x = robot[0]
            msg.transform.translation.y = robot[1]
            msg.transform.translation.z = robot[2]
            msg.transform.rotation.x = robot[3]
            msg.transform.rotation.y = robot[4]
            msg.transform.rotation.z = robot[5]
            msg.transform.rotation.w = robot[6] # NOTE: the ordering here may be wrong

            # Publish msg
            self.tfBroadcaster.sendTransform(msg)


            # Pack pose msg
            msg = TransformStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'ros_pybullet_interface/world'
            msg.child_frame_id = 'ros_pybullet_interface/box'
            msg.transform.translation.x = obj[0]
            msg.transform.translation.y = obj[1]
            msg.transform.translation.z = obj[2]
            msg.transform.rotation.x = obj[3]
            msg.transform.rotation.y = obj[4]
            msg.transform.rotation.z = obj[5]
            msg.transform.rotation.w = obj[6] # NOTE: the ordering here may be wrong

            # Publish msg
            self.tfBroadcaster.sendTransform(msg)
            if self.flag == 0:
                rospy.sleep(2.0)
                self.flag = 1

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
