#!/usr/bin/env python3
import sys
import os
import math
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

import rospy

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
from ros_pybullet_interface.utils import ROOT_DIR
import set_object_state_client


# --- import external library
from py_pack import yaml

# --- import Hybrid Trajectory Optimization class
from py_pack import hybridto_srb

NEW_TRAJ_ROBOT_TOPIC = 'ros_pybullet_interface/end_effector/traj' # publishes end-effector planned trajectory on this topic
NEW_TRAJ_OBJ_TOPIC = 'ros_pybullet_interface/object/traj' # publishes end-effector planned trajectory on this topic


class PlanInterpWithTO:

    def __init__(self):

        # Name of node
        self.name = rospy.get_name()

        # Initialize data stream
        self.trajObjPlan = np.empty(0)
        self.trajRobotPlan = np.empty(0)

        # start punlishers
        self.new_Robottraj_publisher = rospy.Publisher(NEW_TRAJ_ROBOT_TOPIC, Float64MultiArray, queue_size=1)
        self.new_Objtraj_publisher = rospy.Publisher(NEW_TRAJ_OBJ_TOPIC, Float64MultiArray, queue_size=1)

        time.sleep(2.0) # wait for initialisation to complete


    def publishRobotTrajectory(self, event):

        message = self.np2DtoROSmsg(self.trajRobotPlan)

        if message != None:
            self.new_Robottraj_publisher.publish(message)

    def publishObjTrajectory(self, event):

        message = self.np2DtoROSmsg(self.trajObjPlan)

        if message != None:
            self.new_Objtraj_publisher.publish(message)


    def np2DtoROSmsg(self, data2Darray):

        # check if there is nothing to publish
        if data2Darray.size == 0:
            return None

        r, c = data2Darray.shape

        # info: http://docs.ros.org/en/api/std_msgs/html/msg/MultiArrayLayout.html
        # Pack trajectory msg
        msg = Float64MultiArray()

        # specify that the array has 2 dimensions
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim.append(MultiArrayDimension())

        # info for reconstruction of the 2D array
        msg.layout.dim[0].label  = "rows"
        msg.layout.dim[0].size   = r
        msg.layout.dim[1].label  = "columns"
        msg.layout.dim[1].size   = c

        # add data as flattened numpy array
        msg.data = data2Darray.flatten('C') # row major flattening


        # time is missing from this message
        # msg.header.stamp = rospy.Time.now()
        return msg


    def buildTO(self):

        #  create instance of the Hybrid optimisation class
        self.HybOpt3D = hybridto_srb.HybridTOClass3D()

        # build the problem
        self.HybProb = self.HybOpt3D.buildProblem()

    def solveTO(self):

        # get initial guess
        xInit = self.HybOpt3D.buildInitialGuess()


        ''' ATTENTION: replace the following with parameter from ROS as:
            # Get ros parameters
            only_obj = rospy.get_param('~only_object')
        '''

        # get bounds of variables and constraints
        finObjPos = np.array([-1.0, 0, 0, 0, 0, 0])
        maxObjPos = np.array([1000, 1000, 1000, 1000, 1000, 1000])

        finObjVel = np.array([0, 0, 0, 0, 0, 0])
        maxObjVel = np.array([10, 10, 10, 10, 10, 10])

        lbx, ubx, lbg, ubg, cf, gf = self.HybOpt3D.buildBounds(initObjPos, finObjPos, maxObjPos, 0.0,
                                                               initObjVel, finObjVel, maxObjVel, 0.0)

        # solve problem
        solFlag, xSolution = self.HybOpt3D.solveProblem(self.HybProb, xInit, lbx, ubx, lbg, ubg, cf, gf)

        # decode solution
        if (solFlag):
            timeArray, posArray, velArray, forceArray = self.HybOpt3D.decodeSol(xSolution, animateFlag=False)

        self.trajObjPlan = np.vstack((np.vstack((timeArray, posArray)), velArray))

        return solFlag


if __name__=='__main__':

    # --- setup the ros interface --- #
    rospy.init_node('test_ros_traj_opt_obj3D', anonymous=True)
    rospy.logwarn("ATTENTION: This node will not run without the impact-TO library!")

    freq = 100
    PlanInterpWithTO = PlanInterpWithTO()
    rospy.loginfo("%s: node started.", PlanInterpWithTO.name)

    # build the TO problem
    PlanInterpWithTO.buildTO()

    # get bounds of variables and constraints
    path2extrPck = os.environ['PATH2HYBRIDMPC']
    paramFile = os.path.join(path2extrPck, "py_pack/config/parameters.yml")
    with open(paramFile, 'r') as ymlfile:
        params = yaml.load(ymlfile, Loader=yaml.SafeLoader)
    ori_representation = params['TOproblem']['ori_representation']
    if ori_representation == "euler":
        # euler representation initialization #
        initObjPos = np.array([0, -0.6, 0.3, 0, 0, 0])
        pos = initObjPos[0:3]
        quat = R.from_euler('ZYX', initObjPos[3:6]).as_quat()
    elif ori_representation == "quaternion":
        # quaternion representation initialization #
        initObjPos = np.array([0, -0.6, 0.3, 0, 0, 0, 1])
        pos = initObjPos[0:3]
        quat = initObjPos[3:7]
    initObjVel = np.array([0.3, 0.0, 0.0, 0.1, 0.1, 0.1])
    lin_vel = initObjVel[0:3];    ang_vel = initObjVel[3:6]

    # solve the TO problem
    solFlag = PlanInterpWithTO.solveTO()

    # PlanInterpWithTO.writeCallbackTimerRobot = rospy.Timer(rospy.Duration(1.0/float(freq)), PlanInterpWithTO.publishRobotTrajectory)
    PlanInterpWithTO.writeCallbackTimerObj = rospy.Timer(rospy.Duration(1.0/float(freq)), PlanInterpWithTO.publishObjTrajectory)

    if solFlag == True:
        rospy.loginfo(" TO problem solved!")
        set_object_state_client.setObjState(pos, quat, lin_vel, ang_vel, 'target' )

    rospy.spin()
