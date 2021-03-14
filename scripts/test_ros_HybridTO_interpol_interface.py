#!/usr/bin/env python3
import sys
import os
import math
import time
import numpy as np

import rospy

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
from ros_pybullet_interface.utils import ROOT_DIR

# --- library for Hybrid TO
from sys import path
path.insert(0,"/home/lei/research/trajectory_optimization/Hybrid_MPC")

# --- import external library
from py_pack import np, os, yaml
from py_pack import R

# --- import Hybrid Trajectory Optimization class
from py_pack import hybrid_tosrb

NEW_TRAJ_ROBOT_TOPIC = 'ros_pybullet_interface/end_effector/traj' # publishes end-effector planned trajectory on this topic
NEW_TRAJ_OBJ_TOPIC = 'ros_pybullet_interface/object/traj' # publishes end-effector planned trajectory on this topic


class TestInterpolation:

    def __init__(self):

        # Name of node
        self.name = rospy.get_name()

        # Initialize data stream
        self.trajObjPlan = np.empty(0)
        self.trajRobotPlan = np.empty(0)

        # Get ros parameters
        only_obj = rospy.get_param('~only_object')


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


if __name__=='__main__':

    # --- run the Hybrid trajectory optimization --- #
    paramFile = "/home/lei/research/trajectory_optimization/Hybrid_MPC/py_pack/config/parameters.yml"
    path2dirOffile = os.path.dirname(os.path.abspath(__file__))
    with open(paramFile, 'r') as ymlfile:
        params = yaml.load(ymlfile, Loader=yaml.SafeLoader)

    ori_representation = params['TOproblem']['ori_representation']

    #  create instance of the Hybrid optimisation class
    HybOpt3D = hybrid_tosrb.HybridTOClass3D()

    # build the problem
    HybProb = HybOpt3D.buildProblem()

    # get initial guess
    xInit = HybOpt3D.buildInitialGuess()

    # get bounds of variables and constraints
    if ori_representation == "euler":
        # euler representation initialization #
        initObjPos = np.array([-1.5, 0, 0, 0, 0, 0])
        finObjPos = np.array([-1.5, 0, 0, 0, 0, 0])
        maxObjPos = np.array([1000, 1000, 1000, 1000, 1000, 1000])
    elif ori_representation == "quaternion":
        # quaternion representation initialization #
        initObjPos = np.array([-1.5, 0, 0, 0, 0, 0, 1])
        finObjPos = np.array([-1.5, 0, 0, 0, 0, 0, 1])
        maxObjPos = np.array([1000, 1000, 1000, 1000, 1000, 1000, 1000])

    initObjVel = np.array([1, 0, 0, -1, 0, 0])
    finObjVel = np.array([0, 0, 0, 0, 0, 0])
    maxObjVel = np.array([10, 10, 10, 10, 10, 10])

    lbx, ubx, lbg, ubg, cf, gf = HybOpt3D.buildBounds(initObjPos, finObjPos, maxObjPos, 0.0,
                                                      initObjVel, finObjVel, maxObjVel, 0.0)

    # solve problem
    solFlag, xSolution = HybOpt3D.solveProblem(HybProb, xInit, lbx, ubx, lbg, ubg, cf, gf)

    # decode solution
    if (solFlag):
        timeArray, posArray, velArray, forceArray = HybOpt3D.decodeSol(xSolution, animateFlag=False)


    # --- setup the ros interface --- #
    rospy.init_node('test_ros_HybridTO_interpol_interface', anonymous=True)
    freq = 10
    TestInterpolation = TestInterpolation()
    TestInterpolation.trajObjPlan = np.vstack((np.vstack((timeArray, posArray)), velArray))

    rospy.loginfo("%s: node started.", TestInterpolation.name)

    TestInterpolation.writeCallbackTimerRobot = rospy.Timer(rospy.Duration(1.0/float(freq)), TestInterpolation.publishRobotTrajectory)
    TestInterpolation.writeCallbackTimerObj = rospy.Timer(rospy.Duration(1.0/float(freq)), TestInterpolation.publishObjTrajectory)


    rospy.spin()
