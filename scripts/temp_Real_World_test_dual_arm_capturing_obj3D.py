#!/usr/bin/env python3
import sys
import os
import math
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

import tf2_ros
import rospy

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
from ros_pybullet_interface.utils import ROOT_DIR
import set_object_state_client


# --- import external library
from py_pack import yaml

# --- import Hybrid Trajectory Optimization class
from py_pack import hybrid_tosrb
from py_pack import hybrid_todac


NEW_TRAJ_TOPIC= 'ros_pybullet_interface/end_effector/traj' # publishes end-effector planned trajectory on this topic
NEW_TRAJ_OBJ_TOPIC = 'ros_pybullet_interface/object/traj' # publishes end-effector planned trajectory on this topic

WORLD_FRAME_ID = 'ros_pybullet_interface/world'
END_EFFECTOR_FRAME_ID = 'ros_pybullet_interface/robot/end_effector_sponge' # listens for end-effector poses on this topic
ROBOT_BASE_ID = "ros_pybullet_interface/robot/robot_base" # listen for the pose of the robot base


class PlanInterpWithTO:

    def __init__(self):

        # Name of node
        self.name = rospy.get_name()

        # check if the name of the robot is provided
        if rospy.has_param('~robot1_name') and rospy.has_param('~robot2_name'):
            self.robot1_name = rospy.get_param('~robot1_name')
            self.robot2_name = rospy.get_param('~robot2_name')
        else:
            rospy.logerr(f"The name of the robots is not set in {rospy.get_name()}")
            sys.exit(0)

        # Initialize data stream
        self.trajObjPlan = np.empty(0)
        self.trajRobotPlan = np.empty(0)
        self.trajYangPlan = np.empty(0)
        self.trajYinPlan = np.empty(0)

        self.IK_listen_buff = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.IK_listen_buff)

        # start punlishers
        self.new_Yangtraj_publisher = rospy.Publisher(f'{self.robot1_name}/{NEW_TRAJ_TOPIC}', Float64MultiArray, queue_size=1)
        self.new_Yintraj_publisher = rospy.Publisher(f'{self.robot2_name}/{NEW_TRAJ_TOPIC}', Float64MultiArray, queue_size=1)
        self.new_Objtraj_publisher = rospy.Publisher(NEW_TRAJ_OBJ_TOPIC, Float64MultiArray, queue_size=1)

        time.sleep(2.0) # wait for initialisation to complete


    def publishRobotTrajectory(self, event):

        message = self.np2DtoROSmsg(self.trajRobotPlan)

        if message != None:
            self.new_Robottraj_publisher.publish(message)


    def publishYangTrajectory(self, event):

        message = self.np2DtoROSmsg(self.trajYangPlan)

        if message != None:
            self.new_Yangtraj_publisher.publish(message)

    def publishYinTrajectory(self, event):

        message = self.np2DtoROSmsg(self.trajYinPlan)

        if message != None:
            self.new_Yintraj_publisher.publish(message)

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


    def buildTO(self, normIndex):

        #  create instance of the Hybrid optimisation class
        self.HybOpt3D = hybrid_tosrb.HybridTOClass3D()

        self.HybOpt_DAC = hybrid_todac.HybridTOClass_DAC(normIndex)
        # self.normIndex = normIndex

        # build the problem
        self.HybProb = self.HybOpt_DAC.buildProblem()

    def solveTO(self):

        # get object and robot state from pybullet
        basePosYang, baseAttYang, endPosYang, endAttYang = self.readInitials(self.robot1_name)
        print(endPosYang, endAttYang)

        basePosYin, baseAttYin, endPosYin, endAttYin = self.readInitials(self.robot2_name)
        print(endPosYin, endAttYin)

        # get initial guess
        xInit = self.HybOpt_DAC.buildInitialGuess()

        ''' ATTENTION: replace the following with parameter from ROS as:
            # Get ros parameters
            only_obj = rospy.get_param('~only_object')
        '''
        path2extrPck = os.environ['PATH2HYBRIDMPC']
        paramFile = os.path.join(path2extrPck,"py_pack/config/parameters.yml")
        with open(paramFile, 'r') as ymlfile:
            params = yaml.load(ymlfile, Loader=yaml.SafeLoader)

        ori_representation = params['TOproblem']['ori_representation']

        # get bounds of variables and constraints
        if ori_representation == "euler":
            # euler representation initialization #
            initObjPos = np.array([0.0, 0.0, 0.66, 0, 0, 0])
            finObjPos = np.array([0., 0.0, 0.725, 0, 0, 0])
            maxObjPos = np.array([2, 2, 2, np.pi, np.pi, np.pi])
        elif ori_representation == "quaternion":
            # quaternion representation initialization #
            initObjPos = np.array([0, 0, 0.6, 0, 0, 0, 1])
            finObjPos = np.array([0, 0, 0, 0, 0, 0, 1])
            maxObjPos = np.array([2, 2, 2, np.pi, np.pi, np.pi])

        initObjVel = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        finObjVel = np.array([0., 0., 0., 0., 0., 0.])
        maxObjVel = np.array([1.5, 1.5, 1.5, 1.5, 1.5, 1.5])

        slackObjPos = np.array([0.01, 0.01, 0.01, 0.01, 0.01, 0.01])
        slackObjVel = np.array([0.001, 0.001, 0.001, 0.001, 0.001, 0.001])

        initArmPos1 = np.array([endPosYang[0], endPosYang[1], endPosYang[2], endAttYang[0], endAttYang[1], endAttYang[2]])
        minArmPos1 = np.array([-1.0, -1.0, 0.0, -np.pi * 2, -np.pi * 2, -np.pi * 2])
        maxArmPos1 = np.array([1.0, 1.0, 1.0, np.pi * 2, np.pi * 2, np.pi * 2])

        initArmPos2 = np.array([endPosYin[0], endPosYin[1], endPosYin[2], endAttYin[0], endAttYin[1], endAttYin[2]])
        minArmPos2 = np.array([-1.0, -1.0, 0.0, -np.pi * 2, -np.pi * 2, -np.pi * 2])
        maxArmPos2 = np.array([1.0, 1.0, 1.0, np.pi * 2, np.pi * 2, np.pi * 2])

        lbx, ubx, lbg, ubg, cf, gf = self.HybOpt_DAC.buildBounds(initObjPos, finObjPos, maxObjPos, slackObjPos,
                                                          initObjVel, finObjVel, maxObjVel, slackObjVel,
                                                          initArmPos1, minArmPos1, maxArmPos1,
                                                          initArmPos2, minArmPos2, maxArmPos2)

        solFlag, xSolution = self.HybOpt_DAC.solveProblem(self.HybProb, xInit, lbx, ubx, lbg, ubg, cf, gf)

        # decode solution
        if (solFlag):
            timeArray, posBodyArray, velBodyArray, posLimb1Array, velLimb1Array, forLimb1Array,\
            posLimb2Array, velLimb2Array, forLimb2Array = self.HybOpt_DAC.decodeSol(xSolution)
            self.HybOpt_DAC.plotResult(timeArray, posBodyArray, velBodyArray, posLimb1Array, velLimb1Array,
                                       forLimb1Array, posLimb2Array, velLimb2Array, forLimb2Array, animateFlag=False)

            self.trajObjPlan = np.vstack((np.vstack((timeArray, posBodyArray)), velBodyArray))
            self.trajYangPlan = np.vstack((np.vstack((timeArray, posLimb1Array)), velLimb1Array))
            self.trajYinPlan = np.vstack((np.vstack((timeArray, posLimb2Array)), velLimb2Array))

        return solFlag

    def readInitials(self, robot_name):

        rospy.loginfo(f"{self.name}: Reading for /tf topic the position and orientation of the robot")
        # Read the position and orientation of the robot from the /tf topic

        if robot_name == "yang_visual":
            # trans = IK_listen_buff.lookup_transform(WORLD_FRAME_ID, f"{robot_name}/{ROBOT_BASE_ID}", rospy.Time())
            trans = self.IK_listen_buff.lookup_transform(WORLD_FRAME_ID, f"{robot_name}/{ROBOT_BASE_ID}", rospy.Time())
            # replaces base_position = config['base_position']
            base_position = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
            base_orient_quat = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z,
                                trans.transform.rotation.w]
            base_orient_euler = R.from_quat(base_orient_quat).as_euler('ZYX')

            trans = self.IK_listen_buff.lookup_transform(WORLD_FRAME_ID, f"{robot_name}/{END_EFFECTOR_FRAME_ID}", rospy.Time())
            # replaces base_position = config['base_position']
            end_position = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
            # replaces: base_orient_eulerXYZ = config['base_orient_eulerXYZ']
            end_orient_quat = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z,
                                trans.transform.rotation.w]
            end_orient_euler = R.from_quat(end_orient_quat).as_euler('ZYX')
            # base_orient_eulerXYZ = config['base_orient_eulerXYZ']

        elif robot_name == "yin_visual":
            trans = self.IK_listen_buff.lookup_transform(WORLD_FRAME_ID, f"{robot_name}/{ROBOT_BASE_ID}", rospy.Time())
            # replaces base_position = config['base_position']
            base_position = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
            # replaces: base_orient_eulerXYZ = config['base_orient_eulerXYZ']
            base_orient_quat = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z,
                                trans.transform.rotation.w]
            base_orient_euler = R.from_quat(base_orient_quat).as_euler('ZYX')

            trans = self.IK_listen_buff.lookup_transform(WORLD_FRAME_ID, f"{robot_name}/{END_EFFECTOR_FRAME_ID}",
                                                         rospy.Time())
            # replaces base_position = config['base_position']
            end_position = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
            # replaces: base_orient_eulerXYZ = config['base_orient_eulerXYZ']
            end_orient_quat = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z,
                               trans.transform.rotation.w]
            end_orient_euler = R.from_quat(end_orient_quat).as_euler('ZYX')

        return base_position, base_orient_euler, end_position, end_orient_euler


if __name__=='__main__':

    rospy.sleep(1.0)

    # --- setup the ros interface --- #
    rospy.init_node('test_dual_arm_capturing_obj3D', anonymous=True)
    rospy.logwarn("ATTENTION: This node will not run without the impact-TO library!")

    freq = 100
    rospy.sleep(1)
    PlanInterpWithTO = PlanInterpWithTO()
    rospy.loginfo("%s: node started.", PlanInterpWithTO.name)

    # build the TO problem
    normIndex = [0, 3]
    PlanInterpWithTO.buildTO(normIndex)
    # solve the TO problem
    solFlag = PlanInterpWithTO.solveTO()

    if solFlag == True:
        rospy.loginfo("TO problem solved, publish the state of object and robots!")
        PlanInterpWithTO.writeCallbackTimerYang = rospy.Timer(rospy.Duration(1.0/float(freq)), PlanInterpWithTO.publishYangTrajectory)
        PlanInterpWithTO.writeCallbackTimerYin = rospy.Timer(rospy.Duration(1.0/float(freq)), PlanInterpWithTO.publishYinTrajectory)
        PlanInterpWithTO.writeCallbackTimerObj = rospy.Timer(rospy.Duration(1.0/float(freq)), PlanInterpWithTO.publishObjTrajectory)

        rospy.loginfo("TO problem solved, set visual object state in bullet!")
        set_object_state_client.main()

    rospy.spin()
