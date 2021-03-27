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

NEW_TRAJ_ROBOT_TOPIC = 'ros_pybullet_interface/end_effector/traj' # publishes end-effector planned trajectory on this topic
NEW_TRAJ_OBJ_TOPIC = 'ros_pybullet_interface/object/traj' # publishes end-effector planned trajectory on this topic


class TestInterpolation:

    def __init__(self):

        # Name of node
        self.name = rospy.get_name()
        # self.loadPushdata()
        self.loadFreeMotionData()

    def loadFreeMotionData(self):
        # read from file at the moment
        path2file = os.path.join(ROOT_DIR, 'data/example_free_motion_object3D_TO_data.npy')
        with open(path2file, 'rb') as f:
            data = np.load(f)

        print(data)
        exit
        self.trajObjPlan = data
        self.new_Objtraj_publisher = rospy.Publisher(NEW_TRAJ_OBJ_TOPIC, Float64MultiArray, queue_size=1)

        # time.sleep(2.0) # wait for initialisation to complete

    def loadPushData(self):

        # read from file at the moment
        path2file = os.path.join(ROOT_DIR,'data/example_1KUKA_Pushing_BoxonTable.npy')
        with open(path2file, 'rb') as f:
            data = np.load(f)

        # time
        self.time = np.array(data[10,:].reshape(1,data[10,:].shape[0]))
        # robot info
        self.posvelRobotPlan =  np.vstack((np.array(data[6:8,:]), np.array(data[8:10,:])))
        self.trajRobotPlan = np.vstack((self.time, self.posvelRobotPlan))
        # object info
        self.posvelObjPlan = np.vstack((np.array(data[0:3,:]), np.array(data[3:6,:])))
        self.trajObjPlan = np.vstack((self.time, self.posvelObjPlan))

        self.new_Robottraj_publisher = rospy.Publisher(NEW_TRAJ_ROBOT_TOPIC, Float64MultiArray, queue_size=1)
        self.new_Objtraj_publisher = rospy.Publisher(NEW_TRAJ_OBJ_TOPIC, Float64MultiArray, queue_size=1)

        # time.sleep(2.0) # wait for initialisation to complete

    def publishRobotTrajectory(self, event):

        message = self.np2DtoROSmsg(self.trajRobotPlan)

        self.new_Robottraj_publisher.publish(message)

    def publishObjTrajectory(self, event):

        message = self.np2DtoROSmsg(self.trajObjPlan)

        self.new_Objtraj_publisher.publish(message)


    def np2DtoROSmsg(self, data2Darray):

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
    rospy.init_node('test_ros_interpol_interface', anonymous=True)
    freq = 10
    TestInterpolation = TestInterpolation()
    rospy.loginfo("%s: node started.", TestInterpolation.name)

    # TestInterpolation.writeCallbackTimerRobot = rospy.Timer(rospy.Duration(1.0/float(freq)), TestInterpolation.publishRobotTrajectory)
    TestInterpolation.writeCallbackTimerObj = rospy.Timer(rospy.Duration(1.0/float(freq)), TestInterpolation.publishObjTrajectory)


    rospy.spin()
