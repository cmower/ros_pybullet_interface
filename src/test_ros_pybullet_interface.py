#!/usr/bin/env python
import sys

import rospy
import math
from sensor_msgs.msg import JointState

NDOF = 7
TARGET_JOINT_STATE_TOPIC = 'ros_pybullet_interface/joint_state/target'

class Test:

    def __init__(self):
        self.joint_index = 0
        self.position = [0.0]*NDOF
        self.d = 1
        self.traj_index = 0
        self.joint_traj = [math.sin(0.5*2.0*math.pi*float(i)/100.0) for i in range(200)]
        self.pub = rospy.Publisher(TARGET_JOINT_STATE_TOPIC, JointState, queue_size=10)

    def updateJointIndex(self):
        self.joint_index += self.d
        if self.joint_index == NDOF:
            self.joint_index -= 2
            self.d = -1
        if self.joint_index == 0:
            self.d = 1

    def updateTrajIndex(self):
        self.traj_index += 1
        if self.traj_index == len(self.joint_traj):
            self.traj_index = 0
            self.position = [0.0]*NDOF
            self.updateJointIndex()

    def publishJointState(self, event):
        self.position[self.joint_index] = self.joint_traj[self.traj_index]
        self.pub.publish(JointState(position=self.position))
        self.updateTrajIndex()

if __name__=='__main__':
    rospy.init_node('test_ros_pybullet_interface', anonymous=True)
    freq = 50
    rospy.Timer(rospy.Duration(1.0/float(freq)), Test().publishJointState)
    rospy.spin()
