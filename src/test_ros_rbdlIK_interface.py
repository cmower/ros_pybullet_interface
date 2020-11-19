#!/usr/bin/env python3
import sys
import math
import time
import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped


TARGET_END_EFFECTOR_TOPIC = 'ros_pybullet_interface/end_effector/target' # publishes end-effector poses on this topic
r = 0.1 # radius


class TestIK:

    def __init__(self):
        self.traj_index = 0

        # make a line along Z
        num_samplesLin = 100

        # make a simple linear motion
        Xtar1 = -0.783
        Ytar1 = -0.1827
        Ztar1 = 0.571
        linearMotion = np.zeros((num_samplesLin, 3))
        linearMotion[:50,0] = np.linspace(-0.6823, Xtar1, num=int(num_samplesLin/2))
        linearMotion[:50,1] = np.linspace(-0.134, Ytar1, num=int(num_samplesLin/2))
        linearMotion[:50,2] = np.linspace(0.8425, Ztar1, num=int(num_samplesLin/2))

        # make a simple linear motion
        Xtar2 = -0.583
        Ytar2 = -0.1827
        Ztar2 = 0.3371
        linearMotion[50:,0] = np.linspace(Xtar1, Xtar2, num=int(num_samplesLin/2))
        linearMotion[50:,1] = np.linspace(Ytar1, Ytar2, num=int(num_samplesLin/2))
        linearMotion[50:,2] = np.linspace(Ztar1, Ztar2, num=int(num_samplesLin/2))

        # make a simple transition motion from linear to circular
        transitionPtMotion = np.asarray(linearMotion[-1,:]*30)

        # make a simple circlular motion
        num_samplesCirc = 500
        theta = np.linspace(0, 8*np.pi, num_samplesCirc)
        # generate the points on the circle
        x = r * np.cos(theta) + Xtar2
        y = r * np.sin(theta) + Ytar2
        circularMotion = np.vstack((x,y,np.asarray(num_samplesCirc*[Ztar2])))
        # self.eePos_traj = np.vstack((linearMotion, transitionPtMotion, circularMotion.T))

        transitionPtMotion = np.zeros((10, 3))
        transitionPtMotion[:,0] = np.linspace(Xtar2, x[0], num=10)
        transitionPtMotion[:,1] = np.linspace(Ytar2, y[0], num=10)
        transitionPtMotion[:,2] = np.linspace(Ztar2, Ztar2, num=10)

        self.eePos_traj = np.vstack((linearMotion,transitionPtMotion, circularMotion.T))

        self.pub = rospy.Publisher(TARGET_END_EFFECTOR_TOPIC, PoseStamped, queue_size=1)
        time.sleep(2.0) # wait for initialisation to complete


    def updateTrajIndex(self):
        self.traj_index += 1

        if self.traj_index < 50:
            rospy.loginfo("Linear motion 1")
        elif self.traj_index < 100:
            rospy.loginfo("Linear motion 2")
        elif self.traj_index < 110:
            rospy.loginfo("Transition motion")
        elif self.traj_index > 110:
            rospy.loginfo("Circular Motion")

        if self.traj_index == len(self.eePos_traj):
            rospy.loginfo("Test Motion is done")
            self.writeCallbackTimer.shutdown()


    def publishEEtargetState(self, event):
        # Retrieve position and orientation
        position = self.eePos_traj[self.traj_index, :]
        orientation = [0.,0.,0.,0.]

        # Pack pose msg
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]
        msg.pose.orientation.x = orientation[0]
        msg.pose.orientation.y = orientation[1]
        msg.pose.orientation.z = orientation[2]
        msg.pose.orientation.w = orientation[3] # NOTE: the ordering here may be wrong

        # Publish msg
        self.pub.publish(msg)
        self.updateTrajIndex()


if __name__=='__main__':
    rospy.init_node('test_ros_rbdlIK_interface', anonymous=True)
    freq = 10
    testIK = TestIK()
    testIK.writeCallbackTimer = rospy.Timer(rospy.Duration(1.0/float(freq)), testIK.publishEEtargetState)
    rospy.spin()
