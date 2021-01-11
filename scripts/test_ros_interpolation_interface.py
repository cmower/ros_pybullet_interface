#!/usr/bin/env python3
import sys
import math
import time
import numpy as np
import tf2_ros
import rospy
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R


class TestInterpolation:

    def __init__(self):
        self.traj_index = 0

        # make a line along Z
        num_samplesLin = 100

        # make a simple linear motion
        Xtar1 = -0.5
        Ytar1 = -0.5
        Ztar1 = 0.0
        linearMotion = np.zeros((num_samplesLin, 3))
        linearMotion[:,0] = np.linspace(0.5, Xtar1, num=int(num_samplesLin))
        linearMotion[:,1] = np.linspace(0.5, Ytar1, num=int(num_samplesLin))
        linearMotion[:,2] = np.linspace(0.0, Ztar1, num=int(num_samplesLin))


        eeOri_traj = [1.5707963267948966, 0.0, 0.]
        eeOri_traj_Rot = R.from_rotvec(eeOri_traj)
        self.eeOri_traj = eeOri_traj_Rot.as_quat()

        # self.eePos_traj = np.vstack((linearMotion,transitionPtMotion, circularMotion.T))
        self.eePos_traj = np.vstack((linearMotion))

        self.tfBroadcaster = tf2_ros.TransformBroadcaster()
        time.sleep(2.0) # wait for initialisation to complete


    def updateTrajIndex(self):
        self.traj_index += 1

        if self.traj_index >= 100:
            rospy.loginfo("Test reset")
            self.traj_index = 0
            

    def publishBoxState(self, event):
        # Retrieve position and orientation
        position = self.eePos_traj[self.traj_index, :]
        orientation = self.eeOri_traj#[self.traj_index, :]

        # Pack pose msg
        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'ros_pybullet_interface/world'
        msg.child_frame_id = 'ros_pybullet_interface/box'
        msg.transform.translation.x = position[0]
        msg.transform.translation.y = position[1]
        msg.transform.translation.z = position[2]
        msg.transform.rotation.x = orientation[0]
        msg.transform.rotation.y = orientation[1]
        msg.transform.rotation.z = orientation[2]
        msg.transform.rotation.w = orientation[3] # NOTE: the ordering here may be wrong

        # Publish msg
        self.tfBroadcaster.sendTransform(msg)
        self.updateTrajIndex()


if __name__=='__main__':
    rospy.init_node('test_ros_interpolation_interface', anonymous=True)
    freq = 10
    TestInterpolation = TestInterpolation()

    TestInterpolation.writeCallbackTimer = rospy.Timer(rospy.Duration(1.0/float(freq)), TestInterpolation.publishBoxState)
    rospy.spin()
