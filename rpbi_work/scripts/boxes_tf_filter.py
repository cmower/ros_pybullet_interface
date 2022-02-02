#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import UInt8
from ros_pybullet_interface.tf_interface import TfInterface

"""

Publishes flag that identifies closest object out of 3 to a base frame
within a threshold radius. A UInt8 is published at a 100Hz sampling
frequency.

flag=0: nothing detected within threshold
flag=1: object 1 is within threshold and closest to robot
flag=2: object 2 is within threshold and closest to robot
flag=3: object 3 is within threshold and closest to robot

"""

class Node:

    HZ = 100
    dt = 1.0/float(HZ)

    def __init__(self):

        # Setup ros
        rospy.init_node('boxes_tf_filter_node')

        # Get parameters
        self.base_frame = rospy.get_param('~base_frame')
        self.objects_frame = [
            rospy.get_param('~object_1_frame'),
            rospy.get_param('~object_2_frame'),
            rospy.get_param('~object_3_frame'),
        ]

        self.threshold_radius = rospy.get_param('~threshold_radius')

        # Setup tf interface
        self.tf = TfInterface()

        # Setup publisher
        self.pub = rospy.Publisher('object_filter_flag', UInt8, queue_size=10)

        # Start main loop
        rospy.Timer(rospy.Duration(self.dt), self.main_loop)

    def main_loop(self):
        flag = 0  # i.e. nothing within threshold
        distances = []
        for i, frame in enumerate(self.objects_frame):
            pos, rot = self.tf.get_frame(self.base_frame, frame)
            if pos is None:
                distances.append(100000.0)
            else:
                distances.append(np.linalg.norm(np.array(pos)))
        if min(distances) < self.threshold_radius:
            flag = np.argmin(distances)+1
        self.pub.publish(UInt8(data=flag))

    def spin(self):
        rospy.spin()

def main():
    Node().spin()

if __name__ == '__main__':
    main()
