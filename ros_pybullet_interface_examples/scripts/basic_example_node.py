#!/usr/bin/env python3
import sys
import math
import time
import rospy
from std_msgs.msg import Int64
from sensor_msgs.msg import JointState
from ros_pybullet_interface.tf_interface import TfInterface

"""This implements a basic simulation example with the kuka and several objects."""

class Node:

    def __init__(self):

        # Init node
        rospy.init_node('basic_example_node', anonymous=True)
        self.tf = TfInterface()
        freq = 50
        dur = rospy.Duration(1.0/float(freq))

        # Setup subscriber to rpbi/active
        rospy.Subscriber('rpbi/active', Int64, self.active_callback)
        self.active = True

        # Setup visual robot
        rospy.Timer(dur, self.update_visual_object)

    def active_callback(self, msg):
        self.active = bool(msg.data)

    def update_visual_object(self, event):
        if not self.active: return
        tf_frame_id = 'visual_object'
        t = rospy.Time.now().to_sec()
        pos = [0.3*math.sin(t), 0.2*math.cos(t), 0.2]
        self.tf.set_tf('rpbi/world', tf_frame_id, pos, [0, 0, 0, 1])

    def spin(self):
        rospy.spin()


def main():
    Node().spin()


if __name__=='__main__':
    main()
