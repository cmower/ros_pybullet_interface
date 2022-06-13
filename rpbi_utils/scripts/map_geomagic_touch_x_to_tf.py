#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from custom_ros_tools.tf import TfInterface

"""
This node maps geometry_msgs/PoseStamped messages from the TouchX driver to tf messages.
"""

class Node:

    def __init__(self):
        rospy.init_node('map_geomagic_touch_x_to_tf_node')
        self.tf = TfInterface()
        topic = rospy.get_param('~topic', 'Left/Pose')
        rospy.Subscriber(topic, PoseStamped, self.pose_callback)
        self.linear_scale = rospy.get_param('~linear_scale', 1.0)
        self.parent = rospy.get_param('~parent_frame_id', 'map')
        self.child = rospy.get_param('~child_frame_id', 'haptic')        
        
    def pose_callback(self, msg):
        p = self.linear_scale*np.array([getattr(msg.pose.position, d) for d in 'xyz'])
        q = [getattr(msg.pose.orientation, d) for d in 'xyzw']
        self.tf.set_tf(self.parent, self.child, p, q)
        
    def spin(self):
        rospy.spin()

def main():
    Node().spin()

if __name__ == '__main__':
    main()
        
    
        
