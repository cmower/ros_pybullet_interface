#!/usr/bin/env python3
import sys
import math
import time
import rospy
from std_msgs.msg import Int64
from sensor_msgs.msg import JointState
from ros_pybullet_interface.tf_interface import TfInterface

"""This implements a basic simulation example."""

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

        # Setup robot
        robot_name = 'LWR'
        visual_robot_name = 'visual_LWR'
        target_joint_state_topic = f'rpbi/{robot_name}/joint_state/target'
        visual_target_joint_state_topic = f'rpbi/{visual_robot_name}/joint_state/target'
        self.ndof = 7
        self.joint_index = 0
        self.position = [0.0]*self.ndof
        self.d = 1
        self.traj_index = 0
        self.joint_traj = [math.sin(0.5*2.0*math.pi*float(i)/100.0) for i in range(200)]
        self.pub = rospy.Publisher(target_joint_state_topic, JointState, queue_size=10)
        self.visual_pub = rospy.Publisher(visual_target_joint_state_topic, JointState, queue_size=10)
        rospy.Timer(dur, self.publish_joint_state)

        # Setup visual robot
        rospy.Timer(dur, self.update_visual_object)

    def active_callback(self, msg):
        self.active = bool(msg.data)

    def update_joint_index(self):
        self.joint_index += self.d
        if self.joint_index == self.ndof:
            self.joint_index -= 2
            self.d = -1
        if self.joint_index == -1:
            self.joint_index = 0
            self.d = 1

    def update_trajectory_index(self):
        self.traj_index += 1
        if self.traj_index == len(self.joint_traj):
            self.traj_index = 0
            self.position = [0.0]*self.ndof
            self.update_joint_index()

    def publish_joint_state(self, event):
        if not self.active: return
        self.position[self.joint_index] = self.joint_traj[self.traj_index]
        self.pub.publish(JointState(position=self.position))
        self.visual_pub.publish(JointState(position=self.position))
        self.update_trajectory_index()

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
