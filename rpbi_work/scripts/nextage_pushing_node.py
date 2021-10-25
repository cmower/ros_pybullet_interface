#!/usr/bin/env python3
import numpy
import rospy
from std_msgs.msg import Float64MultiArray
from ros_pybullet_interface.tf_interface import TfInterface

class Node:

    hz = 50
    dt = 1.0/float(hz)
    left_hand = 1
    right_hand = 0

    def __init__(self):

        # Init node
        rospy.init_node('nextage_pushing_node')
        self.left_hand_goal = numpy.array([-0.2, 0.2, 0.0])
        self.right_hand_goal = numpy.array([-0.2, -0.2, 0.0])
        self.operator_signal = [numpy.zeros(2), numpy.zeros(2)]

        # Setup tf interface
        self.tf = TfInterface()

        # Setup ros subscribers and timers
        rospy.Subscriber('operator_node/signal0', Float64MultiArray, self.operator_signal_callback, callback_args=0)
        rospy.Subscriber('operator_node/signal1', Float64MultiArray, self.operator_signal_callback, callback_args=1)
        rospy.Timer(rospy.Duration(self.dt), self.main_loop)

    def operator_signal_callback(self, msg, arg):
        self.operator_signal[arg] = self.dt*numpy.array(msg.data)

    def main_loop(self, event):
        self.left_hand_goal[:2] += self.operator_signal[self.left_hand]
        self.right_hand_goal[:2] += self.operator_signal[self.right_hand]
        self.tf.set_tf('table_top', 'left_hand_goal', position=self.left_hand_goal)
        self.tf.set_tf('table_top', 'right_hand_goal', position=self.right_hand_goal)

    def spin(self):
        rospy.spin()

def main():
    Node().spin()

if __name__ == '__main__':
    main()
