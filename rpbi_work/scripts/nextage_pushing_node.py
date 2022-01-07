#!/usr/bin/env python3
import numpy
import rospy
from std_msgs.msg import Float64MultiArray
from ros_pybullet_interface.tf_interface import TfInterface
from ros_pybullet_interface.config import load_config

class Node:

    hz = 50
    dt = 1.0/float(hz)
    left_hand = 1
    right_hand = 0

    def __init__(self):

        # Init node
        rospy.init_node('nextage_pushing_node')

        # Get config
        file_name = rospy.get_param('~config_filename')
        config = load_config(file_name)

        self.left_hand_goal = numpy.array(config['left_hand_goal']) #numpy.array([-0.2, 0.2, 0.0])
        self.right_hand_goal = numpy.array(config['right_hand_goal']) #numpy.array([-0.2, -0.2, 0.0])
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
        #base_frame = 'table_top'
        base_frame = 'sim/nextage_base'
        self.tf.set_tf(base_frame, 'left_hand_goal', position=self.left_hand_goal)
        self.tf.set_tf(base_frame, 'right_hand_goal', position=self.right_hand_goal)

    def spin(self):
        rospy.spin()

def main():
    Node().spin()

if __name__ == '__main__':
    main()
