#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


class Node:

    def __init__(self):

        # Setup ros node
        rospy.init_node('remap_joint_state_positions_to_float_array_node')

        # Get parameters
        input_topic = rospy.get_param('~input_joint_state_topic')
        output_topic = rospy.get_param('~output_float_array_topic')

        # Setup publisher
        self.pub = rospy.Publisher(output_topic, Float64MultiArray, queue_size=10)

        # Setup subscriber
        rospy.Subscriber(input_topic, JointState, self.callback)

    def callback(self, msg_in):
        self.pub.publish(Float64MultiArray(data=msg.position))

    def spin(self):
        rospy.spin()


def main():
    Node().spin()


if __name__ == '__main__':
    main()
