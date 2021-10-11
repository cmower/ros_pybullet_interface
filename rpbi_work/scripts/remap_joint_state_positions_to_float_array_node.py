#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


"""Map JointState positions messages onto Float64MultiArray messages."""


class Node:


    def __init__(self):

        # Setup ros node
        rospy.init_node('remap_joint_state_positions_to_float_array_node')

        # Setup publisher
        # expect output topic to be remapped in launch
        self.pub = rospy.Publisher('output_float_array_topic', Float64MultiArray, queue_size=10)

        # Setup subscriber
        # expect input topic to be remapped in launch
        rospy.Subscriber('input_joint_state_topic', JointState, self.callback)

        rospy.loginfo('Remapper JointState.position -> Float64MultiArray initialized.')


    def callback(self, msg_in):
        self.pub.publish(Float64MultiArray(data=msg.position))


    def spin(self):
        rospy.spin()


def main():
    Node().spin()


if __name__ == '__main__':
    main()
