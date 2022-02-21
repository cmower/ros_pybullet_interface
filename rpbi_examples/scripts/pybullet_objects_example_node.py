#!/usr/bin/env python3
import rospy

class Node:

    def __init__(self):
        rospy.init_node('pybullet_objects_example_node')

    def spin(self):
        rospy.spin()


def main():
    Node().spin()


if __name__ == '__main__':
    main()
