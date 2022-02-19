#!/usr/bin/env python3
import sys
import math
import rospy
from std_msgs.msg import Int64
from sensor_msgs.msg import JointState
from ros_pybullet_interface.srv import RobotInfo


class Node:

    hz = 50
    dt = 1.0/float(hz)

    def __init__(self):

        # Init node
        rospy.init_node('basic_robot_example_node')

        # Get robot name
        if rospy.has_param('~robot_name'):
            robot_name = rospy.get_param('~robot_name')
        elif len(sys.argv) > 1:
            robot_name = sys.argv[1]
        else:
            raise ValueError("robot name not specified, either set the '~robot_name' node parameter or via command line args")
        target_joint_state_topic = f'rpbi/{robot_name}/joint_states/target'

        # Other setup
        dur = rospy.Duration(self.dt)

        # Subscribers
        self.active = False
        rospy.Subscriber('rpbi/status', Int64, self.active_callback)

        # Get ndof
        service = f'rpbi/{robot_name}/robot_info'
        rospy.wait_for_service(service)
        try:
            handle = rospy.ServiceProxy(service, RobotInfo)
            res = handle()
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            sys.exit(0)
        self.ndof = sum([j.jointTypeStr != 'JOINT_FIXED' for j in res.joint_info])
        self.name = [j.jointName for j in res.joint_info if j.jointTypeStr != 'JOINT_FIXED']

        # Setup for joint updates
        self.joint_index = 0
        self.position = [0.0]*self.ndof
        self.d = 1
        self.traj_index = 0
        self.joint_traj = [math.sin(0.5*2.0*math.pi*float(i)/100.0) for i in range(200)]

        # Setup joint target state publisher
        self.pub = rospy.Publisher(target_joint_state_topic, JointState, queue_size=10)

        # Start timer
        rospy.Timer(dur, self.publish_joint_state)

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
        self.pub.publish(JointState(name=self.name, position=self.position))
        self.update_trajectory_index()

    def spin(self):
        rospy.spin()


def main():
    Node().spin()


if __name__=='__main__':
    main()
