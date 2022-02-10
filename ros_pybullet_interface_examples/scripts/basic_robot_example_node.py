#!/usr/bin/env python3
import sys
import math
import rospy
from std_msgs.msg import Int64
from sensor_msgs.msg import JointState
from ros_pybullet_interface.srv import PybulletRobotJointInfo
from rpbi_work.srv import Toggle, ToggleRequest

class Node:

    # hz = 50
    hz = 30
    dt = 1.0/float(hz)

    def __init__(self):

        # Init node
        rospy.init_node('basic_robot_example_node')

        # Get robot name
        robot_name = rospy.get_param('~robot_name')
        target_joint_state_topic = f'rpbi/{robot_name}/joint_state/target'

        # Other setup
        dur = rospy.Duration(self.dt)

        # Subscribers
        self.active = False
        rospy.Subscriber('rpbi/active', Int64, self.active_callback)

        # Get ndof
        service = 'pybullet_robot_joint_info'
        rospy.wait_for_service(service)
        try:
            handle = rospy.ServiceProxy(service, PybulletRobotJointInfo)
            res = handle(robot_name)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            sys.exit(0)
        self.ndof = res.ndof_active
        self.name = res.active_joint_name

        # Setup for joint updates
        self.joint_index = 0
        self.position = [0.0]*self.ndof
        self.d = 1
        self.traj_index = 0
        self.joint_traj = [math.sin(0.5*2.0*math.pi*float(i)/100.0) for i in range(100)]

        # Setup joint target state publisher
        self.pub = rospy.Publisher(target_joint_state_topic, JointState, queue_size=10)

        # Start timer
        input("Start robot example? [ENTER]")
        rospy.Timer(dur, self.publish_joint_state)

        success = True
        srv = 'toggle_remaper'
        rospy.wait_for_service(srv)
        try:
            handle = rospy.ServiceProxy(srv, Toggle)
            req = ToggleRequest(switch='on')
            resp = handle(req)
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)
            success = False

        if success:
            rospy.loginfo('started remapper')



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
