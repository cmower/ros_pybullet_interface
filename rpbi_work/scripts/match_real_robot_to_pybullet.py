#!/usr/bin/env python3
import sys
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


"""Moves robot to current pybullet state."""

# larm_id = [
#     'LARM_JOINT0',
#     'LARM_JOINT1',
#     'LARM_JOINT2',
#     'LARM_JOINT3',
#     'LARM_JOINT4',
#     'LARM_JOINT5',
# ]

# rarm_id = [
#     'RARM_JOINT0',
#     'RARM_JOINT1',
#     'RARM_JOINT2',
#     'RARM_JOINT3',
#     'RARM_JOINT4',
#     'RARM_JOINT5',
# ]

# torso_id = ['CHEST_JOINT0']

# head_id = [
#     'HEAD_JOINT0',
#     'HEAD_JOINT1',
# ]

controller_joints_torso = ['CHEST_JOINT0']
controller_joints_head = ['HEAD_JOINT0', 'HEAD_JOINT1']
controller_joints_arm_left = ['LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5']
controller_joints_arm_right = ['RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5']
controller_joints = controller_joints_arm_left + controller_joints_arm_right + controller_joints_torso + controller_joints_head

class Node:

    hz = 50
    dt = 1.0/float(hz)

    Tmax = 5.0

    def __init__(self):

        # Setup ros node
        rospy.init_node('match_real_robot_to_pybullet_node')

        # Setup publisher
        self.pub = rospy.Publisher('nextagea/streaming_controller/command', Float64MultiArray, queue_size=10)

        # Get current real robot state
        msg = rospy.wait_for_message('/nextagea/joint_states', JointState)
        self.qstart = self.resolve_joint_msg_order(msg)
        # self.q = self.qstart.copy()

        # Get current pybullet joint state
        msg = rospy.wait_for_message('/rpbi/nextage/joint_state', JointState)
        self.qgoal = self.resolve_joint_msg_order(msg)

        # Start main loop (interpolation)
        self.start_time = rospy.Time.now().to_sec()
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.interpolate)

    def resolve_joint_msg_order(self, msg):
        cmd = []
        for i, name in enumerate(controller_joints):
            joint_index = msg.name.index(name)
            joint_position = msg.position[joint_index]
            cmd.append(joint_position)
        return np.array(cmd)

    def interpolate(self, event):
        tcurr = rospy.Time.now().to_sec() - self.start_time
        alpha = tcurr/self.Tmax
        q = alpha*self.qgoal + (1-alpha)*self.qstart
        self.pub.publish(Float64MultiArray(data=q))
        rospy.loginfo('Commanded robot, alpha=%.2f' % alpha)
        if alpha > 1.0:
            rospy.loginfo('Robot has reached pybullet state.')
            self.timer.shutdown()
            sys.exit(0)


    def spin(self):
        rospy.spin()


def main():
    Node().spin()


if __name__ == '__main__':
    main()
