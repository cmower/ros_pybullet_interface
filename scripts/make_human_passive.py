#!/usr/bin/env python3
# license removed for brevity
import rospy
import numpy as np

# ROS message types
from sensor_msgs.msg import JointState

SIM_HUMAN_JOINT_STATE_TOPIC = 'human/ros_pybullet_interface/joint_state/current'
SIM_HUMAN_JOINT_COMMAND_TOPIC = 'human/ros_pybullet_interface/joint_state/target'


NOMINAL_JOINT = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 70.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


class SimPassiveHuman(object):
    """docstring for ."""

    def __init__(self):

        # set name of the node
        node_name = "SimPassiveHuman"
        self.name = f"{node_name}"

        # Setup ros publisher to update human state
        pub_next_human_state_topic_name = f"{SIM_HUMAN_JOINT_COMMAND_TOPIC}"
        self.pub_next_human_joint_state_command_publisher = rospy.Publisher(
            pub_next_human_state_topic_name, JointState)

        # Setup subscriber that reads current human state
        subscr_current_human_state_topic_name = f"{SIM_HUMAN_JOINT_STATE_TOPIC}"
        rospy.Subscriber(subscr_current_human_state_topic_name, JointState, self.republishStates)

        # init message
        self.previous_msg = None

    def republishStates(self, msg):
        # if self.previous_msg is not None:
        # if any(np.abs(np.array(msg.position)[15:19] - np.array(self.previous_msg.position)[15:19]) > np.deg2rad(0.0)):
        new_msg = msg
        custom_joints = NOMINAL_JOINT[0:15] + \
            list(np.array(msg.position)[15:20]) + NOMINAL_JOINT[20:]
        new_msg.position = custom_joints
        self.pub_next_human_joint_state_command_publisher.publish(msg)

        # self.previous_msg = msg

    def cleanShutdown(self):
        print('')
        rospy.loginfo("%s: Sending to safe configuration", self.name)
        # Shut down publishers
        self.pub_next_human_joint_state_command_publisher.shutdown()
        rospy.sleep(1.0)


if __name__ == '__main__':

    try:
        # Initialize node
        rospy.init_node("make_human_passive", anonymous=True)
        # Initialize node class
        SimPassiveHuman = SimPassiveHuman()

        rospy.loginfo("%s: Spawn republisher to make human passive", SimPassiveHuman.name)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
