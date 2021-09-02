#!/usr/bin/env python3
# license removed for brevity
import rospkg
import rospy
import os
import numpy as np


# ROS messages types of the real robot
from sensor_msgs.msg import JointState
from ipab_lwr_msgs.msg import FriState
from ipab_lwr_msgs.msg import FriCommandJointPosition


SIM_CMD_JOINT_STATE_TOPIC = 'ros_pybullet_interface/joint_state/target'
REAL_ROBOT_CMD_JOINT_COMMAND_TOPIC = '/lwr/commandJointPosition' # commands joint states on this topic

REAL_ROBOT_JOINT_STATE_TOPIC = '/kuka_lwr_state'
SIM_ROBOT_JOINT_STATE_TOPIC = 'ros_pybullet_interface/joint_state/target'

JOINT_NAMES = ["lwr_arm_0_joint","lwr_arm_1_joint","lwr_arm_2_joint","lwr_arm_3_joint","lwr_arm_4_joint","lwr_arm_5_joint","lwr_arm_6_joint"]

MIN_JOINT_LIMITS = [-169, -100, -169, -119, -169, -119, -173]
MAX_JOINT_LIMITS = [ 169,  100,  169,  119,  169,  119, 173]

MIN_DELTA_JOINT_LIMITS = [-4.95, -4.95, -4.95, -4.95, -4.95, -4.95, -4.95]
MAX_DELTA_JOINT_LIMITS = [4.95, 4.95, 4.95, 4.95, 4.95, 4.95, 4.95]

GLOBAL_OFFSET = [0., 0., 0., 0., 0., 0., 0.]
# GLOBAL_OFFSET = [2.5, 0., 0., 0., 0., 0., 0.]

class SimCmdToandFromROSLWRFRI(object):
    """docstring for ."""

    def __init__(self):

        # check if the name of the robot is provided
        if rospy.has_param('~robot_name'):
            robot_name = rospy.get_param('~robot_name')
        else:
            rospy.logerr(f"The name of the robot is not set in {rospy.get_name()}")
            sys.exit(0)

        cmd_robot_flag = False
        if rospy.has_param('~cmd_real_robot'):
            cmd_robot_flag = rospy.get_param('~cmd_real_robot')

        # set name of the node
        node_name = "SimCmdToandFromROSLWRFRI"
        self.name = f"{robot_name}_{node_name}"

        # init the latest robot state
        self.latest_robot_state = np.asarray([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.counter = 0
        # ----------------------------------------------------------------------
        # incoming messaging (update simulated robots)
        # ----------------------------------------------------------------------

        # Setup ros publisher to update simulated robots
        sim_state_publishers_topic_name = f"{robot_name}/{SIM_ROBOT_JOINT_STATE_TOPIC}"
        self.sim_joint_state_command_publisher = rospy.Publisher(sim_state_publishers_topic_name, JointState, queue_size=1)

        # Setup subscriber that reads commanded robot state
        subscr_real_state_topic_name =  f"{REAL_ROBOT_JOINT_STATE_TOPIC}"
        rospy.Subscriber(subscr_real_state_topic_name, FriState, self.republishStates)

        # ----------------------------------------------------------------------
        # outgoing messaging (command real robots)
        # ----------------------------------------------------------------------

        # command robots only if the flag is activated
        if cmd_robot_flag:
            # Setup ros publisher to cmd real robots
            real_world_publishers_topic_name = f"{REAL_ROBOT_CMD_JOINT_COMMAND_TOPIC}"
            self.real_world_target_joint_command_publisher = rospy.Publisher(real_world_publishers_topic_name, FriCommandJointPosition, queue_size=1)

            # Setup subscriber that reads commanded robot state
            subscr_sim_cmd_topic_name =  f"{robot_name}_visual/{SIM_CMD_JOINT_STATE_TOPIC}"
            rospy.Subscriber(subscr_sim_cmd_topic_name, JointState, self.republishCmds)


    def republishStates(self, msg):

        # ----------------------------------------------------------------------
        # read state from real robot
        # ----------------------------------------------------------------------
        current_joint_position = np.array(msg.jointPosition)

        # update the latest robot state
        self.latest_robot_state = current_joint_position

        # ----------------------------------------------------------------------
        # update state of the simulated robot
        # ----------------------------------------------------------------------
        sim_msg = JointState(
            name = JOINT_NAMES,
            position = current_joint_position + np.deg2rad(GLOBAL_OFFSET),
            # velocity = ,
            # effort = ,
        )
        sim_msg.header.stamp = rospy.Time.now()
        self.sim_joint_state_command_publisher.publish(sim_msg)


    def republishCmds(self, msg):

        # ----------------------------------------------------------------------
        # read command from simulated robot
        # ----------------------------------------------------------------------
        joint_values = msg.position

        # check if command is within the limits and clip
        cmd_q = np.zeros(len(joint_values))
        np.clip(joint_values, np.deg2rad(MIN_JOINT_LIMITS), np.deg2rad(MAX_JOINT_LIMITS), out=cmd_q)

        # ----------------------------------------------------------------------
        # command the robot
        # ----------------------------------------------------------------------

        # initial the message
        ros_smservo_msg = FriCommandJointPosition()
        ros_smservo_msg.header.frame_id = "Joint_position"

        ros_smservo_msg.header.stamp = rospy.Time.now()

        ros_smservo_msg.jointPosition = cmd_q - np.deg2rad(GLOBAL_OFFSET)
        # send to the robot
        self.real_world_target_joint_command_publisher.publish(ros_smservo_msg)

        self.counter += 1

    def cleanShutdown(self):
        print('')
        rospy.loginfo("%s: Sending to safe configuration", self.name)
        # Shut down publishers
        self.real_world_target_joint_command_publisher.shutdown()
        self.sim_joint_state_command_publisher.shutdown()
        rospy.sleep(1.0)


if __name__ == '__main__':

    try:
        # Initialize node
        rospy.init_node("ros_republish_robot_state_and_commands", anonymous=True)
        # Initialize node class
        Sim2ROSLWRFRI = SimCmdToandFromROSLWRFRI()

        rospy.loginfo("%s: Spawn state and command republisher", Sim2ROSLWRFRI.name)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
