#!/usr/bin/env python3
# license removed for brevity
import rospkg
import rospy
import os
import numpy as np


# ROS messages types of the real robot
from iiwa_msgs.msg import JointPosition, JointQuantity
from sensor_msgs.msg import JointState


SIM_CMD_JOINT_STATE_TOPIC = 'ros_pybullet_interface/joint_state/target'
REAL_ROBOT_CMD_JOINT_COMMAND_TOPIC = 'command/JointPosition' # commands joint states on this topic

REAL_ROBOT_JOINT_STATE_TOPIC = 'state/JointPosition'
SIM_ROBOT_JOINT_STATE_TOPIC = 'ros_pybullet_interface/joint_state/target'

JOINT_NAMES = ["IIWA_Joint_0","IIWA_Joint_1","IIWA_Joint_2","IIWA_Joint_3","IIWA_Joint_4",\
                "IIWA_Joint_5","IIWA_Joint_6"]

MIN_JOINT_LIMITS = [-169, -100, -169, -119, -169, -119, -173]
MAX_JOINT_LIMITS = [ 169,  100,  169,  119,  169,  119, 173]

class SimCmdToandFromROSSmartServo(object):
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
        node_name = "SimCmdToandFromROSSmartServo"
        self.name = f"{robot_name}_{node_name}"

        # ----------------------------------------------------------------------
        # incoming messaging (update simulated robots)
        # ----------------------------------------------------------------------

        # Setup ros publisher to update simulated robots
        sim_state_publishers_topic_name = f"{robot_name}/{SIM_ROBOT_JOINT_STATE_TOPIC}"
        self.sim_joint_state_command_publisher = rospy.Publisher(sim_state_publishers_topic_name, JointState, queue_size=1)

        # Setup subscriber that reads commanded robot state
        subscr_real_state_topic_name =  f"{robot_name}/{REAL_ROBOT_JOINT_STATE_TOPIC}"
        rospy.Subscriber(subscr_real_state_topic_name, JointPosition, self.republishStates)

        # ----------------------------------------------------------------------
        # outgoing messaging (command real robots)
        # ----------------------------------------------------------------------

        # command robots only if the flag is activated
        if cmd_robot_flag:
            # Setup ros publisher to cmd real robots
            real_world_publishers_topic_name = f"{robot_name}/{REAL_ROBOT_CMD_JOINT_COMMAND_TOPIC}"
            self.real_world_target_joint_command_publisher = rospy.Publisher(real_world_publishers_topic_name, JointPosition, queue_size=1)

            # Setup subscriber that reads commanded robot state
            subscr_sim_cmd_topic_name =  f"{robot_name}/{SIM_CMD_JOINT_STATE_TOPIC}"
            rospy.Subscriber(subscr_sim_cmd_topic_name, JointState, self.republishCmds)


    def republishStates(self, msg):

        # ----------------------------------------------------------------------
        # read state from real robot
        # ----------------------------------------------------------------------
        cur_joint_pos = msg.position
        current_joint_position = np.asarray([cur_joint_pos.a1,
                                             cur_joint_pos.a2,
                                             cur_joint_pos.a3,
                                             cur_joint_pos.a4,
                                             cur_joint_pos.a5,
                                             cur_joint_pos.a6,
                                             cur_joint_pos.a7])

        # ----------------------------------------------------------------------
        # update state of the simulated robot
        # ----------------------------------------------------------------------
        sim_msg = JointState(
            name = JOINT_NAMES,
            position = current_joint_position,
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
        ros_smservo_msg = JointPosition()
        ros_smservo_msg.header.frame_id = "Joint_position"

        joint_pos_submsg = JointQuantity()

        ros_smservo_msg.header.stamp = rospy.Time.now()

        joint_pos_submsg.a1 = cmd_q[0]
        joint_pos_submsg.a2 = cmd_q[1]
        joint_pos_submsg.a3 = cmd_q[2]
        joint_pos_submsg.a4 = cmd_q[3]
        joint_pos_submsg.a5 = cmd_q[4]
        joint_pos_submsg.a6 = cmd_q[5]
        joint_pos_submsg.a7 = cmd_q[6]

        ros_smservo_msg.position = joint_pos_submsg

        # send to the robot
        self.real_world_target_joint_command_publisher.publish(ros_smservo_msg)


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
        Sim2ROSSmartServo = SimCmdToandFromROSSmartServo()

        rospy.loginfo("%s: Spawn state and command republisher", Sim2ROSSmartServo.name)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass