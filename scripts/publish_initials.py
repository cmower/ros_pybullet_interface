#!/usr/bin/env python3
# license removed for brevity
import rospkg
import rospy
import os
import argparse

import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation as R

# ROS message types
from sensor_msgs.msg import JointState

# ROS messages types of the real robot
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension

# ------------------------------------------------------------
#  REAL ROBOT
# ------------------------------------------------------------
FREQ = 100
NDOF = 7
REAL_ROBOT_TARGET_JOINT_STATE_TOPIC = 'PositionController/command' # commands joint states on this topic
REAL_ROBOT_JOINT_STATE_TOPIC = 'joint_states'

min_joints = [-169, -100, -169, -119, -169, -119, -173]
max_joints = [ 169,  100,  169,  119,  169,  119, 173]

# number of interpolation steps
NUM_STEPS  = 1000


def go2initial(robot_name=None, target_joint_position = np.asarray([0,0,0,0,0,0,0])):

    if robot_name==None:
        rospy.logerr(f"No robot name was given.")
        return False

    # Setup ros name for topics
    real_world_publishers_topic_name = f"{robot_name}/{REAL_ROBOT_TARGET_JOINT_STATE_TOPIC}"
    real_world_target_joint_state_publisher = rospy.Publisher(real_world_publishers_topic_name, Float64MultiArray, queue_size=1)

    # init node
    rospy.init_node('go2initial_configuration')
    r = rospy.Rate(FREQ) # 10hz

    # read the current configuration of the robot
    rospy.loginfo(f"Waiting for {robot_name}/{REAL_ROBOT_JOINT_STATE_TOPIC} topic, to read the curent configuration of the robot.")
    msgRobotState = rospy.wait_for_message(f"{robot_name}/{REAL_ROBOT_JOINT_STATE_TOPIC}", JointState)
    current_joint_position = np.asarray(list(msgRobotState.position))

    # transform degrees configuration to rads
    target_joint_position_rad = np.deg2rad(target_joint_position)
    # interpolate between current configuration and the goal configuration
    joint_values = np.linspace(current_joint_position, target_joint_position_rad, NUM_STEPS)

    # initial the message
    msg = Float64MultiArray()
    msg.layout.dim.append(MultiArrayDimension())
    # info for reconstruction of the 2D array
    msg.layout.dim[0].label  = "columns"
    msg.layout.dim[0].size   = NDOF

    # move robot to target config by publishing robot configuration states
    for i in range(NUM_STEPS):

        # add data as flattened numpy array
        msg.data = joint_values[i,:]

        # send to the robot
        real_world_target_joint_state_publisher.publish(msg)
        r.sleep()


if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Description of your program')
    parser.add_argument("--robot_name", help="Give the name of the robot")
    parser.add_argument("--target_config", nargs="+", help="Give the target configuration of the robot", type=float)
    args = parser.parse_args()

    if args.target_config:
        goal_q = np.asarray(list(args.target_config))
        rows_target = goal_q.shape[0]
        if rows_target != NDOF:
            rospy.logerr(f"Target configuration was given to the robot has wrong dimensions")
        else:

            np.clip(goal_q, min_joints, max_joints, out=goal_q)
            go2initial(args.robot_name, goal_q)

    else:
        rospy.loginfo(f"No target configuration was given to the robot.")


 # end
