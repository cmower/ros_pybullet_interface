#!/usr/bin/env python3
# license removed for brevity
import rospkg
import rospy
import os
import numpy as np

# ROS message types
from sensor_msgs.msg import JointState

# ROS messages types of the real robot
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import Float32


from topic_tools.srv import MuxSelect

NDOF = 7

SIM_CMD_JOINT_STATE_TOPIC = 'ros_pybullet_interface/joint_state/target'
SIM_CMD_STIFFNESS_TOPIC = 'ros_pybullet_interface/end_effector/stiffness/target'
# REAL_ROBOT_CMD_JOINT_COMMAND_TOPIC = 'PositionController/command' # commands joint states on this topic
REAL_ROBOT_CMD_JOINT_COMMAND_TOPIC = 'JointLimit/command' # commands joint states on this topic


REAL_ROBOT_JOINT_STATE_TOPIC = 'joint_states'
SIM_ROBOT_JOINT_STATE_TOPIC = 'ros_pybullet_interface/joint_state/target'

JOINT_NAMES = ["IIWA_Joint_0","IIWA_Joint_1","IIWA_Joint_2","IIWA_Joint_3","IIWA_Joint_4",\
                "IIWA_Joint_5","IIWA_Joint_6"]

# MIN_JOINT_LIMITS = [-169, -115, -169, -119, -169, -119, -174]
# MAX_JOINT_LIMITS = [ 169,  115,  169,  119,  169,  119, 174]

MIN_JOINT_LIMITS = [-165, -115, -165, -115, -165, -115, -170]
MAX_JOINT_LIMITS = [ 165,  115,  165,  115,  165,  115, 170]

MIN_STIFFNESS_LIMITS = [ 200.,  200.,  200.,  50.,  50.,  50.]
MAX_STIFFNESS_LIMITS = [ 4500.,  4500.,  4500.,  300.,  300.,  300.]
DEFAULT_LIN_STIFF = 4000.
DEFAULT_ANG_STIFF = 100.


class SimCmdToandFromROSFRI(object):
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
        node_name = "SimCmdToandFromROSFRI"
        self.name = f"{robot_name}_{node_name}"

        # init the latest robot state
        self.latest_robot_state = np.asarray([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.stiffness = [DEFAULT_LIN_STIFF,DEFAULT_LIN_STIFF,2000.,DEFAULT_ANG_STIFF,DEFAULT_ANG_STIFF,DEFAULT_ANG_STIFF]
        self.counter = 0
        # ----------------------------------------------------------------------
        # incoming messaging (update simulated robots)
        # ----------------------------------------------------------------------

        # Setup ros publisher to update simulated robots
        sim_state_publishers_topic_name = f"{robot_name}/{SIM_ROBOT_JOINT_STATE_TOPIC}"
        self.sim_joint_state_command_publisher = rospy.Publisher(sim_state_publishers_topic_name, JointState, queue_size=1)

        # Setup subscriber that reads commanded robot state
        subscr_real_state_topic_name =  f"{robot_name}/{REAL_ROBOT_JOINT_STATE_TOPIC}"
        rospy.Subscriber(subscr_real_state_topic_name, JointState, self.republishStates)


        # ----------------------------------------------------------------------
        # outgoing messaging (command real robots)
        # ----------------------------------------------------------------------

        # command robots only if the flag is activated
        if cmd_robot_flag:

            select_srv = rospy.ServiceProxy(f"{robot_name}_mux_joint_position/select", MuxSelect)
            # activate IK-based commanding
            select_srv(f"{robot_name}/JointLimit/command")

            # Setup ros publisher to cmd real robots
            real_world_publishers_topic_name = f"{robot_name}/{REAL_ROBOT_CMD_JOINT_COMMAND_TOPIC}"
            self.real_world_target_joint_command_publisher = rospy.Publisher(real_world_publishers_topic_name, Float64MultiArray, queue_size=1)

            # Setup subscriber that reads commanded robot state
            subscr_sim_cmd_topic_name =  f"{robot_name}_visual/{SIM_CMD_JOINT_STATE_TOPIC}"
            rospy.Subscriber(subscr_sim_cmd_topic_name, JointState, self.republishCmds)

            # Setup subscriber that reads commanded robot stiffness
            subscr_stiffness_topic_name =  f"{robot_name}_visual/{SIM_CMD_STIFFNESS_TOPIC}"
            rospy.Subscriber(subscr_stiffness_topic_name, Float32, self.readStiffness)


    def readStiffness(self, msg):

        stiffnessRatio = 0.1
        linearStiffness = [DEFAULT_LIN_STIFF, DEFAULT_LIN_STIFF, msg.data]
        angularStiffness = [stiffnessRatio*msg.data, stiffnessRatio*msg.data, DEFAULT_ANG_STIFF]

        stiffness = list(linearStiffness) + list(angularStiffness)
        cliped_stiffness = np.zeros(len(stiffness))
        np.clip(np.array(stiffness), MIN_STIFFNESS_LIMITS, MAX_STIFFNESS_LIMITS, out=cliped_stiffness)

        self.stiffness = list(cliped_stiffness)


    def republishStates(self, msg):

        # ----------------------------------------------------------------------
        # read state from real robot
        # ----------------------------------------------------------------------
        current_joint_position = np.asarray(list(msg.position))
        current_joint_velocity = np.asarray(list(msg.velocity))
        current_joint_effort = np.asarray(list(msg.effort))

        # # update the latest robot state
        # self.latest_robot_state = current_joint_position
        #
        # # ----------------------------------------------------------------------
        # # update state of the simulated robot
        # # ----------------------------------------------------------------------
        sim_msg = JointState(
            name = JOINT_NAMES,
            position = current_joint_position[:7],
            velocity = current_joint_velocity[:7],
            effort = current_joint_effort[:7],
        )
        sim_msg.header.stamp = rospy.Time.now()

        # ----------------------------------------------------------------------
        # read state from real robot and update state of the simulated robot
        # ----------------------------------------------------------------------
        # sim_msg = msg
        self.sim_joint_state_command_publisher.publish(sim_msg)


    def republishCmds(self, msg):

        # ----------------------------------------------------------------------
        # read command from simulated robot
        # ----------------------------------------------------------------------
        joint_values = msg.position
        seq_counter = msg.header.seq

        # check if command is within the limits and clip
        cmd_q = np.zeros(len(joint_values))
        np.clip(joint_values, np.deg2rad(MIN_JOINT_LIMITS), np.deg2rad(MAX_JOINT_LIMITS), out=cmd_q)

        # ----------------------------------------------------------------------
        # command the robot
        # ----------------------------------------------------------------------

        # initial the message
        msg = Float64MultiArray()
        msg.layout.dim.append(MultiArrayDimension())
        # info for reconstruction of the 2D array
        msg.layout.dim[0].label  = "columns"
        msg.layout.dim[0].size   = NDOF

        #  fill in positions of the robot
        msg.data = list(cmd_q) + self.stiffness

        # send to the robot
        self.real_world_target_joint_command_publisher.publish(msg)

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
        SimCmdToandFromROSFRI = SimCmdToandFromROSFRI()

        rospy.loginfo("%s: Spawn state and command republisher", SimCmdToandFromROSFRI.name)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
