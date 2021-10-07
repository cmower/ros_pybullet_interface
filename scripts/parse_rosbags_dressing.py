#!/usr/bin/env python3
# license removed for brevity
import rospy
import numpy as np
import tf2_ros
from scipy.spatial.transform import Rotation as R
from scipy.signal import butter, lfilter, freqz

import time
# ROS message types
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
import message_filters


ROBOT_JOINT_STATE_TOPIC = 'ros_pybullet_interface/joint_state/current'

SIM_HUMAN_JOINT_STATE_TOPIC = 'human/ros_pybullet_interface/joint_state/current'

# FORCE_TOPIC = 'yin_human/ros_pybullet_interface/joint_force_torque_sensor/end_effector'
FORCE_TOPIC = 'netft_data'

WORLD_FRAME_ID = 'ros_pybullet_interface/world'

# SHOULDER_FRAME_ID = "human/ros_pybullet_interface/robot/RightUpperArm_f1"
# ELBOW_FRAME_ID = "human/ros_pybullet_interface/robot/RightForeArm_f1"
# WRIST_FRAME_ID = "human/ros_pybullet_interface/robot/RightHand"


SHOULDER_FRAME_ID = "human/ros_pybullet_interface/robot/RightUpperArm_f1"
ELBOW_FRAME_ID = "kolias/RightArm"
WRIST_FRAME_ID = "kolias/RightHand"

END_EFFECTOR_FRAME_ID = 'ros_pybullet_interface/end_effector/target'

END_EFFECTOR_FRAME_ID = 'ros_pybullet_interface/robot/end_effector_sponge'


class ParseROSbagtoMat(object):
    """docstring for ."""

    def __init__(self):

        # set name of the node
        node_name = "ParseROSBAG2MAT"
        self.name = f"{node_name}"

        self.tf_buff = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tf_buff)

        self.robot_name = "yin"
        # self.robot_name = "yin_visual"

        # Setup subscriber that reads current human state
        subscr_current_human_state_topic_name = f"{SIM_HUMAN_JOINT_STATE_TOPIC}"
        sub_joints = message_filters.Subscriber(subscr_current_human_state_topic_name,
                                                JointState)
        # sub_joints.registerCallback(self.readJoints)

        # Setup subscriber that reads current human state
        subscr_current_robot_state_topic_name = f"{self.robot_name}/{ROBOT_JOINT_STATE_TOPIC}"
        sub_joints_robot = message_filters.Subscriber(subscr_current_robot_state_topic_name,
                                                      JointState)

        # Setup subscriber that reads force
        subscr_current_force_state_topic_name = f"{FORCE_TOPIC}"
        sub_force = message_filters.Subscriber(subscr_current_force_state_topic_name,
                                               WrenchStamped)
        # sub_force.registerCallback(self.readForce)

        ts = message_filters.ApproximateTimeSynchronizer(
            [sub_joints, sub_force, sub_joints_robot], 1, 1)
        ts.registerCallback(self.readJointsAndForce)

        # init storage lists
        self.human_joints = []
        self.robot_joints = []
        self.hand = []
        self.elbow = []
        self.shoulder = []
        self.force_local = []
        self.force_global = []
        self.robot_ee = []

        self.temp_time = rospy.Time.now()
        self.time_list = []

    def readJointsAndForce(self, msgJoints, msgForce, msgJointsRobot):

        joints_array = np.array(msgJoints.position)
        q_human = [joints_array[15],
                   joints_array[17], joints_array[18], joints_array[19]]
        self.human_joints.append(q_human)

        robot_joints_array = np.array(msgJointsRobot.position)
        self.robot_joints.append(robot_joints_array)

        robot_ee_pos, robot_ee_orient = self.readSpecificTF(
            f"{self.robot_name}/{END_EFFECTOR_FRAME_ID}")
        self.robot_ee.append(robot_ee_pos)

        force = [msgForce.wrench.force.x,
                 msgForce.wrench.force.y, msgForce.wrench.force.z]
        self.force_local.append(force)

        robot_ee_R = R.from_quat(robot_ee_orient)
        force_global = np.dot(robot_ee_R.as_matrix(), np.array(force))
        self.force_global.append(force_global)

        shoulder_pos, _ = self.readSpecificTF(f"{SHOULDER_FRAME_ID}")
        self.shoulder.append(shoulder_pos)
        elbow_pos, _ = self.readSpecificTF(f"{ELBOW_FRAME_ID}")
        self.elbow.append(elbow_pos)
        hand_pos, _ = self.readSpecificTF(f"{WRIST_FRAME_ID}")
        self.hand.append(hand_pos)

        t = rospy.Time.now()
        self.time_list.append((t-self.temp_time).to_sec())
        self.temp_time = t

    def readSpecificTF(self, topic_name):
        while 1:
            try:
                tf = self.tf_buff.lookup_transform(
                    WORLD_FRAME_ID, topic_name, rospy.Time())
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("Lost - something!", topic_name)

        pos = np.array([tf.transform.translation.x,
                       tf.transform.translation.y, tf.transform.translation.z])
        orient = np.array([tf.transform.rotation.x,
                           tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w])
        return pos, orient

    def readJoints(self, msg):
        joints_array = np.array(msg.position)
        q_human = [joints_array[15], joints_array[17], joints_array[18], joints_array[19]]
        self.human_joints.append(np.array(q_human))

    def readForce(self, msg):
        force = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]
        self.force.append(np.array(force))

    def parsingState(self, event):
        print("------------------------------------")
        time_array = np.asarray(self.time_list)
        print(np.mean(time_array))
        print(np.var(time_array))

        print("------------------------------------")
        joints_array = np.asarray(self.human_joints)
        print(joints_array.shape)
        joints_robots_array = np.asarray(self.robot_joints)
        print(joints_robots_array.shape)
        force_local_array = np.asarray(self.force_local)
        print(force_local_array.shape)
        force_global_array = np.asarray(self.force_global)
        print(force_global_array.shape)
        robot_ee_array = np.asarray(self.robot_ee)
        print(robot_ee_array.shape)
        shoulder_array = np.asarray(self.shoulder)
        print(shoulder_array.shape)
        elbow_array = np.asarray(self.elbow)
        print(elbow_array.shape)
        hand_array = np.asarray(self.hand)
        print(hand_array.shape)
        print(" ")

        print(robot_ee_array.shape)
        diff = np.diff(robot_ee_array, axis=0)
        norm_val_vel_raw = np.linalg.norm(diff, axis=1)
        velocity = filter_vel(diff)
        norm_val_vel = np.linalg.norm(velocity, axis=1)
        loc_zeros = np.argwhere(norm_val_vel < 0.0001)
        out_loc_zeros = np.concatenate(loc_zeros).ravel()
        group_zeros_idx = consecutive(out_loc_zeros)
        print("len(group_zeros_idx) ", len(group_zeros_idx))
        start_idx = 0
        end_idx = -1
        save = False
        if len(group_zeros_idx) == 3:
            print("-----------------Automatic cutting can be succesful--------------------------")
            start_idx = group_zeros_idx[1][-1] - 50
            end_idx = group_zeros_idx[2][1] + 50
            print("start_idx ", start_idx)
            print("end_idx ", end_idx)
            save = True
        start_idx = 750
        end_idx = 1950
        save = True

        plot_ee_vel(diff, norm_val_vel_raw, start_idx, end_idx, "raw")
        plot_ee_vel(velocity, norm_val_vel, start_idx, end_idx, "filtered")

        path = "/home/theo/Documents/data_dressing/data_real_world/dressing_data_2/new_parsed_data/bend_45/traj2"
        number_idx = "31_53"
        type_robot_data = "real"
        # type_robot_data = "sim"
        input_array = np.hstack(
            (hand_array[start_idx:end_idx-1, :], elbow_array[start_idx:end_idx-1, :]))
        input_array = np.hstack((input_array, shoulder_array[start_idx:end_idx-1, :]))
        input_array = np.hstack((input_array, joints_array[start_idx:end_idx-1, :]))
        input_array = np.hstack((input_array, robot_ee_array[start_idx:end_idx-1, :]))
        input_array = np.hstack((input_array, joints_robots_array[start_idx:end_idx-1, :]))

        output_array = np.hstack(
            (hand_array[start_idx+1:end_idx, :], elbow_array[start_idx+1:end_idx, :]))
        output_array = np.hstack((output_array, shoulder_array[start_idx+1:end_idx, :]))
        output_array = np.hstack((output_array, joints_array[start_idx+1:end_idx, :]))
        output_array = np.hstack((output_array, robot_ee_array[start_idx+1:end_idx, :]))
        output_array = np.hstack((output_array, joints_robots_array[start_idx+1:end_idx, :]))

        observation_array_raw = np.hstack(
            (force_local_array[start_idx:end_idx-1, :], force_global_array[start_idx:end_idx-1, :]))

        print(input_array.shape)
        print(output_array.shape)
        print(observation_array_raw.shape)

        filtered_force = filter_force(observation_array_raw)
        plot_forces(observation_array_raw, "raw")
        plot_forces(filtered_force, "filtered")

        if save:
            np.save(f"{path}/{type_robot_data}_input_array_{number_idx}", input_array)
            np.save(f"{path}/{type_robot_data}_output_array_{number_idx}", output_array)
            np.save(f"{path}/{type_robot_data}_observation_array_{number_idx}", observation_array_raw)

    def cleanShutdown(self):
        print('')
        rospy.loginfo("%s: Sending to safe configuration", self.name)
        # Shut down publishers
        rospy.sleep(1.0)


def filter_force(force):

    # Filter requirements.
    order = 6
    fs = 100.0       # sample rate, Hz
    cutoff = 2.8  # desired cutoff frequency of the filter, Hz

    # Filter the data, and plot both the original and filtered signals.
    x_loc = butter_lowpass_filter(force[:, 0], cutoff, fs, order)
    y_loc = butter_lowpass_filter(force[:, 1], cutoff, fs, order)
    z_loc = butter_lowpass_filter(force[:, 2], cutoff, fs, order)
    x_global = butter_lowpass_filter(force[:, 3], cutoff, fs, order)
    y_global = butter_lowpass_filter(force[:, 4], cutoff, fs, order)
    z_global = butter_lowpass_filter(force[:, 5], cutoff, fs, order)

    local_filtered_force = np.vstack((np.vstack((x_loc, y_loc)), z_loc)).T
    global_filtered_force = np.vstack((np.vstack((x_global, y_global)), z_global)).T

    return np.hstack((local_filtered_force, global_filtered_force))


def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a


def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y


def plot_forces(force_data, file_ending):
    import matplotlib.pyplot as plt
    plt.figure()
    index = np.array([i for i in range(force_data.shape[0])])
    plt.subplot(3, 1, 1)
    plt.plot(index, force_data[:, 0], 'r-', linewidth=2, label='local x')
    plt.plot(index, force_data[:, 3], 'k-', linewidth=2, label='global x')
    plt.subplot(3, 1, 2)
    plt.plot(index, force_data[:, 1], 'g-', linewidth=2, label='local y')
    plt.plot(index, force_data[:, 4], 'k-', linewidth=2, label='global y')
    plt.subplot(3, 1, 3)
    plt.plot(index, force_data[:, 2], 'b-', linewidth=2, label='local z')
    plt.plot(index, force_data[:, 5], 'k-', linewidth=2, label='global z')
    plt.xlabel('Time [sec]')
    plt.grid()
    plt.legend()
    plt.savefig(f"force_{file_ending}.png")


def filter_vel(velocity):

    # Filter requirements.
    order = 6
    fs = 100.0       # sample rate, Hz
    cutoff = 2.5

    dx = butter_lowpass_filter(velocity[:, 0], cutoff, fs, order)
    dy = butter_lowpass_filter(velocity[:, 1], cutoff, fs, order)
    dz = butter_lowpass_filter(velocity[:, 2], cutoff, fs, order)

    filtered_vel = np.vstack((np.vstack((dx, dy)), dz)).T

    return filtered_vel


def plot_ee_vel(vel_data, norm_val_vel, idx1, idx2, file_ending=""):
    import matplotlib.pyplot as plt
    plt.figure()
    index = np.array([i for i in range(vel_data.shape[0])])
    plt.subplot(4, 1, 1)
    plt.plot(index, vel_data[:, 0], 'r-', linewidth=2, label=' x')
    plt.subplot(4, 1, 2)
    plt.plot(index, vel_data[:, 1], 'g-', linewidth=2, label=' y')
    plt.subplot(4, 1, 3)
    plt.plot(index, vel_data[:, 2], 'b-', linewidth=2, label=' z')
    plt.subplot(4, 1, 4)
    plt.plot(index, norm_val_vel[:], 'b-', linewidth=2, label='norm vel')
    plt.xlabel('Time [sec]')
    plt.axvline(x=idx1, color='r', linestyle='-')
    plt.axvline(x=idx2, color='r', linestyle='-')
    plt.grid()
    plt.legend()
    plt.savefig(f"ee_velocity_{file_ending}.png")


def consecutive(data, stepsize=1):
    return np.split(data, np.where(np.diff(data) != stepsize)[0]+1)


if __name__ == '__main__':

    try:
        # Initialize node
        rospy.init_node("parseROSbag", anonymous=True)
        # Initialize node class
        ParseROSbagtoMat = ParseROSbagtoMat()

        rospy.loginfo("%s: Spawn republisher to make human passive", ParseROSbagtoMat.name)

        # Create timer for periodic publisher
        dur = rospy.Duration(50)
        ParseROSbagtoMat.writeCallbackTimer = rospy.Timer(
            dur, ParseROSbagtoMat.parsingState)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
