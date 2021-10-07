#!/usr/bin/env python3
import tf2_ros
import scipy.io
import rospy
import numpy as np

# ROS message types
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import WrenchStamped

ROBOT_JOINT_STATE_TOPIC = "ros_pybullet_interface/joint_state/target"

WORLD_FRAME_ID = 'ros_pybullet_interface/world'
HUMAN_WRIST_TOPIC = 'kolias/RightHand'
HUMAN_ELBOW_TOPIC = 'kolias/RightArm'

ZONOTOPE_TOPIC = "/ros_pybullet_interface/zono_info"


class ReplayEstimation:

    def __init__(self):

        # Name of node
        self.name = rospy.get_name()
        robot_name = "yin"

        # read from file at the moment
        path2dir_1 = '/home/theo/Documents/data_dressing/processed_data/data_with_zono_2'
        path2file = f"{path2dir_1}/bend/crossval_14.mat"
        data_mat = scipy.io.loadmat(path2file)['data_with_zono_int']

        print("data_mat ",  data_mat.shape)
        # print("self.position_wrist_list ",  self.position_wrist_list.shape)
        # print("self.position_elbow_list ",  self.position_elbow_list.shape)
        # print("self.robot_joint_list ",  self.robot_joint_list.shape)
        # print("self.position_zono_list ",  self.position_zono_list.shape)
        # print("self.scale_zono_list ",  self.scale_zono_list.shape)

        wait_samples = 200
        self.position_wrist_list = list(
            np.vstack((np.tile(data_mat[0, :3], [wait_samples, 1]), data_mat[:, :3])))
        self.position_elbow_list = list(
            np.vstack((np.tile(data_mat[0, 3:6], [wait_samples, 1]), data_mat[:, 3:6])))
        self.robot_joint_list = list(
            np.vstack((np.tile(data_mat[0, 16:23], [wait_samples, 1]), data_mat[:, 16:23])))
        self.position_zono_list = list(
            np.vstack((np.tile(data_mat[0, 23:26], [wait_samples, 1]), data_mat[:, 23:26])))
        self.scale_zono_list = list(
            np.vstack((np.tile(data_mat[0, 26:]*0, [wait_samples, 1]), data_mat[:, 26:])))

        self.tfBroadcaster = tf2_ros.TransformBroadcaster()

        self.target_joint_state_publisher = rospy.Publisher(
            f"{robot_name}/{ROBOT_JOINT_STATE_TOPIC}", JointState, queue_size=1)
        self.zono_state_publisher = rospy.Publisher(
            ZONOTOPE_TOPIC, WrenchStamped, queue_size=1)

        # time.sleep(2.0) # wait for initialisation to complete

    def publishWristElbow(self, event):

        #
        position_wrist = self.position_wrist_list.pop(0)
        self.publishTF(HUMAN_WRIST_TOPIC, position_wrist)

        #
        position_elbow = self.position_elbow_list.pop(0)
        self.publishTF(HUMAN_ELBOW_TOPIC, position_elbow)

    def publishTF(self, child_id, translation):

        pub_msg = TransformStamped()
        pub_msg.header.stamp = rospy.Time.now()
        pub_msg.header.frame_id = WORLD_FRAME_ID
        pub_msg.child_frame_id = child_id
        pub_msg.transform.translation.x = translation[0]
        pub_msg.transform.translation.y = translation[1]
        pub_msg.transform.translation.z = translation[2]
        pub_msg.transform.rotation.x = 0.
        pub_msg.transform.rotation.y = 0.
        pub_msg.transform.rotation.z = 0.
        pub_msg.transform.rotation.w = 1.

        self.tfBroadcaster.sendTransform(pub_msg)

    def publishdRobotJointState(self, event):

        if len(self.robot_joint_list) > 0:
            msg = JointState(
                name=["J0", "J1", "J2", "J3", "J4", "J5", "J6"],
                position=self.robot_joint_list.pop(0),
            )
            msg.header.stamp = rospy.Time.now()
            self.target_joint_state_publisher.publish(msg)
        else:
            print("Robot Data parsed  --------------------")

    def publishZono(self, event):

        if len(self.position_zono_list) > 0 and len(self.scale_zono_list) > 0:
            position_zono = self.position_zono_list.pop(0)
            scale_zono = self.scale_zono_list.pop(0)

            pub_msg = WrenchStamped()
            pub_msg.header.stamp = rospy.Time.now()
            pub_msg.wrench.force.x = position_zono[0]
            pub_msg.wrench.force.y = position_zono[1]
            pub_msg.wrench.force.z = position_zono[2]
            pub_msg.wrench.torque.x = scale_zono[0]
            pub_msg.wrench.torque.y = scale_zono[1]
            pub_msg.wrench.torque.z = scale_zono[2]
            pub_msg.header.stamp = rospy.Time.now()
            self.zono_state_publisher.publish(pub_msg)
        else:
            print("Zono Data parsed  --------------------")


if __name__ == '__main__':
    rospy.init_node('replay_estimation_interface', anonymous=True)
    freq = 25
    ReplayEstimation = ReplayEstimation()
    rospy.loginfo("%s: node started.", ReplayEstimation.name)

    ReplayEstimation.writeCallbackTimerRobot = rospy.Timer(
        rospy.Duration(1.0/float(freq)), ReplayEstimation.publishdRobotJointState)
    ReplayEstimation.writeCallbackTimerZono = rospy.Timer(
        rospy.Duration(1.0/float(freq)), ReplayEstimation.publishZono)
    ReplayEstimation.writeCallbackTimerHuman = rospy.Timer(
        rospy.Duration(1.0/float(freq)), ReplayEstimation.publishWristElbow)

    rospy.spin()
