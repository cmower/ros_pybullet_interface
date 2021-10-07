#!/usr/bin/env python3
import tf2_ros
import rospy
import numpy as np

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import WrenchStamped

WORLD_FRAME_ID = 'ros_pybullet_interface/world'

HAND_TOPIC = 'hand/pose'
ELBOW_TOPIC = 'elbow/pose'

FORCE_TOPIC = 'netft_data'
FILTER_FORCE_TOPIC = 'filtered/netft_data'


class RepublishTFasTopic():

    def __init__(self, child_id_pub_tf_hand, child_id_pub_tf_elbow):

        self.tfBroadcaster = tf2_ros.TransformBroadcaster()

        self.child_id_pub_tf_hand = child_id_pub_tf_hand
        self.child_id_pub_tf_elbow = child_id_pub_tf_elbow

        # Setup subscriber that reads commanded robot state
        subscr_hand_pose_topic_name = HAND_TOPIC
        rospy.Subscriber(subscr_hand_pose_topic_name, TransformStamped, self.republishTFhand)

        # Setup subscriber that reads commanded robot state
        subscr_elbow_pose_topic_name = ELBOW_TOPIC
        rospy.Subscriber(subscr_elbow_pose_topic_name, TransformStamped, self.republishTFelbow)

        # filtered force publisher
        self.force_pub = rospy.Publisher(FILTER_FORCE_TOPIC, WrenchStamped, queue_size=1)

        # Setup subscriber that reads force
        subscr_current_force_state_topic_name = f"{FORCE_TOPIC}"
        rospy.Subscriber(subscr_current_force_state_topic_name,
                         WrenchStamped, self.republishFilteredForce)

        # list to keep the 5 latest forces
        self.force_list_window = []

        self.store_force_raw = []
        self.store_force_filter = []

    def republishFilteredForce(self, msg):

        window_len = 200

        force_raw = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]

        if len(self.force_list_window) > window_len:
            self.force_list_window.pop(0)
        self.force_list_window.append(force_raw)

        force = np.array(self.force_list_window)

        x_filter = [np.mean(force[:, 0])]
        y_filter = [np.mean(force[:, 1])]
        z_filter = [np.mean(force[:, 2])]
        self.store_force_raw.append(np.array(force_raw))
        self.store_force_filter.append(np.array([x_filter[-1], y_filter[-1], z_filter[-1]]))

        pub_msg = WrenchStamped()
        pub_msg.header.stamp = rospy.Time.now()
        pub_msg.wrench.force.x = x_filter[-1]
        pub_msg.wrench.force.y = y_filter[-1]
        pub_msg.wrench.force.z = z_filter[-1]
        pub_msg.wrench.torque = msg.wrench.torque
        self.force_pub.publish(pub_msg)

    def republishTFhand(self, msg):

        pub_msg = TransformStamped()
        pub_msg.header.stamp = rospy.Time.now()
        pub_msg.header.frame_id = WORLD_FRAME_ID
        pub_msg.child_frame_id = self.child_id_pub_tf_hand
        pub_msg.transform.translation = msg.transform.translation
        pub_msg.transform.rotation = msg.transform.rotation

        self.tfBroadcaster.sendTransform(pub_msg)

    def republishTFelbow(self, msg):

        pub_msg = TransformStamped()
        pub_msg.header.stamp = rospy.Time.now()
        pub_msg.header.frame_id = WORLD_FRAME_ID
        pub_msg.child_frame_id = self.child_id_pub_tf_elbow
        pub_msg.transform.translation = msg.transform.translation
        pub_msg.transform.rotation = msg.transform.rotation

        self.tfBroadcaster.sendTransform(pub_msg)

    def cleanShutdown(self):
        print('------- SHUTDOWN --------------')
        all_force = np.hstack((np.array(self.store_force_raw), np.array(self.store_force_filter)))
        plot_forces(all_force)


def plot_forces(force_data, file_ending=""):
    import matplotlib.pyplot as plt
    plt.figure()
    index = np.array([i for i in range(force_data.shape[0])])/2.0
    plt.subplot(3, 1, 1)
    plt.plot(index, force_data[:, 0], 'r--', linewidth=1, label='raw x')
    plt.plot(index, force_data[:, 3], 'k-', linewidth=2, label='filter x')
    plt.subplot(3, 1, 2)
    plt.plot(index, force_data[:, 1], 'g--', linewidth=1, label='raw y')
    plt.plot(index, force_data[:, 4], 'k-', linewidth=2, label='filter y')
    plt.subplot(3, 1, 3)
    plt.plot(index, force_data[:, 2], 'b--', linewidth=1, label='raw z')
    plt.plot(index, force_data[:, 5], 'k-', linewidth=2, label='filter z')
    plt.xlabel('Time [msec]')
    plt.grid()
    plt.legend()
    plt.show()
    # plt.savefig(f"force_{file_ending}.png")


if __name__ == '__main__':
    rospy.init_node('tf_augmentation')

    child_id_pub_tf_hand = 'kolias/RightHand'
    child_id_pub_tf_elbow = 'kolias/RightArm'

    pubTF = RepublishTFasTopic(child_id_pub_tf_hand, child_id_pub_tf_elbow)

    # Ctrl-C will stop the script
    rospy.on_shutdown(pubTF.cleanShutdown)

    rospy.spin()
