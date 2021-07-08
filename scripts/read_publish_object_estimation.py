#!/usr/bin/env python3
# license removed for brevity
import rospkg
import rospy
import os

import tf2_ros
import numpy as np

# ROS message types
from geometry_msgs.msg import PoseWithCovarianceStamped


WORLD_FRAME_ID = "ros_pybullet_interface/world"
TARGET_FRAME_ID = "ros_pybullet_interface/target"
FREQ = 100

class ObjectRepublisher():

    def __init__(self):

        # Subscribe target end-effector callback
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        # Setup ros publishers
        publisher_topic_name = f"target/sensor/pose"
        self.target_state_publisher = rospy.Publisher(publisher_topic_name, PoseWithCovarianceStamped, queue_size=1)

        #  info about the node
        self.name = "Object read-state and publish to Kalman"
        self.msg_index = 0


    def read2publish(self, event):

        try:
            tf = self.tfBuffer.lookup_transform(WORLD_FRAME_ID, TARGET_FRAME_ID, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return
        target_EE_position = np.asarray([tf.transform.translation.x, tf.transform.translation.y,tf.transform.translation.z])
        target_EE_orientation = np.asarray([tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w])

        # geometry_msgs/PoseWithCovarianceStamped
        msg = PoseWithCovarianceStamped()
        # header
        msg.header.stamp = rospy.Time.now()
        msg.header.seq = self.msg_index
        self.msg_index += 1
        msg.header.frame_id = "map"
        # pose
        # pose.pose.position
        msg.pose.pose.position.x = target_EE_position[0]
        msg.pose.pose.position.y = target_EE_position[1]
        msg.pose.pose.position.z = target_EE_position[2]
        # pose.pose.orientation
        msg.pose.pose.orientation.x = target_EE_orientation[0]
        msg.pose.pose.orientation.y = target_EE_orientation[1]
        msg.pose.pose.orientation.z = target_EE_orientation[2]
        msg.pose.pose.orientation.w = target_EE_orientation[3]
        # manually tunned covariance
        msg.pose.covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,\
                               0.0, 0.01, 0.0, 0.0, 0.0, 0.0,\
                               0.0, 0.0, 0.01, 0.0, 0.0, 0.0,\
                               0.0, 0.0, 0.0, 0.01, 0.0, 0.0,\
                               0.0, 0.0, 0.0, 0.0, 0.01, 0.0,\
                               0.0, 0.0, 0.0, 0.0, 0.0, 0.01]

        self.target_state_publisher.publish(msg)


if __name__=="__main__":

    try:
        # Initialize node
        rospy.init_node("ros_republish_object_state", anonymous=True)
        # Initialize node class
        ObjectRepublisher = ObjectRepublisher()

        rospy.loginfo("%s: Setup target reader.", ObjectRepublisher.name)

        # Create timer for periodic publisher
        dur = rospy.Duration(1./FREQ)
        ObjectRepublisher.writeCallbackTimer = rospy.Timer(dur, ObjectRepublisher.read2publish)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

 # end
