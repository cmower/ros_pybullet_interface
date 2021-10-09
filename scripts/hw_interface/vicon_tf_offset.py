#!/usr/bin/env python3
# license removed for brevity
import rospkg
import rospy
import os
import numpy as np
from scipy.spatial.transform import Rotation as R


import tf2_ros
from geometry_msgs.msg import TransformStamped

# WORLD_FRAME = '/vicon/world'
WORLD_FRAME = '/ros_pybullet_interface/world'


class ViconTFOffset(object):
    """docstring for ."""

    def __init__(self):

        # check if the name of the robot is provided
        if rospy.has_param('~object_name'):
            object_name = rospy.get_param('~object_name')
        else:
            rospy.logerr(f"The name of the robot is not set in {rospy.get_name()}")
            sys.exit(0)

        if rospy.has_param('~offset'):
            str_offset = rospy.get_param('~offset')
            self.offset = np.array([float(i) for i in str_offset.split(" ")])
        else:
            rospy.logerr(f"The offset is not set in {rospy.get_name()}")
            sys.exit(0)

        # set name of the node
        node_name = "ViconRepublishOffset"
        self.name = f"{object_name}_{node_name}"

        # to publish in the tf
        self.tfBroadcaster = tf2_ros.TransformBroadcaster()

        # Setup ros publisher to update simulated robots
        self.vicon_pose_offset_publishers_topic_name = f"vicon_offset/{object_name}/{object_name}"
        self.vicon_pose_offset_publisher = rospy.Publisher(self.vicon_pose_offset_publishers_topic_name, TransformStamped, queue_size=1)

        # Setup subscriber that reads commanded robot state
        subscr_real_state_topic_name =  f"vicon/{object_name}/{object_name}"
        rospy.Subscriber(subscr_real_state_topic_name, TransformStamped, self.republishPose)


    def republishPose(self, msg):

        # ----------------------------------------------------------------------
        # read pose from vicon
        # ----------------------------------------------------------------------
        obj_transform = msg.transform

        # ----------------------------------------------------------------------
        # update pose of the object with offset
        # ----------------------------------------------------------------------
        msg_transform = TransformStamped()
        msg_transform.child_frame_id = self.vicon_pose_offset_publishers_topic_name

        # apply rot offset
        rot_offset = R.from_euler('ZYX', self.offset[3:6], degrees=True)
        init_rot = (R.from_quat(np.array([obj_transform.rotation.x,\
                                          obj_transform.rotation.y,\
                                          obj_transform.rotation.z,\
                                          obj_transform.rotation.w])))


        new_rot = init_rot*rot_offset
        new_quat = new_rot.as_quat()
        msg_transform.transform.rotation.x = new_quat[0]
        msg_transform.transform.rotation.y = new_quat[1]
        msg_transform.transform.rotation.z = new_quat[2]
        msg_transform.transform.rotation.w = new_quat[3]

        self.local_offset = init_rot.as_matrix().dot(self.offset[0:3])
        # apply translation offset
        msg_transform.transform.translation.x = obj_transform.translation.x + self.local_offset[0]
        msg_transform.transform.translation.y = obj_transform.translation.y + self.local_offset[1]
        msg_transform.transform.translation.z = obj_transform.translation.z + self.local_offset[2]

        msg_transform.header.frame_id = WORLD_FRAME
        msg_transform.header.stamp = rospy.Time.now()
        self.vicon_pose_offset_publisher.publish(msg_transform)

        # Publish msg_transform in tf
        self.tfBroadcaster.sendTransform(msg_transform)



    def cleanShutdown(self):
        print('')
        rospy.loginfo("%s: Sending to safe configuration", self.name)
        # Shut down publishers
        self.vicon_pose_offset_publisher.shutdown()
        rospy.sleep(1.0)


if __name__ == '__main__':

    try:
        # Initialize node
        rospy.init_node("ros_republish_vicon_with_offset", anonymous=True)
        # Initialize node class
        ViconTFOffset = ViconTFOffset()

        rospy.loginfo("%s: Spawn state and command republisher", ViconTFOffset.name)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass