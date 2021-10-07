#!/usr/bin/env python3
import tf2_ros
import rospy
from geometry_msgs.msg import TransformStamped

WORLD_FRAME_ID = 'ros_pybullet_interface/world'

HAND_TOPIC = 'hand/pose'
ELBOW_TOPIC = 'elbow/pose'


class RepublishTFasTopic():

    def __init__(self, child_id_read_tf_hand, child_id_pub_hand, child_id_read_tf_elbow, child_id_pub_elbow):

        self.listen_buff = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.listen_buff)

        self.child_ID_tf_hand = child_id_read_tf_hand
        self.child_ID_pub_hand = child_id_pub_hand
        self.hand_pose_pub = rospy.Publisher(HAND_TOPIC, TransformStamped, queue_size=1)

        self.child_ID_tf_elbow = child_id_read_tf_elbow
        self.child_ID_pub_elbow = child_id_pub_elbow
        self.elbow_pose_pub = rospy.Publisher(ELBOW_TOPIC, TransformStamped, queue_size=1)

    def republish_tf(self, event):
        self.read_pub(self.child_ID_tf_hand, self.child_ID_pub_hand, self.hand_pose_pub)
        self.read_pub(self.child_ID_tf_elbow, self.child_ID_pub_elbow, self.elbow_pose_pub)

    def read_pub(self, read_ID, pub_ID, pub):
        read = False
        try:
            trans = self.listen_buff.lookup_transform(
                WORLD_FRAME_ID, read_ID, rospy.Time())

            read = True
        except Exception:
            read = False

        if read:
            # Pack pose msg
            msg = TransformStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = WORLD_FRAME_ID
            msg.child_frame_id = pub_ID
            msg.transform.translation.x = trans.transform.translation.x
            msg.transform.translation.y = trans.transform.translation.y
            msg.transform.translation.z = trans.transform.translation.z
            msg.transform.rotation.x = trans.transform.rotation.x
            msg.transform.rotation.y = trans.transform.rotation.y
            msg.transform.rotation.z = trans.transform.rotation.z
            msg.transform.rotation.w = trans.transform.rotation.w

            pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('tf_extractor')

    child_id_read_tf_hand = 'kolias/RightHand'
    child_id_pub_hand = 'pose/RightHand'

    child_id_read_tf_elbow = 'kolias/RightArm'
    child_id_pub_elbow = 'pose/RightArm'

    RepubTF = RepublishTFasTopic(child_id_read_tf_hand, child_id_pub_hand,
                                 child_id_read_tf_elbow, child_id_pub_elbow)

    # Create timer for periodic publisher
    dur = rospy.Duration(0.01)
    RepubTF.writeCallbackTimer = rospy.Timer(dur, RepubTF.republish_tf)

    rospy.spin()
