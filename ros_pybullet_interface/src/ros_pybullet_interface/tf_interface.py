import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped


class TfInterface:

    """Simple singleton interface to tf2. Simply gets/sets transforms."""

    def __init__(self):
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)

    def set_tf(self, base_frame_id, child_frame_id, position, orientation=[0, 0, 0, 1]):
        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = base_frame_id
        msg.child_frame_id = child_frame_id
        msg.transform.translation.x = position[0]
        msg.transform.translation.y = position[1]
        msg.transform.translation.z = position[2]
        msg.transform.rotation.x = orientation[0]
        msg.transform.rotation.y = orientation[1]
        msg.transform.rotation.z = orientation[2]
        msg.transform.rotation.w = orientation[3]
        self.tf_broadcaster.sendTransform(msg)

    def get_tf(self, base_frame_id, child_frame_id):
        try:
            msg = self.tf_buffer.lookup_transform(base_frame_id, child_frame_id, rospy.Time())
            position = [getattr(msg.transform.translation, d) for d in 'xyz']
            orientation = [getattr(msg.transform.rotation, d) for d in 'xyzw']
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn('Did not recieve frame %s in %s!', child_frame_id, base_frame_id)
            position = None
            orientation = None
        return position, orientation
