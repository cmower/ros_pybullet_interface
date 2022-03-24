import rospy
import tf2_ros
import tf_conversions
import numpy as np
from geometry_msgs.msg import TransformStamped


class TfInterface:

    """Simple singleton interface to tf2. Simply gets/sets transforms."""

    def __init__(self):
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)

    def set_tf(self, parent_frame_id, child_frame_id, position, orientation=[0, 0, 0, 1], eul_deg=False):
        """Set the position and orientation of a child frame with respect to a parent frame."""

        # Handle euler angles
        if len(orientation) == 3:
            if eul_deg:
                eul = np.deg2rad(orientation)
            else:
                eul = np.array(orientation)
            quat = tf_conversions.transformations.quaternion_from_euler(eul)
        else:
            quat = np.array(orientation)

        # Pack transform message and broadcast
        self.tf_broadcaster.sendTransform(self.pack_tf_msg(parent_frame_id, child_frame_id, position, orientation))

    @staticmethod
    def pack_tf_msg(parent_frame_id, child_frame_id, position, rotation):
        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = parent_frame_id
        msg.child_frame_id = child_frame_id
        msg.transform.translation.x = position[0]
        msg.transform.translation.y = position[1]
        msg.transform.translation.z = position[2]
        msg.transform.rotation.x = rotation[0]
        msg.transform.rotation.y = rotation[1]
        msg.transform.rotation.z = rotation[2]
        msg.transform.rotation.w = rotation[3]
        return msg

    def get_tf_msg(self, parent_frame_id, child_frame_id):
        try:
            msg = self.tf_buffer.lookup_transform(parent_frame_id, child_frame_id, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logdebug('Did not recieve frame %s in %s!', child_frame_id, parent_frame_id)
            msg = None
        return msg

    def get_tf(self, parent_frame_id, child_frame_id):
        """Return position and orientation of child frame with respect to a parent frame."""
        msg = self.get_tf_msg(parent_frame_id, child_frame_id)
        if msg is not None:
            position = self.msg_to_position(msg)
            orientation = self.msg_to_quaternion(msg)
        else:
            position = None
            orientation = None
        return position, orientation

    @staticmethod
    def msg_to_position(msg):
        return [getattr(msg.transform.translation, d) for d in 'xyz']

    @staticmethod
    def msg_to_quaternion(msg):
        return [getattr(msg.transform.rotation, d) for d in 'xyzw']

    @staticmethod
    def msg_to_matrix(msg):
        p = self.msg_to_position(msg)
        q = self.msg_to_quternion(msg)
        return self.position_and_quaternion_to_matrix(p, q)

    @staticmethod
    def position_and_quaternion_to_matrix(pos, quat):
        T = tf_conversions.transformations.quaternion_matrix(quat)
        T[:3,-1] = pos
        return T
