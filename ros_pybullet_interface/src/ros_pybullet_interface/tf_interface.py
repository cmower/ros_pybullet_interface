import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped


class TfInterface:

    """Simple singleton interface to tf2. Simply gets/sets transforms."""

    def __init__(self):
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)

    def set_tf(self, parent_frame_id, child_frame_id, position, orientation=[0, 0, 0, 1]):
        """Set the position and orientation of a child frame with respect to a parent frame.

Syntax
------

    tf_interface.set_tf(parent_frame_id, child_frame_id, position, orientation=[0, 0, 0, 1])

Parameters
----------

    parent_frame_id (string)
        The parent frame ID.

    child_frame_id (string)
        The child frame ID.

    position (list[float])
        The 3D position of the child frame with respect to the parent
        frame.

    orientation (list[float], optional)
        The orientation, as a quaternion (xyzw), of the child frame
        with respect to the parent frame. If not specified, then [0,
        0, 0, 1] is used.
"""
        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = parent_frame_id
        msg.child_frame_id = child_frame_id
        msg.transform.translation.x = position[0]
        msg.transform.translation.y = position[1]
        msg.transform.translation.z = position[2]
        msg.transform.rotation.x = orientation[0]
        msg.transform.rotation.y = orientation[1]
        msg.transform.rotation.z = orientation[2]
        msg.transform.rotation.w = orientation[3]
        self.tf_broadcaster.sendTransform(msg)

    def get_tf(self, parent_frame_id, child_frame_id):
        """Return position and orientation of child frame with respect to a parent frame.

Syntax
------

    position, orientation = tf_interface.get_tf(parent_frame_id, child_frame_id)

Parameters
----------

    parent_frame_id (string)
        The parent frame ID, must exist in the tf buffer.

    child_frame_id (string)
        The child frame ID, must exist in the tf buffer.

Returns
-------

    position (list[float] or None)
        The 3D position of the child frame with respect to the parent
        frame. If an error occured in the retrieval (e.g. frame
        doesn't exist) then None is returned in its place.

    orientation (list[float] or None)
        The orientation, as a quaternion (xyzw), of the child frame
        with respect to the parent frame. If an error occured in the
        retrieval (e.g. frame doesn't exist) then None is returned in
        its place.
"""
        try:
            msg = self.tf_buffer.lookup_transform(parent_frame_id, child_frame_id, rospy.Time())
            position = [getattr(msg.transform.translation, d) for d in 'xyz']
            orientation = [getattr(msg.transform.rotation, d) for d in 'xyzw']
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logdebug('Did not recieve frame %s in %s!', child_frame_id, parent_frame_id)
            position = None
            orientation = None
        return position, orientation
