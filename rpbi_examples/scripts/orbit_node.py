import rospy
import numpy as np
from custom_ros_tools.tf import TfInterface
from geometry_msgs.msg import TransformStamped, Transform
from custom_ros_tools.ros_comm import ToggleService
from tf_conversions import transformations


def look_at(frame_from, frame_to):
    normalise = lambda x : x / np.linalg.norm(x)

    forward = normalise(frame_to - frame_from)
    right = normalise(np.cross(forward, normalise([0, 0, 1])))
    down = np.cross(forward, right)

    T = np.identity(4)
    T[:3, 0] = right
    T[:3, 1] = down
    T[:3, 2] = forward
    T[:3, 3] = frame_from

    return T


def transform_to_msg(transformation):
    pos = transformations.translation_from_matrix(transformation) # x, y, z
    quat = transformations.quaternion_from_matrix(transformation) # x, y, z, w

    t = Transform()
    t.translation.x, t.translation.y, t.translation.z = pos
    t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w = quat

    return t


class OrbitNode:
    def __init__(self, base_frame, target_frame, centre, distance, height):

        # Setup ros node
        rospy.init_node('orbit_node')

        # Get parameters
        self.duration = rospy.Duration(1.0/rospy.get_param('~hz', 50))
        tf_ns = rospy.get_param('~tf_ns', '')

        self.target = centre
        # self.distance = distance
        self.distance_xy = np.sqrt(distance**2 - height**2)
        self.height = height
        self.speed = 1/8 * np.pi # rad/s

        # Setup transform and broadcaster
        self.tf_interface = TfInterface()
        self.tf = TransformStamped()
        self.tf.header.frame_id = base_frame
        self.tf.child_frame_id = target_frame
        if tf_ns:
            self.tf.header.frame_id = tf_ns + '/' + self.tf.header.frame_id
            self.tf.child_frame_id = tf_ns + '/' + self.tf.child_frame_id

        self.start_time = rospy.Time.now()
        self.timer = rospy.Timer(self.duration, self.main_loop)

    def main_loop(self, event):
        self.tf.header.stamp = rospy.Time.now()

        t = (self.tf.header.stamp - self.start_time).to_sec()

        # source position at "distance" from target point
        a = self.speed * t
        x = np.cos(a) * self.distance_xy
        y = np.sin(a) * self.distance_xy
        source = np.array([x, y, self.height])

        T = look_at(source, self.target)
        self.tf.transform = transform_to_msg(T)

        self.tf_interface.tf_broadcaster.sendTransform(self.tf)

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    OrbitNode("rpbi/world", "rpbi/camera", [0,0,0], 2, 1.5).spin()
