#!/usr/bin/env python3
import rospy
import numpy as np
from custom_ros_tools.tf import TfInterface
from ros_pybullet_interface.msg import ResetDebugVisualizerCamera

deg = 180.0/np.pi

class VisualizerCamera:

    def __init__(self):
        self._pub = rospy.Publisher(
            'rpbi/reset_debug_visualizer_camera',
            ResetDebugVisualizerCamera,
            queue_size=10,
        )

    def set_pose(self, T):

        # Convert transform to camera distance, yaw, pitch, and target
        p = T[:3, 3]
        target = p + T[:3, 1]
        disp = target - p
        dist = np.linalg.norm(disp)
        yaw = np.arctan2(-disp[0], disp[1]) * deg
        pitch = np.arctan2(disp[2],np.sqrt(disp[0]*disp[0]+disp[1]*disp[1])) * deg

        # Publish new pose
        self._pub.publish(
            ResetDebugVisualizerCamera(
                cameraDistance=dist,
                cameraYaw=yaw,
                cameraPitch=pitch,
                cameraTargetPosition=target
            )
        )

class TfHandler:

    def __init__(self, parent_tf, child_tf):
        self.parent_tf = parent_tf
        self.child_tf = child_tf
        self._tf = TfInterface()
        self._msg = None

    def reset(self):
        self._msg = self._tf.get_tf_msg(self.parent_tf, self.child_tf)
        return self._msg is not None

    def get_transform(self):
        return self._tf.msg_to_matrix(self._msg)
        
class Node:

    def __init__(self):
        rospy.init_node('control_debug_visualizer_camera_by_tf_node')
        self.tf = TfHandler(
            rospy.get_param('parent_frame_id'),
            rospy.get_param('child_frame_id'),
        )
        self.vis_camera = VisualizerCamera()
        rospy.Timer(rospy.Duration(1.0/float(rospy.get_param('~hz', 50))), self.loop)

    def loop(self, event):
        if self.tf.reset():
            self.vis_camera.set_pose(self.tf.get_transform())

    def spin(self):
        rospy.spin()

def main():
    Node().spin()

if __name__ == '__main__':
    main()
