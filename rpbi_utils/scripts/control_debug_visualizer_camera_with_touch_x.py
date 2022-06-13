#!/usr/bin/env python3
import rospy
import numpy as np
from ros_pybullet_interface.msg import ResetDebugVisualizerCamera
from custom_ros_tools.tf import TfInterface

class Node:

    def __init__(self):
        rospy.init_node('control_debug_visualizer_camera_with_touch_x_node')
        self.tf = TfInterface()
        self.pub = rospy.Publisher(
            'rpbi/reset_debug_visualizer_camera', ResetDebugVisualizerCamera, queue_size=10
        )
        self.parent = rospy.get_param('~parent_frame_id', 'map')
        self.child = rospy.get_param('~child_frame_id', 'haptic')                
        hz = rospy.get_param('~hz', 100)        
        rospy.Timer(rospy.Duration(1.0/float(hz)), self.loop)

    def get_camera_position_and_target(self):
        tf = self.tf.get_tf_msg(self.parent, self.child)
        if tf is None: return
        position = self.tf.msg_to_pos(tf)
        T = self.tf.msg_to_matrix(tf)
        R = T[:3, :3]
        y = R[:, 1]
        target = position + y
        return position, target

    def loop(self, event):
        data = self.get_camera_position_and_target()
        if data is None: return
        position, target = data
        disp = target - position
        dist = np.linalg.norm(disp)
        yaw = np.arctan2(-disp[0], disp[1]) * 180.0/np.pi
        pitch = np.arctan2(disp[2],np.sqrt(disp[0]**2+disp[1]**2)) * 180.0/np.pi
        self.pub.publish(ResetDebugVisualizerCamera(cameraDistance=dist, cameraYaw=yaw, cameraPitch=pitch, cameraTargetPosition=target))

    def spin(self):
        rospy.spin()

def main():
    Node().spin()

if __name__ == '__main__':
    main()

                    
