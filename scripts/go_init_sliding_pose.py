#!/usr/bin/env python3
import sys
import math
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import tf2_ros
import rospy
from geometry_msgs.msg import TransformStamped
import make_manual_pybullet_steps
from ros_pybullet_interface.utils import loadYAMLConfig, ROOT_DIR

r = 0.1 # radius
WORLD_FRAME_ID = 'ros_pybullet_interface/world'
END_EFFECTOR_TARGET_FRAME_ID = 'ros_pybullet_interface/end_effector/target' # publish for end-effector poses on this topic

class TestIK:

    def __init__(self):

        # check if the name of the robot is provided
        if rospy.has_param('~robot_name'):
            self.robot_name = rospy.get_param('~robot_name')
        else:
            rospy.logerr(f"The name of the robot is not set in {rospy.get_name()}")
            sys.exit(0)

        if rospy.has_param('~target_name'):
            self.target_name = rospy.get_param('~target_name')
        else:
            rospy.logerr(f"The name of the target is not set in {rospy.get_name()}")
            sys.exit(0)

        dx = 0
        if rospy.has_param('~displacement'):
            dx = rospy.get_param('~displacement')

        self.traj_index = 0

        # make a line along Z
        num_samplesLin = 100

        # load object initial position
        obj_file_name = rospy.get_param('~object_config_file_name', [])[0]
        obj_config = loadYAMLConfig(obj_file_name)
        obj_pos0 = obj_config['link_state']['position']
        obj_ori0 = obj_config['link_state']['orientation_eulerXYZ']

        # load object dynamics
        sliding_dyn_file_name = rospy.get_param('~sliding_param_dyn', [])[0]
        dyn_config = loadYAMLConfig(sliding_dyn_file_name)
        sideLenght = dyn_config['sideLenght']
        pusherRadious = dyn_config['pusherRadious']
        print(obj_pos0)
        print(obj_ori0)
        # print(sideLenght)
        # print(pusherRadious)
        # sys.exit()

        # compute end-effector initial position
        _L = sideLenght/2. + pusherRadious + 0.01
        self.eePos_traj = np.zeros((1, 3))
        self.eePos_traj[0,0] = obj_pos0[0] + np.cos(np.deg2rad(180.+obj_ori0[2]))*_L
        self.eePos_traj[0,1] = obj_pos0[1] + np.sin(np.deg2rad(180.+obj_ori0[2]))*_L
        self.eePos_traj[0,2] = 0.05

        # # compute end-effector initial position
        # self.eePos_traj = np.zeros((1, 3))
        # self.eePos_traj[0,0] = 0.5
        # self.eePos_traj[0,1] = -0.15
        # self.eePos_traj[0,2] = 0.05



        # ------------------Orientation ---------------------------

        eeOri_traj = np.zeros((1, 3))
        eeOri_traj[0,0] = 3.14
        eeOri_traj[0,1] = 0.
        eeOri_traj[0,2] = 0.

        eeOri_traj_Rot = R.from_rotvec(eeOri_traj)
        self.eeOri_traj = eeOri_traj_Rot.as_quat()

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        time.sleep(2.0) # wait for initialisation to complete


    def updateTrajIndex(self):
        self.traj_index += 1

        if self.traj_index == len(self.eePos_traj):
            rospy.loginfo("Test Motion is done")
            self.writeCallbackTimer.shutdown()


    def publishEEtargetState(self, event):
        # Retrieve position and orientation
        position = self.eePos_traj[self.traj_index, :]
        orientation = self.eeOri_traj[self.traj_index, :]

        # Pack pose msg
        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = WORLD_FRAME_ID
        msg.child_frame_id = f'{self.robot_name}/{END_EFFECTOR_TARGET_FRAME_ID}'
        msg.transform.translation.x = position[0]
        msg.transform.translation.y = position[1]
        msg.transform.translation.z = position[2]
        msg.transform.rotation.x = orientation[0]
        msg.transform.rotation.y = orientation[1]
        msg.transform.rotation.z = orientation[2]
        msg.transform.rotation.w = orientation[3] # NOTE: the ordering here may be wrong

        # Publish msg
        self.tf_broadcaster.sendTransform(msg)

        msg.header.stamp = rospy.Time.now()
        msg.child_frame_id = f'ros_pybullet_interface/{self.target_name}'
        # Publish msg
        self.tf_broadcaster.sendTransform(msg)

        # self.updateTrajIndex()

        # service stuff
        make_manual_pybullet_steps.makeStep(1)



if __name__=='__main__':
    rospy.init_node('test_ros_rbdlIK_interface', anonymous=True)
    freq = 100
    testIK = TestIK()

    testIK.writeCallbackTimer = rospy.Timer(rospy.Duration(1.0/float(freq)), testIK.publishEEtargetState)
    rospy.spin()
