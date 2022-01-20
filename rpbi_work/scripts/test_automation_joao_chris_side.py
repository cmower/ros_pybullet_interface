#!/usr/bin/env python3
import sys
import rospy
import numpy as np
from rpbi_work.srv import MoveNextageToPrePushPose, MoveNextageToPrePushPoseRequest
from rpbi_work.srv import Toggle, ToggleRequest
from ros_pybullet_interface.srv import MatchSimToRobot, MatchSimToRobotRequest
from rpbi_work.srv import MoveNextageToState, MoveNextageToStateRequest
from rpbi_work.srv import SolveIK, SolveIKRequest

controller_joints_torso = ['CHEST_JOINT0']
controller_joints_head = ['HEAD_JOINT0', 'HEAD_JOINT1']
controller_joints_arm_left = ['LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5']
controller_joints_arm_right = ['RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5']
controller_joints = controller_joints_arm_left + controller_joints_arm_right + controller_joints_torso + controller_joints_head


class Node:

    def __init__(self):
        rospy.init_node('test_automation_joao_chris_side')

    def _move_to_state(self, q):
        success = True
        srv = 'move_nextage_to_state'
        rospy.wait_for_service(srv)
        try:
            req = MoveNextageToStateRequest(goal_position=q)
            handle = rospy.ServiceProxy(srv, MoveNextageToState)
            resp = handle(req)
            success = resp.success
        except:
            success = False
        return success


    def _move_arm_to_pos(self, pos, arm):
        success = True
        srv = 'solve_ik_%s' % arm
        rospy.wait_for_service(srv)
        try:
            req_input = {'lgoal': [], 'rgoal': []}
            req_input[arm[0]+'goal'] = pos
            req = SolveIKRequest(**req_input)
            handle = rospy.ServiceProxy(srv, SolveIK)
            resp = handle(req)
            success = resp.success
            q = resp.q
        except:
            success = False

        if not success:
            return success

        success = self._move_to_state(q)

        return success

    def _switch_on_remapper(self):

        # DEBUG
        if True:
            return True
        # DEBUG

        success = True
        srv = 'toggle_remaper'
        rospy.wait_for_service(srv)
        try:
            handle = rospy.ServiceProxy(srv, Toggle)
            req = ToggleRequest(switch='on')
            resp = handle(req)
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)
            success = False
            return success

        if not resp.success:
            success = False
            rospy.logerr('Failed to turn on remapper!')
        return success

    def _switch_off_remapper(self):

        # DEBUG
        if True:
            return True
        # DEBUG


        success = True
        srv = 'toggle_remaper'
        rospy.wait_for_service(srv)
        try:
            handle = rospy.ServiceProxy(srv, Toggle)
            req = ToggleRequest(switch='off')
            resp = handle(req)
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)
            success = False
            return success

        if not resp.success:
            success = False
            rospy.logerr('Failed to turn off remapper!')
        return success

    def _resolve_joint_msg_order(self, msg):
        cmd = []
        for i, name in enumerate(controller_joints):
            joint_index = msg.name.index(name)
            joint_position = msg.position[joint_index]
            cmd.append(joint_position)
        return np.array(cmd)

    def snap_pybullet_to_robot(self):

        success = True

        name = 'nextage'
        topic = '/nextagea/joint_states'
        srv = 'match_sim_to_robot'

        rospy.wait_for_service(srv)
        try:
            srv_handle = rospy.ServiceProxy(srv, MatchSimToRobot)
            req = MatchSimToRobotRequest(robot_name=name, robot_joint_state_topic=topic)
            resp = srv_handle(req)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)
            success = False
            return success

        if resp.success:
            rospy.loginfo("Match successful!")
        else:
            rospy.logerr(">>Match unsuccessful!<<")
            success = False
        return success

    def send_robot_left_arm_to_pre_pushing_pose(self):

        success = True
        rospy.loginfo('Sending robot left arm to pre-pushing pose')

        # turn on remapper
        if not self._switch_on_remapper():
            success = False
            return success

        # Move robot to pre push pose
        srv = 'move_nextage_to_pre_push_pose'
        rospy.wait_for_service(srv)
        try:

            handle = rospy.ServiceProxy(srv, MoveNextageToPrePushPose)

            req = MoveNextageToPrePushPoseRequest(
                object_frame_id='pushing_box_visual',
                parent_frame_id='sim/nextage_base',
                arm='left',
                Tmax=5.0,
            )
            resp = handle(req)

        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)
            success = False
            self._switch_off_remapper()
            return success

        if not resp.success:
            rospy.logerr('Failed to move robot to pre push pose!')
            success = False
            self._switch_off_remapper()
            return success

        if not self._switch_off_remapper():
            success = False

        return success

    def reorient_box_with_left_arm(self):
        pass  # REQUIRES JOAO CODE TO BE CALLABLE VIA SERVICE

    def move_left_arm_away(self):

        success = True

        # turn on remapper
        if not self._switch_on_remapper():
            success = False
            return success

        # Move arm to position
        pos = [0,0,0] # TODO
        success = self._move_arm_to_pos(pos, 'left')

        if not success:
            self._switch_off_remapper()
            return

        # turn off remapper
        if not self._switch_off_remapper():
            success = False
            return success

        return success

    def send_robot_right_arm_to_pre_pushing_pose(self):
        success = True
        rospy.loginfo('Sending robot right arm to pre-pushing pose')

        # turn on remapper
        if not self._switch_on_remapper():
            success = False
            return success

        # Move robot to pre push pose
        srv = 'move_nextage_to_pre_push_pose'
        rospy.wait_for_service(srv)
        try:

            handle = rospy.ServiceProxy(srv, MoveNextageToPrePushPose)

            req = MoveNextageToPrePushPoseRequest(
                object_frame_id='',  # TODO
                parent_frame_id='',  # TODO
                arm='right',
                Tmax=5.0,
            )
            resp = handle(req)

        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)
            success = False
            self._switch_off_remapper()
            return success

        if not resp.success:
            rospy.logerr('Failed to move robot to pre push pose!')
            success = False
            self._switch_off_remapper()
            return success

        if not self._switch_off_remapper():
            success = False

        return success

    def push_box_to_goal_position(self):
        pass # REQUIRES JOAO CODE TO BE CALLABLE VIA SERVICE

    def move_robot_to_horray_pose(self):

        success = True

        # turn on remapper
        if not self._switch_on_remapper():
            success = False
            return success

        # Move to state
        qhorray = np.zeros(len(controller_joints))  # TODO
        self._move_to_state(qhorray)

        # turn on remapper
        if not self._switch_off_remapper():
            success = False
            return success

def main():

    # Setup
    node = Node()
    rospy.loginfo('Ready to start automation test, press enter to start ...')
    input()

    # Run the automation test
    def run(handle):
        success = handle()
        if not success:
            rospy.logerr('TEST FAILED, QUITTING...')
            sys.exit(0)
        input()

    run(node.snap_pybullet_to_robot)
    run(node.send_robot_left_arm_to_pre_pushing_pose)
    # run(node.reorient_box_with_left_arm)
    # run(node.move_left_arm_away)
    # run(node.send_robot_right_arm_to_pre_pushing_pose)
    # run(node.push_box_to_goal_position)
    # run(node.move_robot_to_horray_pose)

    rospy.loginfo('Successfully completed automation test!')

if __name__ == '__main__':
    main()
