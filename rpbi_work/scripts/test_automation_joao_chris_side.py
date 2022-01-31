#!/usr/bin/env python3
import sys
import rospy
import numpy as np
from std_msgs.msg import Int8
from rpbi_work.srv import MoveNextageToPrePushPose, MoveNextageToPrePushPoseRequest
from rpbi_work.srv import Toggle, ToggleRequest
from ros_pybullet_interface.srv import MatchSimToRobot, MatchSimToRobotRequest
from rpbi_work.srv import MoveNextageToState, MoveNextageToStateRequest
from rpbi_work.srv import SolveIK, SolveIKRequest
from rpbi_work.srv import PusherSlider, PusherSliderRequest

controller_joints_torso = ['CHEST_JOINT0']
controller_joints_head = ['HEAD_JOINT0', 'HEAD_JOINT1']
controller_joints_arm_left = ['LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5']
controller_joints_arm_right = ['RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5']
controller_joints = controller_joints_arm_left + controller_joints_arm_right + controller_joints_torso + controller_joints_head


class Node:

    USE_REAL_ROBOT = True

    def __init__(self):
        rospy.init_node('test_automation_joao_chris_side')

    def _toggle_ik(self, arm, switch):
        rospy.loginfo("going to switch IK %s for %s arm", switch, arm)
        success = True
        srv ='toggle_ik_%s' % arm
        rospy.wait_for_service(srv)
        try:
            req = ToggleRequest(switch=switch)
            handle = rospy.ServiceProxy(srv, Toggle)
            resp = handle(req)
            success = resp.success
        except Exception as e:
            rospy.logwarn("Failed to toggle IK: %s", e)
            success = False
        if success:
            rospy.loginfo("IK switched on")
        return success

    def _move_to_state(self, q, Tmax):
        success = True


        # turn on remapper
        if not self._switch_on_remapper():
            success = False
            return success

        srv = 'move_nextage_to_state'
        rospy.wait_for_service(srv)
        try:
            req = MoveNextageToStateRequest(goal_position=q, Tmax=Tmax)
            handle = rospy.ServiceProxy(srv, MoveNextageToState)
            resp = handle(req)
            success = resp.success
        except:
            success = False

        # turn off remapper
        if not self._switch_off_remapper():
            success = False
            return success

        return success

    def move_to_desk(self):
        q = np.array([0.0, -0.43, -0.31, 0.0, 0.75, -1.57, 0.0, -0.43, -0.31, 0.0, 0.75, 1.57, 0.0, 0.0, 0.2])
        Tmax = 5.0
        return self._move_to_state(q, Tmax)

    def move_to_conveyor(self):
        q = np.array([0.0, -0.43, -0.31, 0.0, 0.75, -1.57, 0.0, -0.43, -0.31, 0.0, 0.75, 1.57, 1.57, 0.0, 0.2])
        Tmax = 5.0
        return self._move_to_state(q, Tmax)

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

        rospy.loginfo('Going to switch on remapper')

        if not self.USE_REAL_ROBOT:
            return True

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

        if success:
            rospy.loginfo('remapper switched on')
        return success

    def _switch_off_remapper(self):

        rospy.loginfo('Going to switch off remapper')

        if not self.USE_REAL_ROBOT:
            return True

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

        if success:
            rospy.loginfo('remapper switched off')
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

    def _send_robot_arm_to_pre_pushing_pose(self, arm):

        success = True
        rospy.loginfo('Sending robot %s arm to pre-pushing pose', arm)

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
                arm=arm,
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

    def send_robot_left_arm_to_pre_pushing_pose(self):
        return self._send_robot_arm_to_pre_pushing_pose('left')

    def send_robot_right_arm_to_pre_pushing_pose(self):
        return self._send_robot_arm_to_pre_pushing_pose('right')

    def reorient_box_with_left_arm(self):
        if not self._plan_mpc('left'):
            return False
        if not self._exec_mpc('left'):
            return False
        return True

    def _plan_mpc(self, arm):

        success = True
        rospy.loginfo('Planning sliding for robot %s arm', arm)

        # Plan sliding trajectory
        srv = 'planning_sliding_%s' % arm
        rospy.wait_for_service(srv)
        try:
            handle = rospy.ServiceProxy(srv, PusherSlider)

            req = PusherSliderRequest(
                info='',
            )
            resp = handle(req)

        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: %s' % e)
            success = False
            return success

        return success

    def _exec_mpc(self, arm):

        success = True

        rospy.loginfo("starting to execute MPC")

        # turn on remapper
        if not self._switch_on_remapper():
            success = False
            return success

        # Turn on IK
        if not self._toggle_ik(arm, 'on'):
            success = False
            return success

        # Start MPC execution
        srv = 'executing_mpc_%s' % arm
        rospy.wait_for_service(srv)
        try:
            handle = rospy.ServiceProxy(srv, PusherSlider)

            req = PusherSliderRequest(
                info='',
            )
            resp = handle(req)
        except rospy.ServiceException as e:
            rospy.logerr("executing MPC failed: %s", e)
            success = False

        if success:
            msg = None
            max_time = 30 # in seconds
            try:
                msg = rospy.wait_for_message('/mpc_completion_flag', Int8, timeout=max_time)
                if msg.data != 0:
                    rospy.logerr('MPC failed with flag: %d', msg.data)
                    success = False
            except rospy.ROSException as e:
                rospy.logerr('MPC timeout exceeded %d seconds: %s', max_time, e)
                success = False

        # Switch off remapper
        if not self._switch_off_remapper():
            success = False

        # Turn on IK
        if not self._toggle_ik(arm, 'off'):
            success = False
            return success

        return success


    def _move_arm_away(self, arm):

        success = True
        rospy.loginfo('Sending robot %s arm away', arm)

        # turn on remapper
        if not self._switch_on_remapper():
            success = False
            return success

        # Move robot to pre push pose
        srv = 'move_nextage_to_pre_push_pose_rev'
        rospy.wait_for_service(srv)
        try:

            handle = rospy.ServiceProxy(srv, MoveNextageToPrePushPose)

            req = MoveNextageToPrePushPoseRequest(
                object_frame_id='pushing_box_visual',
                parent_frame_id='sim/nextage_base',
                arm=arm,
                Tmax=1.0,
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

    def move_left_arm_away(self):
        return self._move_arm_away('left')

    def move_right_arm_away(self):
        return self._move_arm_away('right')

    def push_box_to_goal_position(self):
        if not self._plan_mpc('right'):
            return False
        if not self._exec_mpc('right'):
            return False
        return True

def main():

    # Setup
    node = Node()
    rospy.loginfo('Ready to start automation test, press enter to start ...')
    input()

    # Run the automation test
    def run(handle):
        rospy.loginfo('-'*60)
        rospy.loginfo('Running step: %s', handle.__name__)
        rospy.loginfo('Continue? [ENTER]')
        input()
        success = handle()
        if not success:
            rospy.logerr('TEST FAILED, QUITTING...')
            sys.exit(0)
        rospy.loginfo('Step successfully completed, moving on to next step.')

    run(node.snap_pybullet_to_robot)
    # run(node.send_robot_left_arm_to_pre_pushing_pose)
    # run(node.reorient_box_with_left_arm)
    # run(node.move_left_arm_away)
    run(node.move_to_desk)
    # run(node.move_to_conveyor)
    # run(node.send_robot_right_arm_to_pre_pushing_pose)
    # run(node.push_box_to_goal_position)
    # run(node.move_right_arm_away)

    rospy.loginfo('Successfully completed automation test!')

if __name__ == '__main__':
    main()
