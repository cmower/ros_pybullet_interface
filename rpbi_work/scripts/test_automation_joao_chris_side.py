#!/usr/bin/env python3
import sys
import rospy
from rpbi_work.srv import MoveNextageToPrePushPose, MoveNextageToPrePushPoseRequest
from rpbi_work.srv import Toggle, ToggleRequest
from ros_pybullet_interface.srv import MatchSimToRobot, MatchSimToRobotRequest

class Node:

    def __init__(self):
        rospy.init_node('test_automation_joao_chris_side')

    def _switch_on_remapper(self):
        success = True
        srv = 'toggle_remaper'
        rospy.wait_for_service(srv)
        try:
            handle = rospy.ServiceProxy(srv, Toggle)
            req = ToggleRequest(switch='on')
            resp = handle(req)
        except rospy.ServiceException as e:
            ropsy.logerr('Service call failed: %s' % e)
            success = False
            return success

        if not resp.success:
            success = False
            rospy.logerr('Failed to turn on remapper!')
        return success

    def _switch_off_remapper(self):
        success = True
        srv = 'toggle_remaper'
        rospy.wait_for_service(srv)
        try:
            handle = rospy.ServiceProxy(srv, Toggle)
            req = ToggleRequest(switch='off')
            resp = handle(req)
        except rospy.ServiceException as e:
            ropsy.logerr('Service call failed: %s' % e)
            success = False
            return success

        if not resp.success:
            success = False
            rospy.logerr('Failed to turn off remapper!')
        return success

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
                object_frame_id='',  # TODO
                parent_frame_id='',  # TODO
                arm='left',
                Tmax=5.0,
            )
            resp = handle(req)

        except rospy.ServiceException as e:
            ropsy.logerr('Service call failed: %s' % e)
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
        pass

    def move_left_arm_away(self):
        pass

    def send_robot_right_arm_to_pre_pushing_pose(self):
        pass

    def push_box_to_goal_position(self):
        pass

    def move_robot_to_horray_pose(self):
        pass


def main():

    # Setup
    node = Node()
    rospy.loginfo('Ready to start automation test, press enter to start ...')
    input()

    # Run the automation test
    def run(handle):
        if not handle():
            rospy.logerr('TEST FAILED, QUITTING...')
            sys.exit(0)
        input()

    run(node.snap_pybullet_to_robot)
    run(node.send_robot_left_arm_to_pre_pushing_pose)
    run(node.reorient_box_with_left_arm)
    run(node.move_left_arm_away)
    run(node.send_robot_right_arm_to_pre_pushing_pose)
    run(node.push_box_to_goal_position)
    run(node.move_robot_to_horray_pose)

    rospy.loginfo('Successfully completed automation test!')

if __name__ == '__main__':
    main()
