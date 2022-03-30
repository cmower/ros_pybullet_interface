#!/usr/bin/env python3
import time
import rospy
import numpy as np
from custom_ros_tools.tf import TfInterface
from keyboard.msg import Key
from std_srvs.srv import SetBool, Trigger
from custom_ros_tools.ros_comm import get_srv_handler
from ros_pybullet_interface.msg import CalculateInverseKinematicsProblem
from ros_pybullet_interface.srv import ResetEffState, ResetEffStateRequest

class TfTracker:

    def __init__(self, tf, parent, child):
        self.tf = tf
        self.parent = parent
        self.child = child
        rospy.Timer(rospy.Duration(0.01), self.main_loop)
        self.transform = None

    def main_loop(self, event):
        self.transform = self.tf.get_tf_msg(self.parent, self.child)

    def distance(self):
        if self.transform is None:
            return np.inf
        return np.linalg.norm(self.tf.msg_to_pos(self.transform))


class Node:

    def __init__(self):
        rospy.init_node('run_teleop_node')
        self.tf = TfInterface()
        self.eff = TfTracker(self.tf, 'teleop_origin', 'rpbi/kuka_lwr/end_effector_ball')
        self.teleop_is_on = False

        # Get service handlers
        self.move_to_eff_state = get_srv_handler(f'rpbi/kuka_lwr/move_to_eff_state', ResetEffState, persistent=True)
        self.start_human_interface = get_srv_handler('operator_node/toggle', SetBool, persistent=True)
        self.toggle_teleop_tf = get_srv_handler('toggle_teleop_tf', SetBool, persistent=True)
        self.reset_zero = get_srv_handler('reset_zero', Trigger, persistent=True)
        self.ik_setup_toggle = get_srv_handler('ik/setup/trac_ik/toggle', SetBool, persistent=True)
        self.ik_solver_toggle = get_srv_handler('ik/solver/trac_ik/toggle', SetBool, persistent=True)

        rospy.Subscriber('keyboard/keydown', Key, self.keyboard_callback)

    def keyboard_callback(self, msg):
        if msg.code == Key.KEY_1:
            self.move_to_initial_pose()
        elif msg.code == Key.KEY_2:
            self.start_teleop()
        elif msg.code == Key.KEY_3:
            self.stop_teleop()

    def move_to_initial_pose(self):

        if self.teleop_is_on:
            rospy.logwarn('teleop is on')
            return

        rospy.loginfo('moving robot to start state ...')

        # Get eff target
        timeout = 10.0
        start_time = time.time()
        while (time.time()-start_time) < timeout:
            init_eff_pos, init_eff_rot = self.tf.get_tf('rpbi/world', 'teleop_origin')
            if init_eff_pos is not None:
                break
        else:
            rospy.logerr("could not recieve end-effector target transform")
            sys.exit(0)

        # Setup problem
        problem = CalculateInverseKinematicsProblem()
        problem.link_name = 'end_effector_ball'
        problem.targetPosition = init_eff_pos
        problem.targetOrientation = init_eff_rot

        # Setup request
        duration = 2.0
        req = ResetEffStateRequest(problem=problem, duration=duration)

        # Move robot
        self.move_to_eff_state(req)

        rospy.loginfo('moved robot to start state')

    def start_teleop(self):

        if self.teleop_is_on:
            rospy.logwarn('teleop is on')
            return

        if self.eff.distance() > 0.03:
            rospy.logwarn('end effector is too far away from start pose, press 1 to move robot to intial pose')
            return

        # Ensure teleop transform is zero
        self.reset_zero()

        rospy.loginfo('started teleoperation')

        self.start_human_interface(True)
        self.toggle_teleop_tf(True)
        self.ik_setup_toggle(True)
        self.ik_solver_toggle(True)

        self.teleop_is_on = True

    def stop_teleop(self):
        self.start_human_interface(False)
        self.toggle_teleop_tf(False)
        self.ik_setup_toggle(False)
        self.ik_solver_toggle(False)
        self.teleop_is_on = False
        rospy.loginfo('stopped teleoperation')

    def spin(self):
        rospy.spin()

def main():
    Node().spin()

if __name__ == '__main__':
    main()
