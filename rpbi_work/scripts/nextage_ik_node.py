#!/usr/bin/env python3
import sys
import rospy
import numpy
import signal
import pyexotica as exo
from sensor_msgs.msg import JointState
from pyexotica.publish_trajectory import sig_int_handler
from ros_pybullet_interface.tf_interface import TfInterface
from rpbi_work.srv import Toggle, ToggleResponse

class Node:

    hz = 100
    # hz = 20
    dt = 1.0/float(hz)

    def __init__(self):

        # Init node
        rospy.init_node('nextage_pushing_node')

        # Setup variables
        self.lgoal = None
        self.rgoal = None
        self.q_prev = None

        # Setup exotica
        # exo.Setup.init_ros()  # we can now instantiate two ik nodes (1 for each arm), no need for exotica-ros since we do our own publishing here
        arm = rospy.get_param('~arm')  # left/right
        if arm == 'left':
            xml_filename = '{rpbi_work}/configs/nextage_ik_left.xml'
        elif arm == 'right':
            xml_filename = '{rpbi_work}/configs/nextage_ik_right.xml'
        else:
            rospy.logerr('Given arm parameter was not recognized (%s), should be "left" or "right"!', arm)
            sys.exit(0)
        rospy.loginfo('loading xml: %s', xml_filename)

        # xml_filename = '{rpbi_work}/configs/nextage_exotica_config.xml'
        self.solver = exo.Setup.load_solver(xml_filename)
        self.problem = self.solver.get_problem()
        self.scene = self.problem.get_scene()
        signal.signal(signal.SIGINT, sig_int_handler)

        # Re-set target to current end-effectors poses
        frameL = self.scene.fk('LEND_EFFECTOR_finger', 'base_link')
        self.scene.attach_object_local('TargetLeft', 'base_link', frameL)
        frameR = self.scene.fk('REND_EFFECTOR_finger', 'base_link')
        self.scene.attach_object_local('TargetRight', 'base_link', frameR)

        # Setup tf interface
        self.tf = TfInterface()

        # Setup joint state publisher
        self.joint_state_pub = rospy.Publisher('rpbi/nextage/joint_state/target', JointState, queue_size=10)

        # Setup services
        self.timer = None
        self.running_ik = False
        rospy.Service('toggle_ik', ToggleIK)

    def toggle_ik(self, req):
        if req.switch == 'on':
            success, info = self.start_ik(req)
        elif req.switch == 'off':
            success, info = self.stop_ik(req)
        else:
            info = 'failed to turn %s IK' % req.switch
            rospy.logerr(info)
            success = False
        return ToggleResponse(success=success, info=info)

    def start_ik(self, req):

        success = True
        info = ''

        if self.running_ik:
            info = "recieved request to start IK, but it is already running!"
            success = False
            rospy.logerr(info)
            return success, info

        # Start main loop
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.main_loop)
        self.running_ik = True
        rospy.loginfo('switched on IK')
        return success, info

    def stop_ik(self, req):

        success = True
        info = ''

        if not self.running_ik:
            info = 'recieved request to stop IK, but it is not running anyway!'
            success = False
            rospy.logerr(info)
            return success, info

        # Stop main loop
        self.timer.shutdown()
        self.timer = None
        self.running_ik = False
        rospy.loginfo('switched off IK')
        return success, info

    def main_loop(self, event):

        # Get goals
        self.lgoal, _ = self.tf.get_tf('rpbi/nextage/nextage_base', 'left_hand_goal')
        self.rgoal, _ = self.tf.get_tf('rpbi/nextage/nextage_base', 'right_hand_goal')

        # Setup problem
        if self.lgoal is not None:
            self.scene.attach_object_local('TargetLeft', 'base_link', self.lgoal)
        if self.rgoal is not None:
            self.scene.attach_object_local('TargetRight', 'base_link', self.rgoal)
        # if self.lgoal is not None:
        #     self.problem.set_goal('LPosition', self.lgoal)
        # if self.rgoal is not None:
        #     self.problem.set_goal('RPosition', self.rgoal)
        # if self.q_prev is not None:
        #     self.problem.start_state = self.q_prev

        if (self.lgoal is not None) or (self.rgoal is not None):

            # Solve problem
            q = self.solver.solve()[0]

            # Publish joint state
            self.publish_joint_state(q)

    def publish_joint_state(self, q_exo):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.position = [q_exo[0], 0, 0]  # chest, head0, head1
        msg.position += q_exo[1:].tolist()
        self.joint_state_pub.publish(msg)

    def spin(self):
        rospy.spin()

def main():
    Node().spin()

if __name__ == '__main__':
    main()
