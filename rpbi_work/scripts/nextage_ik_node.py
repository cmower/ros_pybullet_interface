#!/usr/bin/env python3
import rospy
import numpy
import signal
import pyexotica as exo
from sensor_msgs.msg import JointState
from pyexotica.publish_trajectory import sig_int_handler
from ros_pybullet_interface.tf_interface import TfInterface

class Node:

    hz = 50
    dt = 1.0/float(hz)

    def __init__(self):

        # Init node
        rospy.init_node('nextage_pushing_node')

        # Setup variables
        self.lgoal = None
        self.rgoal = None
        self.q_prev = None

        # Setup exotica
        exo.Setup.init_ros()
        xml_filename = '{rpbi_work}/configs/nextage_ik.xml'
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

        # Start main loop
        rospy.Timer(rospy.Duration(self.dt), self.main_loop)

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
