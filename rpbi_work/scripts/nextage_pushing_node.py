#!/usr/bin/env python3
import signal
import numpy
import rospy
import pyexotica as exo
from sensor_msgs.msg import Joy, JointState
from pyexotica.publish_trajectory import sig_int_handler
from ros_pybullet_interface.tf_interface import TfInterface

class Node:

    def __init__(self):
        rospy.init_node('nextage_pushing_node')
        exo.Setup.init_ros()
        self.solver = exo.Setup.load_solver('{ros_pybullet_interface_examples}/configs/nextage_exotica_config.xml')
        self.problem = self.solver.get_problem()
        signal.signal(signal.SIGINT, sig_int_handler)
        self.hz = 50
        self.dt = 1.0/float(self.hz)
        self.pub = rospy.Publisher('rpbi/nextage/joint_state/target', JointState, queue_size=10)
        self.maxvel = 0.04
        self.h = numpy.zeros(2)
        self.tf = TfInterface()
        # self.goal = numpy.array([0.1, 0.1, 0.0, 0, 0, 0])
        self.goal = numpy.array([0.1, 0.1, 0.0])
        rospy.Subscriber('joy', Joy, self.joy_callback)
        rospy.Timer(rospy.Duration(self.dt), self.update)

    def joy_callback(self, msg):
        h = numpy.array([msg.axes[0], msg.axes[1]])
        # hnorm = numpy.linalg.norm(h)
        # try:
        #     h *= min(1.0, hnorm)*h/hnorm
        # except ZeroDivisionError:
        #     pass
        self.h = h

    def update(self, event):
        vel = self.h*self.dt*self.maxvel
        self.goal[0] += vel[0]
        self.goal[1] += vel[1]

        print(self.goal)
        self.problem.set_goal('LPosition', self.goal)
        q = self.solver.solve()[0]
        self.publish_joint_state(q)

    def publish_joint_state(self, q_exo):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.position = [0, 0, 0]  # chest, head0, head1
        msg.position += q_exo.tolist()
        self.pub.publish(msg)

    def spin(self):
        rospy.spin()

def main():
    Node().spin()

if __name__ == '__main__':
    main()
