#!/usr/bin/env python3
import sys
import rospy
import time
import numpy
from sensor_msgs.msg import JointState

larm_id = [
    'LARM_JOINT0',
    'LARM_JOINT1',
    'LARM_JOINT2',
    'LARM_JOINT3',
    'LARM_JOINT4',
    'LARM_JOINT5',
]

rarm_id = [
    'RARM_JOINT0',
    'RARM_JOINT1',
    'RARM_JOINT2',
    'RARM_JOINT3',
    'RARM_JOINT4',
    'RARM_JOINT5',
]

torso_id = ['CHEST_JOINT0']

head_id = [
    'HEAD_JOINT0',
    'HEAD_JOINT1',
]

nextage_joint_names = larm_id+rarm_id+torso_id+head_id
ndof = len(nextage_joint_names)

class Node:

    hz = 20
    dt = 1.0/float(hz)

    def __init__(self):

        rospy.loginfo('Settingup')

        # Init node
        rospy.init_node('nextage_move_to_home_node')

        # Setup joint state publisher
        self.joint_state_pub = rospy.Publisher('rpbi/nextage/joint_state/target', JointState, queue_size=10)

        # Get current
        self.get_current_joint_state()

        # Compute alpha
        time_max = 10.0  # secs
        self.N = int(round(time_max * self.hz))
        self.alpha = numpy.linspace(0, 1, self.N, dtype=float)

        # Setup qgoal
        self.qgoal = numpy.zeros(ndof)

        # Set rate
        self.rate = rospy.Rate(self.hz)
        time.sleep(1.0)  # leave time for remapper to setup
        rospy.loginfo('initialized move_nextage_to_home_node, moving robot to home position')

    def get_current_joint_state(self):

        # Get current state
        msg = rospy.wait_for_message('/nextagea/joint_states', JointState)
        nmsg = len(msg.position)
        assert nmsg == ndof, f"length of recieved joint state '{nmsg}' is difference to the nextage ndof '{ndof}'!"

        # Pass joint state message to numpy array
        self.qcurr = numpy.zeros(ndof)

        for name in nextage_joint_names:
            idx = msg.name.index(name)
            self.qcurr[idx] = msg.position[idx]

    def main_loop(self):
        for k in range(self.N):
            a = self.alpha[k]
            qtarg = self.qgoal*a + (1-a)*self.qcurr
            self.publish_joint_state(qtarg)
            rospy.loginfo('moving nextage to home state [%.2f\%]', 100.0*a)
            self.rate.sleep()
        rospy.loginfo('nextage reached goal configuration')

    def publish_joint_state(self, q):
        msg = JointState(name=nextage_joint_names, position=q.tolist())
        msg.header.stamp = rospy.Time.now()
        self.joint_state_pub.publish(msg)


def main():
    rval = 0
    try:
        Node().main_loop()
    except:
        rval = 1
    return rval


if __name__ == '__main__':
    sys.exit(
        main()
    )
