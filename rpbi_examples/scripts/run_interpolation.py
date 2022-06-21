#!/usr/bin/env python3
import sys
import rospy
import numpy as np
from custom_ros_tools.tf import TfInterface
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension


class Node:

    def __init__(self):
        rospy.init_node('run_interpolation_node')
        self.tf = TfInterface()

        # initialize empty trajectory plan
        self.trajPlan = np.empty(0)

        # topic where sequence of waypoints is published
        wpts_topic_name = rospy.get_param(
            '~topic_name_4_waypoints', 'ros_pybullet_interface/waypt_traj')

        # init the publisher
        self.new_traj_publisher = rospy.Publisher(
            f"{wpts_topic_name}", Float64MultiArray, queue_size=1)

    def publish_trajectory(self, event):

        message = self.np2D_to_ROSmsg(self.trajPlan)

        if message is not None:
            self.new_traj_publisher.publish(message)

    def np2D_to_ROSmsg(self, data2Darray):

        # check if there is nothing to publish
        if data2Darray.size == 0:
            return None

        r, c = data2Darray.shape

        # info: http://docs.ros.org/en/api/std_msgs/html/msg/MultiArrayLayout.html
        # Pack trajectory msg
        msg = Float64MultiArray()

        # specify that the array has 2 dimensions
        msg.layout.dim.append(MultiArrayDimension())
        msg.layout.dim.append(MultiArrayDimension())

        # info for reconstruction of the 2D array
        msg.layout.dim[0].label = "rows"
        msg.layout.dim[0].size = r
        msg.layout.dim[1].label = "columns"
        msg.layout.dim[1].size = c

        # add data as flattened numpy array
        msg.data = data2Darray.flatten('C')  # row major flattening

        # time is missing from this message
        # msg.header.stamp = rospy.Time.now()
        return msg

    def exec_plan(self):

        # Generate plan
        rospy.loginfo('generating plan by interpolating between waypoints...')

        # Execute plan
        rospy.loginfo('generating waypoints ...')

        # init_eff_pos = None
        # while (init_eff_pos is None):
        init_eff_pos, init_eff_rot = self.tf.wait_for_tf(
            'rpbi/world', 'rpbi/kuka_lwr/end_effector_ball')

        #
        # Manually create a sequence of waypoints
        #

        # make the time axis of the waypoints
        timeSeq = np.array([0.0, 4.0, 9.0, 14.0])

        # initial position of the robot
        initpose = np.hstack((init_eff_pos, init_eff_rot))

        wpt1_pos = np.array([-0.4, -0.0, 0.1])
        wpt1 = np.hstack((wpt1_pos, init_eff_rot))

        wpt2_pos = np.array([-0.5, 0.2, 0.3])
        wpt2 = np.hstack((wpt2_pos, init_eff_rot))

        wpt3_pos = np.array([-0.15, 0.4, 0.2])
        wpt3 = np.hstack((wpt3_pos, init_eff_rot))

        # generate the position of the waypoints
        posBody = np.vstack((initpose, wpt1))
        posBody = np.vstack((posBody, wpt2))
        posBody = np.vstack((posBody, wpt3)).T

        # generate a rough velocity of the waypoints
        diff = np.diff(posBody, axis=1)
        velBody = posBody*0.0
        scale_vel_factor = 0.5
        velBody[:3, 1:-1] = scale_vel_factor*(diff[:3, :-1] + diff[:3, 1:])/2.0

        # assemble the waypoint plan
        self.trajPlan = np.vstack((np.vstack((timeSeq, posBody)), velBody))

        # publish the waypoint plan
        self.writeCallbackTimer = rospy.Timer(
            rospy.Duration(1.0/float(2)), self.publish_trajectory)

        rospy.loginfo("Waypoints were published!")

    def spin(self):
        rospy.spin()


def main():
    node = Node()
    node.exec_plan()
    node.spin()


if __name__ == '__main__':
    main()
