#!/usr/bin/env python3
import sys
import copy
import rospy
import numpy as np
from std_msgs.msg import Float64
from custom_ros_tools.tf import TfInterface
from geometry_msgs.msg import PoseStamped, Vector3, WrenchStamped
from geomagic_touch_x_ros.msg import OmniFeedback

FF_SCALE = 1

class Node:

    def __init__(self):
        rospy.init_node('run_human_interaction_node')
        touchx_name = rospy.get_param('~device_name', 'Left')
        self.tf = TfInterface()
        self.dev_pos = None
        self.ft_reading_corrected = None
        self.zpos = None
        self.ft_reading = None
        self.zforce_offset = 0.0
        rospy.Subscriber(f'{touchx_name}/Pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('rpbi/kuka_lwr/force_torque_sensor_mount/ft_sensor', WrenchStamped, self.force_feedback_callback)
        self.ff_pub = rospy.Publisher(f'{touchx_name}/force_feedback', OmniFeedback, queue_size=10)
        self.ft_reading_corrected_pub = rospy.Publisher('ft_reading', Float64, queue_size=10)
        self.calibrate_ft_sensor()
        rospy.sleep(5.0)
        rospy.loginfo('demo ready')
        rospy.Timer(rospy.Duration(1.0/float(rospy.get_param('~hz', 400))), self.main_loop)

    def calibrate_ft_sensor(self):
        rospy.sleep(1.0)

        r = rospy.Rate(100)

        maxit = 100
        for i in range(maxit):
            if self.ft_reading is not None:
                break
            r.sleep()
        else:
            rospy.logerr('>>>no ft reading<<<')
            sys.exit(0)

        rospy.loginfo('calibrating ft_sensor....')
        num = 100
        data = np.zeros(num)

        for i in range(num):
            data[i] = copy.deepcopy(self.ft_reading)

        self.zforce_offset = data.mean()

        rospy.loginfo('calibrated')

    def pose_callback(self, msg):
        self.dev_pos = Vector3(
            x=msg.pose.position.x,
            y=msg.pose.position.y,
            z=msg.pose.position.z,
        )
        self.zpos = np.clip(-12*msg.pose.position.z, -10.0, 0.08)

    def force_feedback_callback(self, msg):
        self.ft_reading = msg.wrench.force.z
        self.ft_reading_corrected = self.ft_reading - self.zforce_offset
        self.ft_reading_corrected_pub.publish(Float64(data=self.ft_reading_corrected))

    def main_loop(self, event):
        if self.zpos is None: return

        # Specify ik target
        offsetz = -0.1
        self.tf.set_tf('teleop_origin', 'teleop_target', [0, 0, self.zpos-offsetz])

        if self.dev_pos is None: return
        if self.ft_reading is None: return

        # Get force feedback
        f = -FF_SCALE*self.ft_reading_corrected
        ff = Vector3(x=0, y=f, z=0)

        # Apply force feeback
        self.ff_pub.publish(OmniFeedback(position=self.dev_pos, force=ff))

    def spin(self):
        rospy.spin()

def main():
    Node().spin()

if __name__ == '__main__':
    main()
