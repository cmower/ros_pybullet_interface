#!/usr/bin/env python3
import os
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from keyboard.msg import Key
import tf_conversions

from ros_pybullet_interface.utils import loadYAMLConfig, ROOT_DIR

FREQ = 20
DT = 1.0 / float(FREQ)

WORLD_FRAME_ID = 'ros_pybullet_interface/world'
EFF_TARGET_FRAME_ID = 'ros_pybullet_interface/end_effector/target'
eff_id = loadYAMLConfig(os.path.join(ROOT_DIR, 'configs', 'kuka_teleop.yaml'))['end_effector']
EFF_CURRENT_FRAME_ID = f'ros_pybullet_interface/robot/{eff_id}'

class TeleopNode:

    MAX_VEL = 0.4

    def __init__(self):
        rospy.init_node('teleop_example_node', anonymous=True)
        rospy.on_shutdown(self.shutdown)

        # check if the name of the robot is provided
        if rospy.has_param('~robot_name'):
            self.robot_name = rospy.get_param('~robot_name')
        else:
            rospy.logerr(f"The name of the robot is not set in {rospy.get_name()}")
            sys.exit(0)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.velocity = {
            'xpos': 0,
            'xneg': 0,
            'ypos': 0,
            'yneg': 0,
            'zpos': 0,
            'zneg': 0
        }
        self.eff_transform = None
        while self.eff_transform is None:
            try:
                tf = self.tf_buffer.lookup_transform(
                    WORLD_FRAME_ID,
                    f"{self.robot_name}/{EFF_CURRENT_FRAME_ID}",
                    rospy.Time()
                )
                self.eff_transform = tf.transform
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                continue

    def setupKeyboardReader(self):
        self.keydown_sub = rospy.Subscriber(
            'keyboard/keydown',
            Key,
            self.readKeyboardDown
        )
        self.keyup_sub = rospy.Subscriber(
            'keyboard/keyup',
            Key,
            self.readKeyboardUp
        )

    def readKeyboardDown(self, msg):
        if msg.code == Key.KEY_UP:
            self.velocity['xpos'] = 1
            return
        if msg.code == Key.KEY_DOWN:
            self.velocity['xneg'] = 1
            return
        if msg.code == Key.KEY_LEFT:
            self.velocity['ypos'] = 1
            return
        if msg.code == Key.KEY_RIGHT:
            self.velocity['yneg'] = 1
            return
        if msg.code == Key.KEY_f:
            self.velocity['zpos'] = 1
            return
        if msg.code == Key.KEY_b:
            self.velocity['zneg'] = 1
            return

    def readKeyboardUp(self, msg):
        if msg.code == Key.KEY_UP:
            self.velocity['xpos'] = 0
            return
        if msg.code == Key.KEY_DOWN:
            self.velocity['xneg'] = 0
            return
        if msg.code == Key.KEY_LEFT:
            self.velocity['ypos'] = 0
            return
        if msg.code == Key.KEY_RIGHT:
            self.velocity['yneg'] = 0
            return
        if msg.code == Key.KEY_f:
            self.velocity['zpos'] = 0
            return
        if msg.code == Key.KEY_b:
            self.velocity['zneg'] = 0
            return

    def __getVelocity(self):
        return [
            self.MAX_VEL*float(
                self.velocity['xpos'] - self.velocity['xneg']
            ),
            self.MAX_VEL*float(
                self.velocity['ypos'] - self.velocity['yneg']
            ),
            self.MAX_VEL*float(
                self.velocity['zpos'] - self.velocity['zneg']
            ),
        ]

    def setupUpdateTargetEffPositionTimer(self):
        self.update_target_eff_position_timer = rospy.Timer(
            rospy.Duration(DT),
            self.updateTargetEffPosition
        )

    def updateTargetEffPosition(self, event):
        vx, vy, vz = self.__getVelocity()
        self.eff_transform.translation.x += DT*vx
        self.eff_transform.translation.y += DT*vy
        self.eff_transform.translation.z += DT*vz
        tf = TransformStamped()
        tf.header.stamp = rospy.Time.now()
        tf.header.frame_id = WORLD_FRAME_ID
        tf.child_frame_id = f"{self.robot_name}/{EFF_TARGET_FRAME_ID}"
        tf.transform = self.eff_transform
        self.tf_broadcaster.sendTransform(tf)

    def spin(self):
        try:
            rospy.spin()
        except rospy.ROSException:
            self.shutdown()

    def shutdown(self):
        self.update_target_eff_position_timer.shutdown()
        self.keydown_sub.unregister()
        self.keyup_sub.unregister()


if __name__ == '__main__':
    teleop_node = TeleopNode()
    teleop_node.setupKeyboardReader()
    teleop_node.setupUpdateTargetEffPositionTimer()
    teleop_node.spin()
