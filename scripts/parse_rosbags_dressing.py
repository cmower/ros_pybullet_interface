#!/usr/bin/env python3
# license removed for brevity
import rospy
import numpy as np
import tf2_ros

# ROS message types
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
import message_filters

SIM_HUMAN_JOINT_STATE_TOPIC = 'human/ros_pybullet_interface/joint_state/current'

FORCE_TOPIC = 'yin_human/ros_pybullet_interface/joint_force_torque_sensor/end_effector'

WORLD_FRAME_ID = 'ros_pybullet_interface/world'

SHOULDER_FRAME_ID = "human/ros_pybullet_interface/robot/RightUpperArm_f1"
ELBOW_FRAME_ID = "human/ros_pybullet_interface/robot/RightForeArm_f1"
WRIST_FRAME_ID = "human/ros_pybullet_interface/robot/RightHand"
END_EFFECTOR_FRAME_ID = 'ros_pybullet_interface/end_effector/target'


class ParseROSbagtoMat(object):
    """docstring for ."""

    def __init__(self):

        # set name of the node
        node_name = "ParseROSBAG2MAT"
        self.name = f"{node_name}"

        self.tf_buff = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tf_buff)

        # Setup subscriber that reads current human state
        subscr_current_human_state_topic_name = f"{SIM_HUMAN_JOINT_STATE_TOPIC}"
        sub_joints = message_filters.Subscriber(subscr_current_human_state_topic_name,
                                                JointState)
        # sub_joints.registerCallback(self.readJoints)

        # Setup subscriber that reads force
        subscr_current_force_state_topic_name = f"{FORCE_TOPIC}"
        sub_force = message_filters.Subscriber(subscr_current_force_state_topic_name,
                                               WrenchStamped)
        # sub_force.registerCallback(self.readForce)

        ts = message_filters.ApproximateTimeSynchronizer([sub_joints, sub_force], 1, 1)
        ts.registerCallback(self.readJointsAndForce)

        # init storage lists
        self.human_joints = []
        self.hand = []
        self.elbow = []
        self.shoulder = []
        self.force = []
        self.robot_ee = []

    def readJointsAndForce(self, msgJoints, msgForce):
        joints_array = np.array(msgJoints.position)
        q_human = [joints_array[15],
                   joints_array[17], joints_array[18], joints_array[19]]
        self.human_joints.append(q_human)

        force = [msgForce.wrench.force.x,
                 msgForce.wrench.force.y, msgForce.wrench.force.z]
        self.force.append(force)

        self.robot_ee.append(self.readSpecificTF(f"yin_human/{END_EFFECTOR_FRAME_ID}"))
        self.shoulder.append(self.readSpecificTF(f"{SHOULDER_FRAME_ID}"))
        self.elbow.append(self.readSpecificTF(f"{ELBOW_FRAME_ID}"))
        self.hand.append(self.readSpecificTF(f"{WRIST_FRAME_ID}"))

    def readSpecificTF(self, topic_name):
        while 1:
            try:
                tf = self.tf_buff.lookup_transform(
                    WORLD_FRAME_ID, topic_name, rospy.Time())
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("Lost - something!")

        return np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])

    def readJoints(self, msg):
        joints_array = np.array(msg.position)
        q_human = [joints_array[15], joints_array[17], joints_array[18], joints_array[19]]
        self.human_joints.append(np.array(q_human))

    def readForce(self, msg):
        force = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]
        self.force.append(np.array(force))

    def parsingState(self, event):
        print("------------------------------------")
        joints_array = np.asarray(self.human_joints)
        print(joints_array.shape)
        force_array = np.asarray(self.force)
        print(force_array.shape)
        robot_ee_array = np.asarray(self.robot_ee)
        print(robot_ee_array.shape)
        shoulder_array = np.asarray(self.shoulder)
        print(shoulder_array.shape)
        elbow_array = np.asarray(self.elbow)
        print(elbow_array.shape)
        hand_array = np.asarray(self.hand)
        print(hand_array.shape)
        print(" ")

        path = "/home/theo/Documents/data_dressing/data1"
        input_array = np.hstack((hand_array[:-1, :], elbow_array[:-1, :]))
        input_array = np.hstack((input_array, shoulder_array[:-1, :]))
        input_array = np.hstack((input_array, joints_array[:-1, :]))
        input_array = np.hstack((input_array, robot_ee_array[:-1, :]))
        np.save(f"{path}/input_array", input_array)

        output_array = np.hstack((hand_array[1:, :], elbow_array[1:, :]))
        output_array = np.hstack((output_array, shoulder_array[1:, :]))
        output_array = np.hstack((output_array, joints_array[1:, :]))
        output_array = np.hstack((output_array, robot_ee_array[1:, :]))
        np.save(f"{path}/output_array", output_array)

        observation_array = force_array[:-1, :]
        np.save(f"{path}/observation_array", observation_array)

        print(input_array.shape)
        print(output_array.shape)
        print(observation_array.shape)

    def cleanShutdown(self):
        print('')
        rospy.loginfo("%s: Sending to safe configuration", self.name)
        # Shut down publishers
        rospy.sleep(1.0)


if __name__ == '__main__':

    try:
        # Initialize node
        rospy.init_node("parseROSbag", anonymous=True)
        # Initialize node class
        ParseROSbagtoMat = ParseROSbagtoMat()

        rospy.loginfo("%s: Spawn republisher to make human passive", ParseROSbagtoMat.name)

        # Create timer for periodic publisher
        dur = rospy.Duration(25)
        ParseROSbagtoMat.writeCallbackTimer = rospy.Timer(
            dur, ParseROSbagtoMat.parsingState)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
