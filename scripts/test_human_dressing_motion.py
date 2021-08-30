#!/usr/bin/env python3
import sys
import numpy as np
import tf2_ros
import rospy

from scipy.spatial.transform import Rotation as R

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension

# define the WORLD FRAME for the tf topic
WORLD_FRAME_ID = 'ros_pybullet_interface/world'

# listen for the pose of the robot base
ROBOT_BASE_ID = "ros_pybullet_interface/robot/robot_base"

# listens for end-effector poses on this topic
END_EFFECTOR_FRAME_ID = 'ros_pybullet_interface/robot/end_effector_sponge'

# publishes end-effector planned trajectory on this topic
NEW_TRAJ_TOPIC = 'ros_pybullet_interface/end_effector/traj'


class readStateRobotHumanPubPlan():

    def __init__(self, robot_name):

        # Name of node
        self.name = rospy.get_name()

        # init the tf topic parser
        self.state_listen_buff = tf2_ros.Buffer()
        _ = tf2_ros.TransformListener(self.state_listen_buff)

        # initialize empty trajectory plan
        self.trajPlan = np.empty(0)

        # init the publisher
        self.new_traj_publisher = rospy.Publisher(
            f"{robot_name}/{NEW_TRAJ_TOPIC}", Float64MultiArray, queue_size=1)

    def readRobotInitials(self, robot_name):

        rospy.loginfo(f"{self.name}: Reading /tf topic for the pose of the robot")
        # Read the position and orientation of the robot base from the /tf topic
        base_position, base_orient_quat, base_orient_euler = self.readObjectState(
            f"{robot_name}/{ROBOT_BASE_ID}")

        # Read the position and orientation of the robot end effector from the /tf topic
        end_position, end_orient_quat, end_orient_euler = self.readObjectState(
            f"{robot_name}/{END_EFFECTOR_FRAME_ID}")

        return base_position, base_orient_euler, base_orient_quat, end_position, end_orient_euler, end_orient_quat

    def readObjectState(self, child_id):

        while 1:
            try:
                # Read the position and orientation of the robot from the /tf topic
                trans = self.state_listen_buff.lookup_transform(
                    WORLD_FRAME_ID, child_id, rospy.Time())
                break
            except Exception:
                rospy.logwarn(
                    f"{self.name}: /tf topic does NOT have {child_id}")

        position = [trans.transform.translation.x,
                    trans.transform.translation.y,
                    trans.transform.translation.z]
        orient_quat = [trans.transform.rotation.x,
                       trans.transform.rotation.y,
                       trans.transform.rotation.z,
                       trans.transform.rotation.w]
        orient_euler = R.from_quat(orient_quat).as_euler('ZYX')

        return position, orient_quat, orient_euler

    def publishTrajectory(self, event):

        message = self.np2DtoROSmsg(self.trajPlan)

        if message is not None:
            self.new_traj_publisher.publish(message)

    def np2DtoROSmsg(self, data2Darray):

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


def orth_space_vecs(z_vec):
    z = z_vec/np.linalg.norm(z_vec)
    x = np.array([0.866, 0.0, 0.5])   # take a random vector
    x -= x.dot(z) * z               # make it orthogonal to k
    x /= np.linalg.norm(x)          # normalize it
    y = np.cross(z, x)              # cross product with k

    rot_mat = np.vstack((np.vstack((x, y)), z)).T
    return rot_mat


if __name__ == '__main__':

    rospy.sleep(5.0)

    # --- setup the ros interface --- #
    rospy.init_node('test_human_dressing_motion', anonymous=True)

    # check if the name of the robot is provided
    if rospy.has_param('~robot_name'):
        robot_name = rospy.get_param('~robot_name')
    else:
        rospy.logerr(f"The name of the robot is not set in {rospy.get_name()}")
        sys.exit(0)

    freq = 100

    # init wrapper class
    stateReaderPlanPubl = readStateRobotHumanPubPlan(robot_name)

    # get the initial position and orientation of the robot
    _, _, _, endPos, endAtt, endAtt_Quat = stateReaderPlanPubl.readRobotInitials(
        robot_name)

    pelvis_position, pelvis__orient_quat, pelvis_orient_euler = stateReaderPlanPubl.readObjectState(
        "human/ros_pybullet_interface/robot/L5_f1")
    wrist_position, wrist_orient_quat, wrist_orient_euler = stateReaderPlanPubl.readObjectState(
        "human/ros_pybullet_interface/robot/RightHand")
    elbow_position, elbow_orient_quat, elbow_orient_euler = stateReaderPlanPubl.readObjectState(
        "human/ros_pybullet_interface/robot/RightForeArm_f1")
    shoulder_position, shoulder_orient_quat, shoulder_orient_euler = stateReaderPlanPubl.readObjectState(
        "human/ros_pybullet_interface/robot/RightUpperArm_f1")

    # make the plan
    timeSeq = np.array([0.0, 5.0, 8.0, 15.0, 25.0])

    # initial position of the robot
    initpose = np.hstack((endPos, endAtt_Quat))

    # offset from human hand
    z_offset = np.array([0., 0., 0.1])

    wrist_z = -np.asarray(wrist_position) + np.asarray(pelvis_position)
    wrist_mat = orth_space_vecs(wrist_z)
    wrist_orient = R.from_matrix(wrist_mat).as_quat()
    wristpose = np.hstack((wrist_position + z_offset, wrist_orient))

    elbow_z = -np.asarray(elbow_position) + np.asarray(pelvis_position)
    elbow_mat = orth_space_vecs(elbow_z)
    elbow_orient = R.from_matrix(elbow_mat).as_quat()
    elbowpose = np.hstack((elbow_position + z_offset, elbow_orient))

    shoulder_z = -np.asarray(shoulder_position) + np.asarray(pelvis_position)
    shoulder_mat = orth_space_vecs(shoulder_z)
    shoulder_orient = R.from_matrix(shoulder_mat).as_quat()
    shoulderpose = np.hstack((shoulder_position + z_offset, shoulder_orient))

    # position waypoints
    posBody = np.vstack((initpose, wristpose))
    posBody = np.vstack((posBody, wristpose))
    posBody = np.vstack((posBody, elbowpose))
    posBody = np.vstack((posBody, shoulderpose)).T

    # velocity waypoints
    diff = np.diff(posBody, axis=1)
    av_vel = np.sum(diff[:, -2:], axis=1)/2.
    velBody = posBody*0.0
    scale = 0.1
    velBody[:, -2] = scale*av_vel

    trajPlan = np.vstack((np.vstack((timeSeq, posBody)), velBody))
    stateReaderPlanPubl.trajPlan = trajPlan

    input()
    stateReaderPlanPubl.writeCallbackTimer = rospy.Timer(
        rospy.Duration(1.0/float(freq)), stateReaderPlanPubl.publishTrajectory)

    rospy.loginfo("Plan was published!")

    rospy.spin()
