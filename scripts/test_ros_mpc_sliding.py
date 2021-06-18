#!/usr/bin/env python3
import sys
import os
import math
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

import rospy

# ROS message types
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64MultiArray
# ros_pybullet funcs
from ros_pybullet_interface.utils import ROOT_DIR
import set_object_state_client

# --- import external library
import sliding_pack

NEW_TRAJ_ROBOT_TOPIC = 'ros_pybullet_interface/end_effector/traj' # publishes end-effector planned trajectory on this topic
NEW_TRAJ_OBJ_TOPIC = 'ros_pybullet_interface/object/traj' # publishes end-effector planned trajectory on this topic
CMD_DOF = 7
GLB_ORI_OBJ = np.array([0., 0., 1.])
GLB_ORI_ROBOT = np.array([0., 0., -1.])
TABLE_HEIGHT = 0.
SAFETY_HEIGHT = 0.1
OBJECT_NAME = "ros_pybullet_interface/catch_box"
ROBOT_NAME = "LWR/ros_pybullet_interface/robot/end_effector_ball"

class ROSSlidingMPC:

    def __init__(self):

        # Name of node
        self.name = rospy.get_name()

        # start punlishers
        self.new_Robottraj_publisher = rospy.Publisher(NEW_TRAJ_ROBOT_TOPIC, Float64MultiArray, queue_size=1)
        self.new_Objtraj_publisher = rospy.Publisher(NEW_TRAJ_OBJ_TOPIC, Float64MultiArray, queue_size=1)

        # start subcriber
        # TODO: subscriber to read object position and orientation
        self.mpc_listen_buff = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(self.mpc_listen_buff)

        # Establish connection with planning node
        rospy.loginfo(f"{self.name}: Waiting for self.current_traj_topic topic")
        msgTraj = rospy.wait_for_message(self.current_traj_topic , Float64MultiArray)

        self.tfBroadcaster = tf2_ros.TransformBroadcaster()

        # Initialize data stream
        self.trajObjPlan = np.empty(0)
        self.trajRobotPlan = np.empty(0)

        # Initialize internal variables
        self._cmd_robot_pose = np.empty(CMD_DOF)
        self._cmd_obj_pose = np.empty(CMD_DOF)
        self._obj_pose = np.empty(CMD_DOF)
        self._robot_pose = np.empty(CMD_DOF)

        ## Set Problem constants
        #  -------------------------------------------------------------------
        a = 0.09 # side dimension of the square slider in meters
        T = 12 # time of the simulation is seconds
        freq = 50 # number of increments per second
        r_pusher = 0.01 # radius of the cylindrical pusher in meter
        miu_p = 0.2  # friction between pusher and slider
        N_MPC = 15 # time horizon for the MPC controller
        x_init_val = [-0.01, 0.03, 30*(np.pi/180.), 0]
        f_lim = 0.3 # limit on the actuations
        psi_dot_lim = 3.0 # limit on the actuations
        psi_lim = 40*(np.pi/180.0)
        # solver_name = 'ipopt'
        self.solver_name = 'snopt'
        # solver_name = 'gurobi'
        # solver_name = 'qpoases'
        self.no_printing = True
        self.code_gen = False
        #  -------------------------------------------------------------------
        # Computing Problem constants
        #  -------------------------------------------------------------------
        dt = 1.0/freq # sampling time
        N = int(T*freq) # total number of iterations
        Nidx = int(N)
        # Nidx = 3
        #  -------------------------------------------------------------------
        # define system dynamics
        #  -------------------------------------------------------------------
        self.dyn = sliding_pack.dyn.System_square_slider_quasi_static_ellipsoidal_limit_surface(
                slider_dim=a,
                pusher_radious=r_pusher,
                miu=miu_p,
                f_lim=f_lim,
                psi_dot_lim=psi_dot_lim,
                psi_lim=psi_lim
        )
        #  -------------------------------------------------------------------
        ## Generate Nominal Trajectory
        #  -------------------------------------------------------------------
        # x0_nom, x1_nom = sliding_pack.traj.generate_traj_line(0.5, 0.0, N, N_MPC)
        # x0_nom, x1_nom = sliding_pack.traj.generate_traj_line(0.5, 0.3, N, N_MPC)
        # x0_nom, x1_nom = sliding_pack.traj.generate_traj_circle(-np.pi/2, 3*np.pi/2, 0.1, N, N_MPC)
        x0_nom, x1_nom = sliding_pack.traj.generate_traj_eight(0.2, N, N_MPC)
        #  -------------------------------------------------------------------
        # stack state and derivative of state
        self.X_nom_val, _ = sliding_pack.traj.compute_nomState_from_nomTraj(x0_nom, x1_nom, dt)
        #  ------------------------------------------------------------------
        # define optimization problem
        #  -------------------------------------------------------------------
        self.optObj = sliding_pack.nlp.MPC_nlpClass(
                dyn, N_MPC, X_nom_val, dt=dt)
        #  -------------------------------------------------------------------

        time.sleep(2.0) # wait for initialisation to complete

    def publishPose(self, event):
        """ Publish 6D information for the respective rigid body """

        # pose = self.trajManag.getNextWayPt()
        # TODO: we need to get position and orientation of robot and object

        # if the pose plan is not empty
        if pose is not None:

            # Pack pose msg
            msg = TransformStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.msg_header_frame_id
            msg.child_frame_id = self.msg_child_frame_id
            msg.transform.translation.x = pose[0]
            msg.transform.translation.y = pose[1]
            msg.transform.translation.z = pose[2]
            msg.transform.rotation.x = pose[3]
            msg.transform.rotation.y = pose[4]
            msg.transform.rotation.z = pose[5]
            msg.transform.rotation.w = pose[6] # NOTE: the ordering here may be wrong

            # Publish msg
            self.tfBroadcaster.sendTransform(msg)

    def publishRobotPose(self, event):

        robot_pose = self.np2DtoROSmsg(self.trajRobotPlan) # TODO: replace func for robot

        if robot_pose != None:
            self.publishPose(robot_pose)

    def publishObjectPose(self, event):

        obj_pose = self.np2DtoROSmsg(self.trajRobotPlan) # TODO: replace func for object

        if obj_pose != None:
            self.publishPose(obj_pose)

    def readTFs(self, event):
        """ Read robot and object pose periodically """

        self._obj_pose = readPose(OBJECT_NAME)
        self._robot_pose = readPose(ROBOT_NAME)

    def readPose(self, frame_id_string):

        trans = self.mpc_listen_buff.lookup_transform(WORLD_FRAME_ID, frame_id_string, rospy.Time())
        # replaces base_position = config['base_position']
        end_position = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
        # replaces: base_orient_eulerXYZ = config['base_orient_eulerXYZ']
        end_orient_quat = np.array([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
        end_pose = np.hstack((end_position, end_orient_quat))

        return end_pose


    def buildMPC(self):

        # build the problem
        self.optObj.buildProblem(self.solver_name, self.code_gen, self.no_printing)

    def solveMPC(self):

        obj_pos_2d_read = self._obj_pose[0:2]
        obj_ori_2d_read = np.linalg.norm(R.from_quat(self._obj_pose[3:]).as_rotvec())
        robot_pos_2d_read = self._robot_pose[0:2]
        # compute relative angle between pusher (robot) and slider (object)
        psi_prov = self.optObj.dyn.psi(np.array(
            obj_pos_2d_read[0],
            obj_pos_2d_read[1],
            obj_ori_2d_read,
            0.]),
            robot_pos_2d_read)
        # build initial state for optimizer
        x0 = np.array([
            obj_pos_2d_read[0],
            obj_pos_2d_read[1],
            obj_ori_2d_read,
            psi_prov])
        # we can store those as self._robot_pose and self._obj_pose
        # ---- solve problem ----
        solFlag, x_opt, u_opt, del_opt, f_opt, t_opt = self.optObj.solveProblem(idx, x0)
        x_next = x_opt[:,0]
        robot_pos = x_next[0:3] + np.dot(dyn.R(x_next), np.array([-dyn.sl/2, -dyn.sl/2, 0]))

        # decode solution
        # compute object pose
        obj_pose_2d = self.optObj.dyn.s(x_next)
        obj_pos = np.hstack((obj_pose_2d[0:2], TABLE_HEIGHT))
        obj_ori = R.from_rotvec(GLB_ORI_OBJ * pose_2d[2])
        obj_ori_quat = obj_ori.as_quat()
        self._cmd_obj_pose = np.hstack((obj_pos, obj_ori_quat))
        # compute robot pose
        robot_pos_2d = self.optObj.dyn.p(x_next)
        robot_pos = np.hstack((robot_pos_2d, TABLE_HEIGHT+SAFETY_HEIGHT))
        robot_ori = R.from_rotvec(GLB_ORI_ROBOT)
        robot_ori_quat = robot_ori.as_quat()
        self._cmd_robot_pose = np.hstack((robot_pos, robot_ori_quat))

        return solFlag


if __name__=='__main__':

    # --- setup the ros interface --- #
    rospy.init_node('test_ros_traj_opt_obj3D', anonymous=True)
    rospy.logwarn("ATTENTION: This node will not run without the impact-TO library!")
    # Initialize node class
    ROSSlidingMPC = ROSSlidingMPC()

    freq = 100
    PlanInterpWithTO = PlanInterpWithTO()
    rospy.loginfo("%s: node started.", PlanInterpWithTO.name)

    # build the TO problem
    PlanInterpWithTO.buildTO()
    # solve the TO problem
    solFlag = PlanInterpWithTO.solveTO()

    # Create timer for periodic publisher
    dur = rospy.Duration(ROSSlidingMPC.dt)
    ROSSlidingMPC.writeCallbackTimer = rospy.Timer(dur, ROSSlidingMPC.publishPose)

    # Create timer for periodic subscriber
    dur = rospy.Duration(ROSSlidingMPC.dt)
    ROSSlidingMPC.readTFSCallbackTimer = rospy.Timer(dur, ROSSlidingMPC.readTFs)

    if solFlag == True:
        rospy.loginfo(" TO problem solved!")
        set_object_state_client.main()

    rospy.spin()
