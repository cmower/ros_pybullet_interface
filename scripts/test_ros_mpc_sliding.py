#!/usr/bin/env python3
import sys
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import tf2_ros
import casadi as cs
from ros_pybullet_interface.utils import loadYAMLConfig, ROOT_DIR

import rospy

# ROS message types
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64MultiArray
# ros_pybullet funcs
from ros_pybullet_interface.utils import ROOT_DIR
import make_manual_pybullet_steps

# --- import external library
import sliding_pack

CMD_DOF = 7
GLB_ORI_ROBOT = np.array([[1., 0., 0.],
                          [0., 1., 0.],
                          [0., 0., -1.]])
TABLE_HEIGHT = -0.055
SAFETY_HEIGHT = 0.1
OBJECT_NAME = "ros_pybullet_interface/sliding_box"  # real box
# OBJECT_NAME = "ros_pybullet_interface/visual_sliding_box"  # real box
OBJECT_TARGET_FRAME_ID = "ros_pybullet_interface/visual_sliding_box"  # visual box

ROBOT_NAME = "LWR/ros_pybullet_interface/robot/end_effector_ball"
WORLD_FRAME = "ros_pybullet_interface/world"
END_EFFECTOR_TARGET_FRAME_ID = 'LWR/ros_pybullet_interface/end_effector/target' # listens for end-effector poses on this topic
SHOW_NOM_FLAG = False
RUN_FREQ = 100

class ROSSlidingMPC:

    def __init__(self):

        # Name of node
        self.name = rospy.get_name()

        # start subcriber
        self.mpc_listen_buff = tf2_ros.Buffer()
        _ = tf2_ros.TransformListener(self.mpc_listen_buff)

        self.tfBroadcaster = tf2_ros.TransformBroadcaster()

        # Initialize data stream
        self.trajObjPlan = np.empty(0)
        self.trajRobotPlan = np.empty(0)

        # Nominal trajectory indexing 
        self.idx_nom = 0

        # load object initial position
        obj_file_name = rospy.get_param('~object_config_file_names', [])[0]
        obj_config = loadYAMLConfig(obj_file_name)
        obj_pos0 = obj_config['link_state']['position']

        # Initialize internal variables
        # self._cmd_robot_pose = np.empty(CMD_DOF)
        # self._cmd_obj_pose = np.empty(CMD_DOF)
        # self._obj_pose = np.empty(CMD_DOF)
        # self._robot_pose = np.empty(CMD_DOF)
        self._cmd_robot_pose = None
        self._cmd_obj_pose = None
        self._cmd_visual_obj_pose = None
        self._obj_pose = None
        self._robot_pose = None

        # Set Problem constants
        #  -------------------------------------------------------------------
        a = 0.17 # side dimension of the square slider in meters
        T = 5 # time of the simulation is seconds
        freq = 50 # number of increments per second
        r_pusher = 0.015 # radius of the cylindrical pusher in meter
        miu_p = 0.1  # friction between pusher and slider
        N_MPC = 50 # time horizon for the MPC controller
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
        self.dt = 1.0/freq # sampling time
        N = int(T*freq) # total number of iterations
        self.Nidx = int(N)
        # Nidx = 3
        #  -------------------------------------------------------------------
        # define system dynamics
        #  -------------------------------------------------------------------
        self.sliding_mode = 'sliding_contact'
        self.dyn = sliding_pack.dyn.System_square_slider_quasi_static_ellipsoidal_limit_surface(
                mode=self.sliding_mode,
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
        x0_nom, x1_nom = sliding_pack.traj.generate_traj_line(0.0, 0.5, N, N_MPC)
        x0_nom = x0_nom + obj_pos0[0]
        x1_nom = x1_nom + obj_pos0[1]
        # x0_nom, x1_nom = sliding_pack.traj.generate_traj_line(0.5, 0.3, N, N_MPC)
        # x0_nom, x1_nom = sliding_pack.traj.generate_traj_circle(-np.pi/2, 3*np.pi/2, 0.1, N, N_MPC)
        # x0_nom, x1_nom = sliding_pack.traj.generate_traj_eight(0.2, N, N_MPC)
        #  -------------------------------------------------------------------
        # stack state and derivative of state
        self.X_nom_val, _ = sliding_pack.traj.compute_nomState_from_nomTraj(x0_nom, x1_nom, self.dt)
        #  ------------------------------------------------------------------
        # define optimization problem
        #  -------------------------------------------------------------------
        if self.sliding_mode == 'sliding_contact':
            W_x = cs.diag(cs.SX([1.0, 1.0, 0.01, 0.0]))
            W_u = cs.diag(cs.SX([0., 0., 0., 0.]))
        elif self.sliding_mode == 'sticking_contact':
            W_x = cs.diag(cs.SX([1.0, 1.0, 0.01, 0.]))
            W_u = cs.diag(cs.SX([0., 0.]))
        else:
            rospy.logerr('Wrong specification of sliding mode')
        self.optObj = sliding_pack.nlp.MPC_nlpClass(
                self.dyn, N_MPC, W_x, W_u, self.X_nom_val, dt=self.dt)
        #  -------------------------------------------------------------------

        time.sleep(2.0)  # wait for initialisation to complete

    def publishPose(self, pose, frame_id):
        """ Publish 6D information for the respective rigid body """

        # if the pose plan is not empty
        if pose is not None:

            # Pack pose msg
            msg = TransformStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = WORLD_FRAME
            msg.child_frame_id = frame_id
            msg.transform.translation.x = pose[0]
            msg.transform.translation.y = pose[1]
            msg.transform.translation.z = pose[2]
            msg.transform.rotation.x = pose[3]
            msg.transform.rotation.y = pose[4]
            msg.transform.rotation.z = pose[5]
            msg.transform.rotation.w = pose[6]

            # Publish msg
            self.tfBroadcaster.sendTransform(msg)

    def publishRobotObjectPose(self, event):

        self.publishPose(self._cmd_robot_pose, END_EFFECTOR_TARGET_FRAME_ID)
        self.publishPose(self._cmd_visual_obj_pose, OBJECT_TARGET_FRAME_ID)

    def readTFs(self, event):
        """ Read robot and object pose periodically """

        self._obj_pose = self.readPose(OBJECT_NAME)
        self._robot_pose = self.readPose(ROBOT_NAME)

    def readPose(self, frame_id_string):

        trans = self.mpc_listen_buff.lookup_transform(WORLD_FRAME, frame_id_string, rospy.Time())
        # replaces base_position = config['base_position']
        end_position = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
        # replaces: base_orient_eulerXYZ = config['base_orient_eulerXYZ']
        end_orient_quat = np.array([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
        end_pose = np.hstack((end_position, end_orient_quat))

        return end_pose


    def buildMPC(self):

        # build the problem
        self.optObj.buildProblem(self.solver_name, self.code_gen, self.no_printing)

    def solveMPC(self, event):

        if self._obj_pose is None or self._robot_pose is None:
            return -1

        # TODO: change to sliding motion
        obj_pos_2d_read = self._obj_pose[0:2]
        obj_ori_2d_read = R.from_quat(self._obj_pose[3:]).as_euler('xyz', degrees=False)[2]
        # robot_pos_2d_read = self._robot_pose[0:2]
        # compute relative angle between pusher (robot) and slider (object)
        # psi_prov = self.optObj.dyn.psi(np.array([
        #     obj_pos_2d_read[0],
        #     obj_pos_2d_read[1],
        #     obj_ori_2d_read,
        #     0.]),
        #     robot_pos_2d_read)
        # build initial state for optimizer: TODO: get this from dyn function
        x0 = [obj_pos_2d_read[0], obj_pos_2d_read[1], obj_ori_2d_read, 0.]
        # we can store those as self._robot_pose and self._obj_pose # ---- solve problem ----
        solFlag, x_opt, u_opt, del_opt, f_opt, t_opt = self.optObj.solveProblem(self.idx_nom, x0)
        self.idx_nom += 1
        x_next = x_opt[:, 1]
        print('*******************')
        print(x0)
        print(self.X_nom_val[:, self.idx_nom].T)
        print(x_next)

        # decode solution
        # compute object pose
        if SHOW_NOM_FLAG:
            # TODO: later replace with call of func from dyn class
            obj_pose_2d = np.array(self.X_nom_val[:, self.idx_nom].T)[0]
        else:
            obj_pose_2d = np.array(self.optObj.dyn.s(x_next).elements())
        obj_pos = np.hstack((obj_pose_2d[0:2], TABLE_HEIGHT))
        obj_ori = np.array([0., 0., obj_pose_2d[2]])
        obj_ori = R.from_rotvec(obj_ori)
        obj_ori_quat = obj_ori.as_quat()
        self._cmd_obj_pose = np.hstack((obj_pos, obj_ori_quat))
        # set visula object pose
        # TODO: later replace with call of func from dyn class
        visual_obj_pose_2d = np.array(self.X_nom_val[:, self.idx_nom].T)[0]
        visual_obj_pos = np.hstack((visual_obj_pose_2d[0:2], TABLE_HEIGHT))
        visual_obj_ori = np.array([0., 0., visual_obj_pose_2d[2]])
        visual_obj_ori = R.from_rotvec(visual_obj_ori)
        visual_obj_ori_quat = visual_obj_ori.as_quat()
        self._cmd_visual_obj_pose = np.hstack((visual_obj_pos, visual_obj_ori_quat))
        # compute robot pose
        robot_pos_2d = np.array(self.optObj.dyn.p(x_next).elements())
        robot_pos = np.hstack((robot_pos_2d, TABLE_HEIGHT+SAFETY_HEIGHT))
        robot_ori = R.from_matrix(GLB_ORI_ROBOT)
        robot_ori_quat = robot_ori.as_quat()
        self._cmd_robot_pose = np.hstack((robot_pos, robot_ori_quat))
        if self.idx_nom > self.Nidx:
            rospy.signal_shutdown("End of nominal trajectory")
        else:
            input()

        # service stuff
        make_manual_pybullet_steps.makeStep(1)

        return solFlag


if __name__=='__main__':

    time.sleep(2.)
    # --- setup the ros interface --- #
    rospy.init_node('test_ros_traj_opt_obj3D', anonymous=True)
    rospy.logwarn("ATTENTION: This node will not run without the impact-TO library!")
    # Initialize node class
    ROSSlidingMPC = ROSSlidingMPC()

    rospy.loginfo("%s: node started.", ROSSlidingMPC.name)

    # build the MPC problem
    ROSSlidingMPC.buildMPC()

    # Create timer for periodic subscriber
    dur_pubsub = rospy.Duration(1./RUN_FREQ)
    ROSSlidingMPC.readTFSCallbackTimer = rospy.Timer(dur_pubsub, ROSSlidingMPC.readTFs)

    # Create timer for periodic publisher
    ROSSlidingMPC.writePoseCallbackTimer = rospy.Timer(dur_pubsub, ROSSlidingMPC.publishRobotObjectPose)
    dur = rospy.Duration(ROSSlidingMPC.dt)
    ROSSlidingMPC.solveMPCCallbackTimer = rospy.Timer(dur, ROSSlidingMPC.solveMPC)

    rospy.spin()
