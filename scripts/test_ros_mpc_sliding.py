#!/usr/bin/env python3
import sys
import time
import math
import numpy as np
from scipy.spatial.transform import Rotation as R
import tf2_ros
import casadi as cs
from ros_pybullet_interface.utils import loadYAMLConfig, ROOT_DIR
np.set_printoptions(precision=3)

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
ROBOT_HEIGHT = 0.06
VISUAL_OBJ_HEIGHT = 0.02 + 0.035
OBJECT_TARGET_FRAME_ID = "ros_pybullet_interface/visual_sliding_box"  # visual box
WORLD_FRAME = "ros_pybullet_interface/world"
SHOW_NOM_FLAG = False
RUN_FREQ = 50

REAL_SETUP = False

OBJECT_NAME_SIM = "ros_pybullet_interface/sliding_box"  # real box
ROBOT_NAME_SIM = "LWR/ros_pybullet_interface/robot/end_effector_ball"
END_EFFECTOR_TARGET_FRAME_ID_SIM = 'LWR/ros_pybullet_interface/end_effector/target' # listens for end-effector poses on this topic
OBSTACLE_NAME_SIM = "ros_pybullet_interface/sliding_obs"

OBJECT_NAME_REAL = "vicon_offset/pushing_box_wood/pushing_box_wood"
ROBOT_NAME_REAL = "LWR_visual/ros_pybullet_interface/robot/end_effector_ball"
END_EFFECTOR_TARGET_FRAME_ID_REAL = 'LWR_visual/ros_pybullet_interface/end_effector/target' # listens for end-effector poses on this
OBSTACLE_NAME_REAL = "vicon_offset/pushing_obs_cyl_1/pushing_obs_cyl_1"


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
        
        # Save variable for time
        self.comp_time_plot = []

        self.real_setup=False
        # check if the real_setup param exists
        if rospy.has_param('~real_setup'):
            self.real_setup = rospy.get_param('~real_setup')
        else:
            rospy.logerr(f"The real_setup parameter is not set in {rospy.get_name()}")
            sys.exit(0)

        if self.real_setup:
            self.robot_name = ROBOT_NAME_REAL
            self.end_effector_target_frame_id = END_EFFECTOR_TARGET_FRAME_ID_REAL
            self.object_name = OBJECT_NAME_REAL
            self.obstacle_name = OBSTACLE_NAME_REAL

        else:
            self.robot_name = ROBOT_NAME_SIM
            self.end_effector_target_frame_id = END_EFFECTOR_TARGET_FRAME_ID_SIM
            self.object_name = OBJECT_NAME_SIM
            self.obstacle_name = OBSTACLE_NAME_SIM

        # Loop till the pos and ori of the object has been read.
        while 1:
            try:
                # Read the position and orientation of the robot from the /tf topic
                trans = self.mpc_listen_buff.lookup_transform(WORLD_FRAME, f"{self.object_name}", rospy.Time())
                # replaces base_position = config['base_position']
                obj_pos0 = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]

                break
            except:
                rospy.logwarn(f"{self.name}: /tf topic does NOT have 123 {self.object_name}")

        # Get config files
        #  -------------------------------------------------------------------
        sliding_dyn_file_name = rospy.get_param('~sliding_param_dyn', [])[0]
        dyn_config = loadYAMLConfig(sliding_dyn_file_name)
        tracking_traj_file_name = rospy.get_param('~sliding_param_tracking_traj', [])[0]
        tracking_config = loadYAMLConfig(tracking_traj_file_name)
        #  -------------------------------------------------------------------

        # Initialize internal variables
        self._cmd_robot_pose = None
        self._cmd_obj_pose = None
        self._cmd_visual_obj_pose = None
        self._obj_pose = None
        self._obs_pose = None
        self._robot_pose = None

        # Set Problem constants
        #  -------------------------------------------------------------------
        T = 8  # time of the simulation is seconds
        # T = 10  # time of the simulation is seconds
        freq = 25  # number of increments per second
        N_MPC = 25  # time horizon for the MPC controller
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
        self.dyn = sliding_pack.dyn.Sys_sq_slider_quasi_static_ellip_lim_surf(
                dyn_config, 
                tracking_config['contactMode']
        )

        planning_flag = True
        if planning_flag:
            print('I am planning.')
            while 1:
                try:
                    # Read the position and orientation of the robot from the /tf topic
                    trans = self.mpc_listen_buff.lookup_transform(WORLD_FRAME, f"{self.obstacle_name}", rospy.Time())
                    # replaces base_position = config['base_position']
                    obs_pos0 = [trans.transform.translation.x, trans.transform.translation.y]
                    break
                except:
                    rospy.logwarn(f"{self.name}: /tf topic does NOT have 456 {self.obstacle_name}")
            print('I received info.')
            # nom traj file for planning
            nom_traj_file_name = rospy.get_param('~sliding_param_nom_traj', [])[0]
            nom_config = loadYAMLConfig(nom_traj_file_name)
            X_goal = nom_config['X_goal']
            x0_nom, x1_nom = sliding_pack.traj.generate_traj_line(X_goal[0]-obj_pos0[0], X_goal[1]-obj_pos0[1], N, N_MPC)
            x0_nom = x0_nom + obj_pos0[0]
            x1_nom = x1_nom + obj_pos0[1]
            X_nom_val_temp, _ = sliding_pack.traj.compute_nomState_from_nomTraj(x0_nom, x1_nom, self.dt)
            # Compute nominal actions for sticking contact
            #  ------------------------------------------------------------------
            print('i am going to build')
            optObj = sliding_pack.to.buildOptObj(
                    self.dyn, N+N_MPC, nom_config, X_nom_val_temp, dt=self.dt)
            print('i built')
            # read obstacle position
            if optObj.numObs == 0:
                obsCentre = None
                obsRadius = None
            elif optObj.numObs == 1:
                obsCentre = [obs_pos0]
                obsRadius = [0.065]
            print('about to start planning')
            resultFlag, X_nom_val, U_nom_val, other_opt, _, t_opt = optObj.solveProblem(
                    0, X_nom_val_temp[:, 0].elements(),
                    obsCentre=obsCentre, obsRadius=obsRadius)
            #  ---------------------------------------------------------------
            fig, ax = sliding_pack.plots.plot_nominal_traj(
                        cs.DM(x0_nom), cs.DM(x1_nom))
            # add computed nominal trajectory
            X_nom_val_plot = np.array(X_nom_val)
            ax.plot(X_nom_val_plot[0, :], X_nom_val_plot[1, :], color='blue',
                    linewidth=2.0, linestyle='dashed')
            # add obstacles
            import matplotlib.pyplot as plt
            if optObj.numObs > 0:
                for i in range(len(obsCentre)):
                    circle_i = plt.Circle(obsCentre[i], obsRadius[i], color='b')
                    ax.add_patch(circle_i)
            ax.set_xlim((-0.5, 0.5))
            ax.set_ylim((-1.0, -0.3))
            plt.show()
            self.X_nom_val = X_nom_val
        else:
            #  -------------------------------------------------------------------
            # Generate Nominal Trajectory
            #  -------------------------------------------------------------------
            # x0_nom, x1_nom = sliding_pack.traj.generate_traj_line(0.5, 0., N, N_MPC)
            # x0_nom, x1_nom = sliding_pack.traj.generate_traj_line(0., 0.3, N, N_MPC)
            # x0_nom, x1_nom = sliding_pack.traj.generate_traj_line(0.5, 0.3, N, N_MPC)
            # x0_nom, x1_nom = sliding_pack.traj.generate_traj_eight(0.11, N, N_MPC)
            # x0_nom, x1_nom = sliding_pack.traj.generate_traj_circle(0., -2*np.pi, 0.1, N, N_MPC)
            x0_nom, x1_nom = sliding_pack.traj.generate_traj_ellipse(0., -2*np.pi, 0.3, 0.09, N, N_MPC)
            #  -------------------------------------------------------------------
            x0_nom = x0_nom + obj_pos0[0]
            x1_nom = x1_nom + obj_pos0[1]
            # stack state and derivative of state
            self.X_nom_val, _ = sliding_pack.traj.compute_nomState_from_nomTraj(x0_nom, x1_nom, self.dt)



        #  ------------------------------------------------------------------
        # define optimization problem
        #  -------------------------------------------------------------------
        self.optObj = sliding_pack.to.buildOptObj(
                self.dyn, N_MPC, tracking_config,
                self.X_nom_val, dt=self.dt)
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

        self.publishPose(self._cmd_robot_pose, self.end_effector_target_frame_id)
        self.publishPose(self._cmd_visual_obj_pose, OBJECT_TARGET_FRAME_ID)

    def readTFs(self, event):
        """ Read robot and object pose periodically """

        self._obj_pose = self.readPose(self.object_name)
        self._obs_pose = self.readPose(self.obstacle_name)
        self._robot_pose = self.readPose(self.robot_name)

    def readPose(self, frame_id_string):

        trans = self.mpc_listen_buff.lookup_transform(WORLD_FRAME, frame_id_string, rospy.Time())
        # replaces base_position = config['base_position']
        end_position = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
        # replaces: base_orient_eulerXYZ = config['base_orient_eulerXYZ']
        end_orient_quat = np.array([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
        end_pose = np.hstack((end_position, end_orient_quat))

        return end_pose


    def solveMPC(self, event):

        if self._obj_pose is None or self._robot_pose is None:
            return -1

        # read object position and orientation
        obj_pos_2d_read = self._obj_pose[0:2]
        obj_ori_2d_read = R.from_quat(self._obj_pose[3:]).as_euler('xyz', degrees=False)[2]
        # hack to adjust read orientation around the 180 - -180 angle flipping
        _target_ori = self.X_nom_val[2, self.idx_nom]
        if obj_ori_2d_read < (_target_ori - np.pi/2.):
            obj_ori_2d_read += 2.*np.pi
        elif obj_ori_2d_read > (_target_ori + np.pi/2.):
            obj_ori_2d_read -= 2.*np.pi
        robot_pos_2d_read = self._robot_pose[0:2]
        # compute relative angle between pusher (robot) and slider (object)
        psi0 = self.optObj.dyn.psi(np.array([
            obj_pos_2d_read[0],
            obj_pos_2d_read[1],
            obj_ori_2d_read,
            0.]),
            robot_pos_2d_read).elements()[0]
        # read obstacle position
        if self.optObj.numObs == 0:
            obsCentre = None
            obsRadius = None
        elif self.optObj.numObs == 1:
            obsCentre = [self._obs_pose[0:2]]
            obsRadius = [0.065]
        
        # build initial state for optimizer: TODO: get this from dyn function
        x0 = [obj_pos_2d_read[0], obj_pos_2d_read[1], float(obj_ori_2d_read), psi0]
        # we can store those as self._robot_pose and self._obj_pose # ---- solve problem ----
        solFlag, x_opt, u_opt, del_opt, f_opt, t_opt = self.optObj.solveProblem(self.idx_nom, x0,
                obsCentre=obsCentre, obsRadius=obsRadius)
        # saving computation times
        self.comp_time_plot.append(t_opt)
        self.idx_nom += 1
        x_next = x_opt[:, 1]

        # decode solution
        # compute object pose
        if SHOW_NOM_FLAG:
            # TODO: later replace with call of func from dyn class
            obj_pose_2d = np.array(self.X_nom_val[:, self.idx_nom].T)[0]
        else:
            obj_pose_2d = np.array(self.optObj.dyn.s(x_next).elements())
        obj_pos = np.hstack((obj_pose_2d[0:2], ROBOT_HEIGHT))
        obj_ori = np.array([0., 0., obj_pose_2d[2]])
        obj_ori = R.from_rotvec(obj_ori)
        obj_ori_quat = obj_ori.as_quat()
        self._cmd_obj_pose = np.hstack((obj_pos, obj_ori_quat))
        # set visula object pose
        # TODO: later replace with call of func from dyn class
        visual_obj_pose_2d = np.array(self.X_nom_val[:, self.idx_nom].T)[0]
        visual_obj_pos = np.hstack((visual_obj_pose_2d[0:2], VISUAL_OBJ_HEIGHT))
        visual_obj_ori = np.array([0., 0., visual_obj_pose_2d[2]])
        visual_obj_ori = R.from_rotvec(visual_obj_ori)
        visual_obj_ori_quat = visual_obj_ori.as_quat()
        self._cmd_visual_obj_pose = np.hstack((visual_obj_pos, visual_obj_ori_quat))
        if solFlag:
            # compute robot pose
            robot_pos_2d = np.array(self.optObj.dyn.p(x_next).elements())
            robot_pos = np.hstack((robot_pos_2d, ROBOT_HEIGHT))
            robot_ori = R.from_matrix(GLB_ORI_ROBOT)
            robot_ori_quat = robot_ori.as_quat()
            self._cmd_robot_pose = np.hstack((robot_pos, robot_ori_quat))
        if self.idx_nom > self.Nidx:
            # np.save('/home/kuka-lwr/pybullet_ws/files/comp_time', np.asarray(self.comp_time_plot))
            # np.save('/home/kuka-lwr/pybullet_ws/files/nominal_traj', self.X_nom_val)
            rospy.signal_shutdown("End of nominal trajectory")
        else:
            # input()
            pass

        if not self.real_setup:
            # service stuff
            make_manual_pybullet_steps.makeStep(int(100./RUN_FREQ))

        return solFlag


if __name__=='__main__':

    time.sleep(2.)
    # --- setup the ros interface --- #
    rospy.init_node('test_ros_traj_opt_obj3D', anonymous=True)
    rospy.logwarn("ATTENTION: This node will not run without the impact-TO library!")
    # Initialize node class
    ROSSlidingMPC = ROSSlidingMPC()

    rospy.loginfo("%s: node started.", ROSSlidingMPC.name)

    # Create timer for periodic subscriber
    dur_pubsub = rospy.Duration(1./RUN_FREQ)
    ROSSlidingMPC.readTFSCallbackTimer = rospy.Timer(dur_pubsub, ROSSlidingMPC.readTFs)

    # Create timer for periodic publisher
    ROSSlidingMPC.writePoseCallbackTimer = rospy.Timer(dur_pubsub, ROSSlidingMPC.publishRobotObjectPose)
    dur = rospy.Duration(ROSSlidingMPC.dt)
    ROSSlidingMPC.solveMPCCallbackTimer = rospy.Timer(dur, ROSSlidingMPC.solveMPC)

    rospy.spin()
