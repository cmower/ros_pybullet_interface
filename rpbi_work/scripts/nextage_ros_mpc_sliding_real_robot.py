#!/usr/bin/env python3
import sys
import time
import math
import numpy as np
from std_msgs.msg import Int8
from scipy.spatial.transform import Rotation as R
import tf2_ros
import casadi as cs
from ros_pybullet_interface.config import load_config
np.set_printoptions(precision=3)
# service for stepping the simulation from code
from ros_pybullet_interface.srv import ManualPybullet, ManualPybulletRequest
from rpbi_work.srv import PusherSlider, PusherSliderResponse

import rospy

# ROS message types
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64MultiArray
# ros_pybullet funcs

# --- import external library
import sliding_pack

GLB_ORI_ROBOT = np.array([[1., 0., 0.],
                          [0., 1., 0.],
                          [0., 0., 1.]])
# ROBOT_HEIGHT = 0.75
ROBOT_HEIGHT = -0.05
VISUAL_OBJ_HEIGHT = -0.08
# OBJECT_TARGET_FRAME_ID = "rpbi_work/nextage_sliding_box_visual"  # visual box
OBJECT_NOM_FRAME_ID = "pushing_box_visual_nom"  # visual box
OBJECT_GOAL_FRAME_ID = "pushing_box_visual_goal"  # visual box
# WORLD_FRAME = "rpbi/world"
WORLD_FRAME = "sim/nextage_base"
SHOW_NOM_FLAG = False
RUN_FREQ = 50

class ROSSlidingMPC:

    def __init__(self):

        # Name of node
        self.name = rospy.get_name()
        rospy.loginfo("%s: node started.", self.name)

        # Flag for debugging plot
        self.plotFlag = True

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

        # get working arm
        arm = rospy.get_param('~arm')  # left/right

        # get configuration file for opt setup
        setup_file_name = '{rpbi_work}/configs/nextage_real_setup_%.yaml' % arm
        setup_config = load_config(setup_file_name)
        # get setup configurations
        self.real_setup = setup_config['real_setup']
        self.robot_name = setup_config['robot_name']
        self.end_effector_target_frame_id = setup_config['end_effector_target_frame_id']
        self.object_name = setup_config['object_name']
        self.obstacle_name = setup_config['obstacle_name']

        # nominal trajectory file for planning
        nom_traj_file_name = '{rpbi_work}/configs/nextage_nom_config_%.yaml' % arm
        nom_config = load_config(nom_traj_file_name)

        # Get config files
        #  -------------------------------------------------------------------
        sliding_dyn_file_name = rospy.get_param('~sliding_param_dyn', [])[0]
        dyn_config = load_config(sliding_dyn_file_name)
        tracking_traj_file_name = rospy.get_param('~sliding_param_tracking_traj', [])[0]
        tracking_config = load_config(tracking_traj_file_name)
        #  -------------------------------------------------------------------

        # Initialize internal variables
        self._cmd_robot_pose = None
        self._cmd_obj_pose = None
        self._cmd_nom_visual_obj_pose = None
        self._cmd_goal_visual_obj_pose = None
        self._obj_pose = None
        self._obs_pose = None
        self._robot_pose = None

        # Set Problem constants
        #  -------------------------------------------------------------------
        self.T = nom_config['TimeHorizon']  # time of the simulation is seconds
        self.freq = 25  # number of increments per second
        N_MPC = 25  # time horizon for the MPC controller
        #  -------------------------------------------------------------------
        # Computing Problem constants
        #  -------------------------------------------------------------------
        self.dt = 1.0/freq # sampling time
        N = int(T*freq) # total number of iterations
        self.Nidx = int(N)
        #  -------------------------------------------------------------------
        # define system dynamics
        #  -------------------------------------------------------------------
        self.dyn = sliding_pack.dyn.Sys_sq_slider_quasi_static_ellip_lim_surf(
                dyn_config, 
                tracking_config['contactMode']
        )

        # Setup timers
        self.dur_pubsub = rospy.Duration(1./RUN_FREQ)
        rospy.Timer(self.dur_pubsub, self.readTFs)

        # publisher for MPC completion
        self._mpc_completion_pub = rospy.Publisher('/mpc_completion_flag', Int8, queue_size=1)

        # Setup services
        selt.runningMPC = False
        selt.planningDone = False
        rospy.Service('planning_sliding_%s' % arm, PusherSlider, self.plan_sliding_service)
        rospy.Service('executing_mpc_%s' % arm, PusherSlider, self.start_mpc_service)

    def plan_sliding_service(self, req):
        rospy.loginfo("I am planning")
        # Loop till the pos and ori of the object has been read.
        try:
            # Read the position and orientation of the robot from the /tf topic
            trans = self.mpc_listen_buff.lookup_transform(WORLD_FRAME, f"{self.object_name}", rospy.Time())
            # replaces base_position = config['base_position']
            obj_pos0 = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
            obj_ori_quat0 = np.array([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
            obj_ori0 = R.from_quat(obj_ori_quat0).as_euler('xyz', degrees=False)[2]
        except:
            rospy.logerr(f"{self.name}: /tf topic does NOT have 123 {self.object_name}")
        # read obstacle pose
        try:
            # Read the position and orientation of the robot from the /tf topic
            trans = self.mpc_listen_buff.lookup_transform(WORLD_FRAME, f"{self.obstacle_name}", rospy.Time())
            # replaces base_position = config['base_position']
            obs_pos0 = [trans.transform.translation.x, trans.transform.translation.y]
        except:
            rospy.logwarn(f"{self.name}: /tf topic does NOT have 456 {self.obstacle_name}")
        # 
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
                0, [obj_pos0[0], obj_pos0[1], obj_ori0, 0],
                obsCentre=obsCentre, obsRadius=obsRadius)
        #  ---------------------------------------------------------------
        if self.plotFlag:
            fig, ax = sliding_pack.plots.plot_nominal_traj(
                        cs.DM(x0_nom), cs.DM(x1_nom))
            # add computed nominal trajectory
            X_nom_val_plot = np.array(X_nom_val)
            ax.plot(X_nom_val_plot[0, :], X_nom_val_plot[1, :], color='blue',
                    linewidth=2.0, linestyle='dashed')
            import matplotlib.pyplot as plt
            if optObj.numObs > 0:
                for i in range(len(obsCentre)):
                    circle_i = plt.Circle(obsCentre[i], obsRadius[i], color='b')
                    ax.add_patch(circle_i)
            # ax.set_xlim((-0.5, 0.5))
            # ax.set_ylim((-1.0, -0.3))
            plt.show()
        self.X_nom_val = X_nom_val
        # set object goal transform
        visual_obj_pose_2d = np.array(X_goal)
        visual_obj_pos = np.hstack((visual_obj_pose_2d[0:2], VISUAL_OBJ_HEIGHT))
        visual_obj_ori = R.from_rotvec(np.array([0., 0., X_goal[2]]))
        visual_obj_ori_quat = visual_obj_ori.as_quat()
        self._cmd_goal_visual_obj_pose = np.hstack((visual_obj_pos, visual_obj_ori_quat))

        #  ------------------------------------------------------------------
        # define optimization problem
        #  -------------------------------------------------------------------
        self.optObj = sliding_pack.to.buildOptObj(
                self.dyn, N_MPC, tracking_config,
                self.X_nom_val, dt=self.dt)
        #  -------------------------------------------------------------------
        success = True
        self.planningDone = True
        return PusherSliderResponse(success=success, info=info)

    def start_mpc_service(self, rep):

        success = True
        info = ''

        if not self.planningDone:
            info = "need to call plan_sliding_service first!"
            success = False
            rospy.logerr(info)
            return PusherSliderResponse(success=success, info=info)
        if self.runningMPC:
            info = "recieved request to start MPC, but it is already running!"
            success = False
            rospy.logerr(info)
            return PusherSliderResponse(success=success, info=info)

        # Create timer for periodic publisher
        self.publishRobotObjectPoseTimer = rospy.Timer(self.dur_pubsub, self.publishRobotObjectPose)
        # Start main loop
        dur = rospy.Duration(self.dt)
        self.solveMPCCallbackTimer = rospy.Timer(dur, self.solveMPC)
        self.runningMPC = True
        rospy.loginfo('Switched on MPC')
        return PusherSliderResponse(success=success, info=info)

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
        self.publishPose(self._cmd_nom_visual_obj_pose, OBJECT_NOM_FRAME_ID)
        self.publishPose(self._cmd_goal_visual_obj_pose, OBJECT_GOAL_FRAME_ID)

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
            self._mpc_completion_pub.publish(Int8(data=-1))
            self.solveMPCCallbackTimer.shutdown()
            self.publishRobotObjectPoseTimer.shutdown()
            rospy.logerr("Object or pose does not exist!")
            return

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
        self._cmd_nom_visual_obj_pose = np.hstack((visual_obj_pos, visual_obj_ori_quat))
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
            # publish completion
            self._mpc_completion_pub.publish(Int8(data=0))
            # shutdown timers
            self.solveMPCCallbackTimer.shutdown()
            self.publishRobotObjectPoseTimer.shutdown()
            rospy.loginfo("End of nominal trajectory")
        else:
            # input()
            pass

        if not self.real_setup:
            # service stuff
            rospy.wait_for_service('manual_pybullet_step')
            try:
                srv_handle = rospy.ServiceProxy('manual_pybullet_step', ManualPybullet)
                srv_req = ManualPybulletRequest(num_steps=1, hz=RUN_FREQ) # this gets the inputs for the service func
                srv_handle(srv_req) # this is calling the service
            except rospy.ServiceException as e:
                print("It failed to step with error: %s"%e)

if __name__=='__main__':

    # --- setup the ros interface --- #
    rospy.init_node('test_ros_traj_opt_obj3D', anonymous=True)
    rospy.logwarn("ATTENTION: This node will not run without the impact-TO library!")
    # Initialize node class
    ROSSlidingMPC()

    rospy.spin()
