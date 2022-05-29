#!/usr/bin/env python3
import time
import rospy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from dmp.data_collector import TFPositionDMPDataCollector
from dmp.dmp_proc import DMPProcessor
from custom_ros_tools.tf import TfInterface
from keyboard.msg import Key
from std_srvs.srv import SetBool, Trigger
from custom_srvs.srv import SetTransform
from std_msgs.msg import Float64MultiArray
from custom_ros_tools.ros_comm import get_srv_handler
from ros_pybullet_interface.msg import CalculateInverseKinematicsProblem
from ros_pybullet_interface.srv import ResetEffState, ResetEffStateRequest
from ros_pybullet_interface.srv import ResetJointState, ResetJointStateRequest
from ros_pybullet_interface.srv import AddPybulletObject, AddPybulletObjectRequest
from ros_pybullet_interface.msg import PybulletObject
from cob_srvs.srv import SetString


class TfTracker:

    def __init__(self, tf, parent, child):
        self.tf = tf
        self.parent = parent
        self.child = child
        rospy.Timer(rospy.Duration(0.01), self.main_loop)
        self.transform = None

    def main_loop(self, event):
        self.transform = self.tf.get_tf_msg(self.parent, self.child)

    def distance(self):
        if self.transform is None:
            return np.inf
        return np.linalg.norm(self.tf.msg_to_pos(self.transform))

    def get_pos(self):
        return self.tf.msg_to_pos(self.transform)

class Node:

    def __init__(self):
        rospy.init_node('run_lfd_node')
        self.tf = TfInterface()
        self.eff = TfTracker(self.tf, 'teleop_origin', 'rpbi/kuka_lwr/end_effector_ball')
        self.robot_is_on = False
        self.dmp = None
        self.goal = None
        self.t_plan = None
        self.pos_traj_plan = None
        self.plot_the_results = False
        self.dmp_proc = DMPProcessor()
        self.data_collector = TFPositionDMPDataCollector('teleop_origin', 'rpbi/kuka_lwr/end_effector_ball', interpolate=50)

        # Get service handlers
        self.add_pybullet_object = get_srv_handler('rpbi/add_pybullet_object', AddPybulletObject, persistent=True)
        self.remove_pybullet_object = get_srv_handler('rpbi/remove_pybullet_object', SetString, persistent=True)
        self.move_to_eff_state = get_srv_handler(f'rpbi/kuka_lwr/move_to_eff_state', ResetEffState, persistent=True)
        self.move_to_initial_joint_state = get_srv_handler(f'rpbi/kuka_lwr/move_to_initial_joint_state', ResetJointState, persistent=True)
        self.reset_teleop_transform_zero = get_srv_handler('reset_teleop_transform_to_zero', Trigger, persistent=True)
        self.reset_teleop_transform = get_srv_handler('reset_teleop_transform', SetTransform, persistent=True)

        self.toggle_human_interface = get_srv_handler('operator_node/toggle', SetBool, persistent=True)
        self.toggle_teleop_tf = get_srv_handler('toggle_teleop_tf', SetBool, persistent=True)
        self.toggle_ik_setup = get_srv_handler('ik/setup/trac_ik/toggle', SetBool, persistent=True)
        self.toggle_ik_solver = get_srv_handler('ik/solver/trac_ik/toggle', SetBool, persistent=True)

        # Setup ros communication
        self.signal_pub = rospy.Publisher('operator_node/signal', Float64MultiArray, queue_size=10)
        rospy.Subscriber('keyboard/keydown', Key, self.keyboard_callback)

    def keyboard_callback(self, msg):
        if msg.code == Key.KEY_1:
            self.move_to_initial_pose()
        elif msg.code == Key.KEY_2:
            if not self.robot_is_on:
                self.start_teleop()
            else:
                self.stop_teleop()
        elif msg.code == Key.KEY_3:
            if self.dmp is None:
                self.learn_dmp()
            else:
                self.exec_dmp_plan()

    def add_pushing_box(self):
        obj = PybulletObject()
        obj.object_type = PybulletObject.DYNAMIC
        obj.filename = "{rpbi_examples}/configs/lfd/pushing_box.yaml"
        req = AddPybulletObjectRequest(pybullet_object=obj)
        resp = self.add_pybullet_object(req)
        if resp.success:
            rospy.loginfo(resp.message)
        else:
            rospy.logwarn(resp.message)

    def remove_pushing_box(self):
        resp = self.remove_pybullet_object('pushing_box')
        if resp.success:
            rospy.loginfo(resp.message)
        else:
            rospy.logwarn(resp.message)

    def move_to_initial_pose(self):

        if self.robot_is_on:
            rospy.logwarn('robot is on')
            return

        rospy.loginfo('moving robot to start state ...')
        self.robot_is_on = True

        # Move to initial joint configuration
        req = ResetJointStateRequest(duration=2.0)
        self.move_to_initial_joint_state(req)

        # Get eff target
        timeout = 10.0
        start_time = time.time()
        while (time.time()-start_time) < timeout:
            init_eff_pos, init_eff_rot = self.tf.get_tf('rpbi/world', 'teleop_origin')
            if init_eff_pos is not None:
                break
        else:
            rospy.logerr("could not recieve end-effector target transform")
            sys.exit(0)

        # Setup problem
        problem = CalculateInverseKinematicsProblem()
        problem.link_name = 'end_effector_ball'
        problem.targetPosition = init_eff_pos
        problem.targetOrientation = init_eff_rot

        # Setup request
        duration = 0.5
        req = ResetEffStateRequest(problem=problem, duration=duration)

        # Move robot
        self.move_to_eff_state(req)

        self.robot_is_on = False
        rospy.loginfo('moved robot to start state')

    def start_teleop(self):

        if self.robot_is_on:
            rospy.logwarn('robot is on')
            return

        if self.eff.distance() > 0.03:
            rospy.logwarn('end effector is too far away from start pose, press 1 to move robot to intial pose')
            return

        # Ensure teleop transform is zero
        self.reset_teleop_transform_zero()

        rospy.loginfo('started teleoperation')

        self.add_pushing_box()

        self.toggle_human_interface(True)
        self.toggle_teleop_tf(True)
        self.toggle_ik_setup(True)
        self.toggle_ik_solver(True)

        self.robot_is_on = True

        self.dmp = None  # collecting new data -> will want to learn this
        self.data_collector.reset()
        self.data_collector.start()

    def stop_teleop(self):
        self.data_collector.stop()
        self.goal = self.eff.get_pos()
        self.toggle_human_interface(False)
        self.toggle_teleop_tf(False)
        self.toggle_ik_setup(False)
        self.toggle_ik_solver(False)
        self.remove_pushing_box()
        self.robot_is_on = False
        rospy.loginfo('stopped teleoperation')

    def learn_dmp(self):
        if self.data_collector.is_empty():
            rospy.logwarn('no data has been collected')
            return

        rospy.loginfo('learning dmp from collected data...')

        t, pos_traj = self.data_collector.get()
        k_gain = 100.0
        d_gain = 2.0*np.sqrt(k_gain)
        num_bases = 4
        self.dmp = self.dmp_proc.learn_dmp(t, pos_traj, k_gain, d_gain, num_bases)

        rospy.loginfo('dmp learned')

    def exec_dmp_plan(self):

        if self.eff.distance() > 0.03:
            rospy.logwarn('end effector is too far away from start pose, press 1 to move robot to intial pose')
            return

        self.robot_is_on = True

        # Generate plan
        rospy.loginfo('generating plan from learned dmp...')
        pos0 = self.eff.get_pos()
        rlim = 0.1
        pos0[:2] += np.random.uniform(-rlim, rlim, size=(2,))
        vel0 = np.zeros(3)
        t0 = 0.0
        goal_thresh = np.array([0.03, 0.03, 0.03])
        seg_length = -1
        tau = 2.0*self.dmp.tau
        dt = 0.02
        integrate_iter = 5
        success, t, pos_traj, vel_traj = self.dmp_proc.generate_plan(
            self.dmp, pos0, vel0, t0, self.goal,
            goal_thresh, seg_length, tau, dt,
            integrate_iter,
        )
        self.t_plan = t
        self.pos_traj_plan = pos_traj
        if success:
            rospy.loginfo('plan generated using learned dmp')
        else:
            rospy.logwarn('failed to generate plan')
            return

        # Execute plan
        rospy.loginfo('executing plan ...')
        self.add_pushing_box()
        self.reset_teleop_transform(self.tf.pack_tf(pos0, [0, 0, 0, 1]))
        self.toggle_teleop_tf(True)
        self.toggle_ik_setup(True)
        self.toggle_ik_solver(True)

        N = vel_traj.shape[1]
        rate = rospy.Rate(int(1.0/dt))
        for i in range(N):
            self.signal_pub.publish(Float64MultiArray(data=vel_traj[:,i].flatten().tolist()))
            rate.sleep()

        rospy.loginfo('plan executed')
        self.toggle_teleop_tf(False)
        self.toggle_ik_setup(False)
        self.toggle_ik_solver(False)
        self.remove_pushing_box()
        self.robot_is_on = False
        self.plot_the_results = True

    def plot_results(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        t_demo, pos_traj_demo = self.data_collector.get()
        ax.plot(pos_traj_demo[0,:], pos_traj_demo[1,:], pos_traj_demo[2,:], '-b', label='Demo')
        ax.plot(self.pos_traj_plan[0,:], self.pos_traj_plan[1,:], self.pos_traj_plan[2,:], '-r', label='Plan')
        ax.legend()
        plt.show()
        self.plot_the_results = False

    def spin(self):
        rate = rospy.Rate(100.0)
        while not rospy.is_shutdown():
            if self.plot_the_results:
                self.plot_results()
            rate.sleep()

def main():
    Node().spin()

if __name__ == '__main__':
    main()
