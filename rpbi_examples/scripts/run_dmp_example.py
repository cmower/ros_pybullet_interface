#!/usr/bin/env python3
import sys
import rospy
import numpy as np
from typing import List
from dataclasses import dataclass, field
from scipy.interpolate import interp1d
from keyboard.msg import Key
from std_srvs.srv import SetBool
from sensor_msgs.msg import JointState
from dmp.msg import DMPTraj, DMPPoint
from dmp.srv import LearnDMPFromDemo, SetActiveDMP, GetDMPPlan
from rpbi.tf_interface import TfInterface
from ros_pybullet_interface.srv import ResetJointState, ResetJointStateRequest
import matplotlib
matplotlib.use('GTK3Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def get_srv_handle(srv_name, srv_type):
    rospy.wait_for_service(srv_name)
    return rospy.ServiceProxy(srv_name, srv_type)

@dataclass
class DataPoint:
    time: float
    position: List[float] = field(default_factory=True)

class DataCollector:

    def __init__(self, parent_id, child_id, nd, dt):
        self.tf = TfInterface()
        self.parent_id = parent_id
        self.child_id = child_id
        self.nd = nd  # number of interpolated points
        self.dt = dt
        self.data = None
        self.timer = None

    def start(self):
        self.data = []
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.loop)

    def stop(self):
        self.timer.shutdown()
        self.timer = None

    def loop(self, event):
        pos, _ = self.tf.get_tf(self.parent_id, self.child_id)
        t = rospy.Time.now().to_sec()
        if pos is None: return
        self.data.append(DataPoint(t, pos))

    def get(self):

        # Get raw data
        t_ = np.array([d.time for d in self.data]) - self.data[0].time
        p_ = np.array([d.position for d in self.data]).T

        # Interpolate
        pfun = interp1d(t_, p_)
        t = np.linspace(0, t_.max(), self.nd)
        p = pfun(t)

        # Build demo trajectory
        demotraj = DMPTraj(
            times=t.tolist(),
            points=[DMPPoint(positions=p[:,i].tolist()) for i in range(self.nd)],
        )

        return demotraj


class Node:

    def __init__(self):

        # Intialize joy node
        rospy.init_node('run_dmp_example')

        # Setup variables
        self.teleop_on = False
        self.dmp = None

        # Setup data collector
        self.data_collector = DataCollector('kuka_start', 'kuka_target', 100, 0.01)

        # Grab service handles
        self.toggle_operator_signal = get_srv_handle('operator_node/scalenode/toggle_callback', SetBool)
        self.toggle_teleop_tf = get_srv_handle('toggle_teleop_tf', SetBool)
        self.toggle_tf_to_floatarray = get_srv_handle('toggle_tf_to_floatarray', SetBool)
        self.toggle_ik = get_srv_handle('toggle_ik', SetBool)
        self.learn_dmp_from_demo = get_srv_handle('learn_dmp_from_demo', LearnDMPFromDemo)
        self.set_active_dmp = get_srv_handle('set_active_dmp', SetActiveDMP)
        self.get_dmp_plan = get_srv_handle('get_dmp_plan', GetDMPPlan)
        self.reset_joint_state = get_srv_handle('rpbi/kuka_lwr/move_to_joint_state', ResetJointState)

        # Setup button subscriber
        self.sub = rospy.Subscriber('keyboard/keydown', Key, self.keyboard_callback)

        rospy.loginfo('start dmp example demo')

        print("\ncontrols:")
        human_interface = rospy.get_param('human_interface')
        if human_interface=='keyboard':
            print("  press 1 to start teleoperation, and recording")
            print("    control 3D position of end-effector using keys: LEFT, RIGHT, UP, DOWN, W, S")
            print("  press 1 again to stop teleoperation, and recording")
            print("  press 2 to learn the dmp")
            print("  press 3 to execute the dmp")
        elif human_interface=='spacenav':
            pass

    def keyboard_callback(self, msg):

        if msg.code == Key.KEY_1:
            if not self.teleop_on:
                self.start_teleop()
                self.data_collector.start()
                rospy.loginfo('started recording data, do some movement ...')
            else:
                self.data_collector.stop()
                self.stop_teleop()
                rospy.loginfo('stopped recording data, press 2 to learn DMP')

        elif msg.code == Key.KEY_2:
            rospy.loginfo('learning dmp ...')
            self.learn_dmp()
            rospy.loginfo('learning dmp complete')

        elif msg.code == Key.KEY_3:
            rospy.loginfo('playing dmp plan ...')
            self.play_dmp()
            rospy.loginfo('completed plan')

        elif msg.code in {Key.KEY_ESCAPE, Key.KEY_q}:
            rospy.loginfo('user quit, goodbye!')
            self.sub.unregister()
            sys.exit(0)

    def start_teleop(self):
        self.toggle_operator_signal(True)
        self.toggle_teleop_tf(True)
        self.toggle_tf_to_floatarray(True)
        self.toggle_ik(True)
        self.teleop_on = True

    def stop_teleop(self):
        self.toggle_operator_signal(False)
        self.toggle_teleop_tf(False)
        self.toggle_tf_to_floatarray(False)
        self.toggle_ik(False)
        self.teleop_on = False

    def learn_dmp(self):

        # Learn dmp
        demotraj = self.data_collector.get()
        dims = len(demotraj.points[0].positions)
        k_gains = [100.0]*dims
        d_gains = [2.0*np.sqrt(100.0)]*dims
        num_bases = len(demotraj.points)
        self.dmp = self.learn_dmp_from_demo(demotraj, k_gains, d_gains, num_bases)

        # Set active dmp
        self.set_active_dmp(self.dmp.dmp_list)

    def play_dmp(self):

        # Reset joint position
        q = np.deg2rad([
            -67.43029874237594,
            -43.27708308875299,
            109.3690545555588,
            -119.9388173764962,
            77.07446236689286,
            -34.464300806724026,
            -21.86171402255347
        ])  # initial joint config as in configs/dmp_example/kuka_lwr.yaml
        e = np.deg2rad(30.0)
        q += np.random.uniform(-e, e, size=q.shape)  # make sure it's not the same start point (but near'ish)

        rospy.loginfo('sending robot to starting position ...')
        joint_state = JointState()
        joint_state.name = [f"lwr_arm_{i}_joint" for i in range(7)]
        joint_state.position = q.tolist()
        duration = 3.0
        self.reset_joint_state(joint_state, duration)
        rospy.loginfo('robot arrived at starting position')
        rospy.sleep(1.0)

        # Get end-effector position
        pos, _ = self.data_collector.tf.get_tf('kuka_start', 'rpbi/kuka_lwr/lwr_arm_7_link')
        if pos is None:
            rospy.logerr('could not recieve tf, cannot play dmp')
            return

        # Get plan
        x0 = np.asarray(pos).tolist()
        xdot0 = [0.0, 0.0, 0.0]
        t0 = 0.0
        goal = self.data_collector.data[-1].position  # final position of demo
        goal_thresh = [0.2]*3
        seg_length = -1
        tau = 2*self.dmp.tau
        dt = 0.01
        integrate_iter = 500
        plan = self.get_dmp_plan(x0, xdot0, t0, goal, goal_thresh, seg_length, tau, dt, integrate_iter)

        # Plot plan
        plot_plan = rospy.get_param('~plot_param', True)
        if plot_plan:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')

            def plot_line(ax, points, fmt, **kwargs):
                x = [pt.positions[0] for pt in points]
                y = [pt.positions[1] for pt in points]
                z = [pt.positions[2] for pt in points]
                ax.plot(x, y, z, fmt, **kwargs)

            demo = self.data_collector.get()
            plot_line(ax, demo.points, '-k', label='demo')
            plot_line(ax, plan.plan.points, '-b', label='plan')
            ax.plot([demo.points[0].positions[0]], [demo.points[0].positions[1]], [demo.points[0].positions[2]], 'ok')
            ax.plot([plan.plan.points[0].positions[0]], [plan.plan.points[0].positions[1]], [plan.plan.points[0].positions[2]], 'ob')
            ax.legend()
            plt.show()

        # Execute plan
        self.data_collector.tf.set_tf('kuka_start', 'kuka_target', x0)
        self.toggle_tf_to_floatarray(True)
        self.toggle_ik(True)
        rate = rospy.Rate(int(1.0/dt))
        for p in plan.plan.points:
            self.data_collector.tf.set_tf('kuka_start', 'kuka_target', p.positions)
            rate.sleep()
        self.toggle_tf_to_floatarray(False)
        self.toggle_ik(False)

    def spin(self):
        rospy.spin()


def main():
    Node().spin()

if __name__ == '__main__':
    main()
