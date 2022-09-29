#!/usr/bin/env python3
import rospy
import optas
import numpy as np
from math import radians
from keyboard.msg import Key
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import WrenchStamped
from custom_ros_tools.ros_comm import get_srv_handler
from custom_ros_tools.config import replace_package
from ros_pybullet_interface.srv import AddPybulletObject, AddPybulletObjectRequest
from ros_pybullet_interface.srv import ResetJointState, ResetJointStateRequest
from cob_srvs.srv import SetString
from ros_pybullet_interface.msg import PybulletObject
from haption import HaptionImpedance
np.set_printoptions(precision=2, suppress=True)

class Kuka:


    def __init__(self, dt):
        self.dt = dt
        urdf = replace_package('{ik_ros_examples}/robots/kuka_lwr.urdf')
        robot = optas.RobotModel(urdf, time_derivs=[1])
        self.robot = robot
        self.name = robot.get_name()
        builder = optas.OptimizationBuilder(T=1, robots=robot, derivs_align=True)
        goal = builder.add_parameter('goal', 2)
        dq = builder.get_model_state(self.name, t=0, time_deriv=1)
        qc = builder.add_parameter('qc', robot.ndof)
        dqc = builder.add_parameter('dqc', robot.ndof)
        ddq = (dq - dqc)/dt
        Je = robot.get_geometric_jacobian('end_effector_ball', qc, base_link='end_effector_ball')
        Jl = robot.get_global_linear_geometric_jacobian('end_effector_ball', qc)
        pc = robot.get_global_link_position('end_effector_ball', qc)
        deffl = Jl@dq
        p = pc + self.dt*deffl
        deff = Je @ dq
        q = qc + self.dt*dq
        qnom = optas.DM([radians(d) for d in [0., 30., 0., -90., 0., 60., 0.]])
        builder.add_cost_term('motion', 1e6*optas.sumsqr(deff[:2] - goal))
        builder.add_cost_term('no_motion', 1e5*optas.sumsqr(deff[2:]))
        builder.add_cost_term('min_ddq', 1e2*optas.sumsqr(ddq))
        # builder.add_cost_term('min_dq', optas.sumsqr(dq))
        # builder.add_cost_term('nominal', optas.sumsqr(q - qnom))
        builder.add_equality_constraint('maintain_z', p[2], 0.151)
        # builder.add_equality_constraint('maintain_z', deffl[2])
        # builder.add_equality_constraint('maintain_ori', deff[2:])

        # self._solver = optas.OSQPSolver(builder.build()).setup()
        self._solver = optas.CasADiSolver(builder.build()).setup('ipopt')

        self.update_joint_state = True

        self._pub = rospy.Publisher('rpbi/kuka_lwr/joint_states/target', JointState, queue_size=10)
        self._qc = None
        self._dqc = None
        self._goal = None
        rospy.Subscriber('rpbi/kuka_lwr/joint_states', JointState, self._joint_state_callback)
        # rospy.Subscriber('goal', Float64MultiArray, self._goal_callback)


    def _joint_state_callback(self, msg):
        qc = []
        dqc = []
        for name in self.robot.actuated_joint_names:
            idx = msg.name.index(name)
            qc.append(msg.position[idx])
            dqc.append(msg.velocity[idx])

        self._qc = np.array(qc)
        self._dqc = np.array(dqc)


    def _goal_callback(self, msg):
        self._goal = np.array(msg.data[:2])

    def set_goal(self, goal):
        self._goal = np.asarray(goal)


    def update(self, event):

        if not ((self._qc is not None) and (self._goal is not None) and (self._dqc is not None)):
            return

        if not self.update_joint_state:
            return

        qc = self._qc.copy()
        dqc = self._dqc.copy()
        self._solver.reset_parameters({'qc': qc, 'goal': self._goal, 'dqc': dqc})

        solution = self._solver.solve()
        dq = solution[f'{self.name}/dq'].toarray().flatten()

        q = qc + self.dt*dq

        # print(q)

        msg = JointState(name=self.robot.actuated_joint_names, position=q.tolist())
        msg.header.stamp = rospy.Time.now()
        self._pub.publish(msg)


class Haption:

    scale = 0.5

    def __init__(self, kuka):

        self._haption_setup = True
        self.kuka = kuka

        self._force = np.zeros(3)

        self._started_callback = False
        topic = 'rpbi/kuka_lwr/lwr_arm_7_joint/ft_sensor/calibrated'
        rospy.Subscriber(topic, WrenchStamped, self._callback)

        serial_number = rospy.get_param('~haption_serial_number', 153)
        self._haption_imped = HaptionImpedance(serial_number)
        self._haption_imped.setup_virtuose_force_publisher()
        self._haption_imped.start_virtuose_speed_subscriber()
        self._haption_imped.wait_for_virtuose_reset()
        self._haption_imped.wait_for_virtuose_impedance()
        self._haption_imped.setup()
        # self._haption_imped.reset()
        self._haption_setup = True
        self._haption_imped.publish_wrench()  # zero

        self._temp_timer = rospy.Timer(rospy.Duration(1.0/float(50)), self._temp)


    def _temp(self, event):
        if self._started_callback:
            self._temp_timer.shutdown()
        else:
            self._haption_imped.publish_wrench()  # zero


    def get_speed(self):
        return self._haption_imped.get_virtuose_speed()


    def _callback(self, msg):

        self._started_callback = True

        if not self._haption_setup:
            self._haption_imped.publish_wrench()  # zero
            return

        speed = self.get_speed()
        if speed is None:
            self._haption_imped.publish_wrench()  # zero
            return

        vscale = 12.
        vx = vscale*speed.virtuose_speed.linear.x
        vy = -vscale*speed.virtuose_speed.linear.y
        goal = [vx, vy]
        self.kuka.set_goal(goal)

        fx = -self.scale*msg.wrench.force.x
        fy = self.scale*msg.wrench.force.y
        alpha = 0.9
        self._force = alpha*self._force + (1.-alpha)*np.array([fx, fy, 0.])
        # rospy.logerr(self._force)
        max_f = 7.5
        self._force = np.clip(self._force, -max_f, max_f)

        self._haption_imped.publish_wrench(force=self._force)


    def reset_haption(self):
        self._haption_imped.close()


class Node:

    def __init__(self):

        rospy.init_node('nsl_node')
        rospy.on_shutdown(self.close)

        self.add_pybullet_object = get_srv_handler('rpbi/add_pybullet_object', AddPybulletObject, persistent=True)
        self.remove_pybullet_object = get_srv_handler('rpbi/remove_pybullet_object', SetString, persistent=True)
        self.move_to_initial_joint_state = get_srv_handler(f'rpbi/kuka_lwr/move_to_initial_joint_state', ResetJointState, persistent=True)

        hz = 50
        dt = 1.0/float(hz)
        self.kuka = Kuka(dt)

        self.haption = Haption(self.kuka)

        rospy.Subscriber('keyboard/keydown', Key, self._key_callback)

        rospy.Timer(rospy.Duration(dt), self.kuka.update)

    def _key_callback(self, msg):
        if msg.code == Key.KEY_r:
            self.kuka.update_joint_state = False
            req = ResetJointStateRequest(duration=0.0)
            self.move_to_initial_joint_state(req)
            # rospy.sleep(1.)
            self.remove_pushing_box()
            self.add_pushing_box()
            self.kuka.update_joint_state = True

    def add_pushing_box(self):
        obj = PybulletObject()
        obj.object_type = PybulletObject.DYNAMIC
        obj.filename = "{rpbi_examples}/configs/nsl/box.yaml"
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


    def spin(self):
        rospy.spin()

    def close(self):
        self.haption.reset_haption()


def main():
    Node().spin()


if __name__ == '__main__':
    main()
