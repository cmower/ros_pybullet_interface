#!/usr/bin/env python3
import sys
import rospy
import numpy
import numpy as np
import pyexotica as exo
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from rpbi_work.srv import MoveNextageToPrePushPose, MoveNextageToPrePushPoseResponse
from rpbi_work.srv import EffPoseFromObject, EffPoseFromObjectRequest
from ros_pybullet_interface.tf_interface import TfInterface

controller_joints_torso = ['CHEST_JOINT0']
controller_joints_head = ['HEAD_JOINT0', 'HEAD_JOINT1']
controller_joints_arm_left = ['LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5']
controller_joints_arm_right = ['RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5']
controller_joints = controller_joints_arm_left + controller_joints_arm_right + controller_joints_torso + controller_joints_head


class ExoticaInterface:

    def __init__(self, arm):

        # Setup exotica
        if arm == 'left':
            xml_filename = '{rpbi_work}/configs/nextage_ik_left.xml'
            self.target = 'TargetLeft'
        elif arm == 'right':
            xml_filename = '{rpbi_work}/configs/nextage_ik_right.xml'
            self.target = 'TargetRight'

        self.solver = exo.Setup.load_solver(xml_filename)
        self.problem = self.solver.get_problem()
        self.scene = self.problem.get_scene()

    def set_goal(self, pos):
        self.scene.attach_object_local(self.target, 'base_link', pos)


class Node:

    pre_pre_offset = np.array([-0.02, 0, 0.1])  # TODO: NOTE: this may need to be tuned

    def __init__(self):

        # Init node
        rospy.init_node('move_nextage_to_pre_push_pose')

        # Setup exotica
        self.exo_left = ExoticaInterface('left')
        self.exo_right = ExoticaInterface('right')

        # Setup tf interface
        self.tf = TfInterface()

        # Setup joint state publisher
        self.joint_state_pub = rospy.Publisher('rpbi/nextage/joint_state/target', JointState, queue_size=10)

        # Setup service
        rospy.Service('move_nextage_to_pre_push_pose', MoveNextageToPrePushPose, self.move_nextage_to_pre_push_pose)

        rospy.Service('move_nextage_to_pre_push_pose_rev', MoveNextageToPrePushPose, self.move_nextage_to_pre_push_pose_rev)

    def move_nextage_to_pre_push_pose_rev(self, req):

        info = ''

        # Get goal position
        rospy.wait_for_service('get_eff_pose_from_pushing_object')
        try:
            handle = rospy.ServiceProxy('get_eff_pose_from_pushing_object', EffPoseFromObject)
            resp = handle(EffPoseFromObjectRequest(object_frame_id=req.object_frame_id, parent_frame_id=req.parent_frame_id, arm=req.arm))
        except rospy.ServiceException as e:
            rospy.logerr('Service error: %s', str(e))
            info = 'Failed to retrieve goal from server'
            return MoveNextageToPrePushPoseResponse(success=False, info=info)

        if not resp.success:
            info = 'Failed to retrieve goal from server'
            return MoveNextageToPrePushPoseResponse(success=False, info=info)

        goal = numpy.array(resp.position)

        # Get current joint position
        # msg = rospy.wait_for_message('/nextagea/joint_states', JointState)
        msg = rospy.wait_for_message('/rpbi/nextage/joint_state', JointState)
        qstart = getattr(self, 'exo_%s'%req.arm).resolve_joint_state_msg(msg)

        # Move robot
        # NOTE: robot will move as follows:
        # 1. from where ever it is to a pre-pre push pose (above/behind object), this is to prevent robot colliding with object
        q, success = self.move_robot(goal+self.pre_pre_offset, req.arm, req.Tmax, qstart)
        if not success:
            # move to above/behind object (pre-pre pose)
            info = 'Failed to move nextage to pre-pre eff goal pose'
            return MoveNextageToPrePushPoseResponse(success=False, info=info)

        return MoveNextageToPrePushPoseResponse(success=True, info=info)

    def move_nextage_to_pre_push_pose(self, req):

        info = ''

        # Get goal position
        rospy.wait_for_service('get_eff_pose_from_pushing_object')
        try:
            handle = rospy.ServiceProxy('get_eff_pose_from_pushing_object', EffPoseFromObject)
            resp = handle(EffPoseFromObjectRequest(object_frame_id=req.object_frame_id, parent_frame_id=req.parent_frame_id, arm=req.arm))
        except rospy.ServiceException as e:
            rospy.logerr('Service error: %s', str(e))
            info = 'Failed to retrieve goal from server'
            return MoveNextageToPrePushPoseResponse(success=False, info=info)

        if not resp.success:
            info = 'Failed to retrieve goal from server'
            return MoveNextageToPrePushPoseResponse(success=False, info=info)

        goal = numpy.array(resp.position)

        # rospy.loginfo('Does this goal look good?')
        # import sys
        # sys.exit(0)

        # Get current joint position
        msg = rospy.wait_for_message('/nextagea/joint_states', JointState)
        qstart = self.resolve_joint_msg_order(msg)

        # Move robot
        # NOTE: robot will move as follows:
        # 1. from where ever it is to a pre-pre push pose (above/behind object), this is to prevent robot colliding with object
        # 2. from pre-pre-push pose to object
        q, success = self.move_robot(goal+self.pre_pre_offset, req.arm, req.Tmax, qstart)
        if not success:
            # move to above/behind object (pre-pre pose)
            info = 'Failed to move nextage to pre-pre eff goal pose'
            return MoveNextageToPrePushPoseResponse(success=False, info=info)

        rospy.logwarn('Robot should be at pre-pre grasp pose now')
        # sys.exit(0)

        if not self.move_robot(goal, req.arm, 1.0, q)[1]:
            # move to requested position
            info = 'Failed to move nextage to pre eff goal pose'
            return MoveNextageToPrePushPoseResponse(success=False, info=info)

        return MoveNextageToPrePushPoseResponse(success=True, info=info)

    def move_robot(self, pos, arm, Tmax, qstart):

        success = True

        try:

            rospy.loginfo('Robot will start moving to a goal state.')

            # Solve exotica
            e = getattr(self, 'exo_%s' % arm)
            e.set_goal(pos)
            e.problem.start_state = qstart
            qgoal = e.solver.solve()[0]

            # Interpolate
            keep_running = True
            rate = rospy.Rate(50)
            start_time = rospy.Time.now().to_sec()
            while keep_running:
                tcurr = rospy.Time.now().to_sec() - start_time
                alpha = tcurr/Tmax
                q = alpha*qgoal + (1-alpha)*qstart
                self.publish_joint_state(q, arm)
                rospy.loginfo('Commanded robot, %.2f percent complete', 100.*alpha)
                if alpha > 1.0:
                    keep_running = False
                    rospy.loginfo('Robot has reached goal state.')

        except Exception as e:
            rospy.logerr("Failed to move robot: %s", e)

            success = False

        return qgoal, success


    def resolve_joint_msg_order(self, msg):
        cmd = []
        for i, name in enumerate(controller_joints):
            joint_index = msg.name.index(name)
            joint_position = msg.position[joint_index]
            cmd.append(joint_position)
        return np.array(cmd)

    def publish_joint_state(self, q_exo, arm):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = getattr(self, 'exo_%s' % arm).scene.get_controlled_joint_names()
        msg.position = q_exo.tolist()
        # msg.position = [q_exo[0], 0, 0]  # chest, head0, head1
        # msg.position += q_exo[1:].tolist()
        self.joint_state_pub.publish(msg)

    def spin(self):
        rospy.spin()

def main():
    Node().spin()


if __name__ == '__main__':
    main()
