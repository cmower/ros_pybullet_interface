#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from rpbi_work.srv import MoveNextageToState, MoveNextageToStateResponse

controller_joints_torso = ['CHEST_JOINT0']
controller_joints_head = ['HEAD_JOINT0', 'HEAD_JOINT1']
controller_joints_arm_left = ['LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5']
controller_joints_arm_right = ['RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5']
controller_joints = controller_joints_arm_left + controller_joints_arm_right + controller_joints_torso + controller_joints_head

class Node:

    def __init__(self):
        rospy.init_node('move_nextage_to_state')
        self.joint_state_pub = rospy.Publisher('rpbi/nextage/joint_state/target', JointState, queue_size=10)
        rospy.Service('move_nextage_to_state', MoveNextageToState, self.move)

    def move(self, req):

        success = True
        info = ''

        try:

            # Get current and goal joint position
            msg = rospy.wait_for_message('/nextagea/joint_states', JointState)
            qstart = self.resolve_joint_msg_order(msg)
            qgoal = np.array(req.goal_position)
            Tmax = req.Tmax

            # Interpolate
            keep_running = True
            rate = rospy.Rate(50)
            start_time = rospy.Time.now().to_sec()
            while keep_running:
                tcurr = rospy.Time.now().to_sec() - start_time
                alpha = tcurr/Tmax
                q = alpha*qgoal + (1-alpha)*qstart
                self.publish_joint_state(q)
                rospy.loginfo('Commanded robot, %.2f percent complete', 100.*alpha)
                if alpha > 1.0:
                    keep_running = False
                    rospy.loginfo('Robot has reached goal state.')
        except Exception as e:
            success = False
            info = str(e)
            rospy.logerr('failed to move robot to state: %s', info)


        return MoveNextageToStateResponse(success=success, info=info)



    def resolve_joint_msg_order(self, msg):
        cmd = []
        for i, name in enumerate(controller_joints):
            joint_index = msg.name.index(name)
            joint_position = msg.position[joint_index]
            cmd.append(joint_position)
        return np.array(cmd)


    def publish_joint_state(self, q_exo):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = controller_joints #self.scene.get_controlled_joint_names()
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
