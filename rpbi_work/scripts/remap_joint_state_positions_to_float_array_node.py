#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from rpbi_work.srv import Toggle, ToggleResponse

"""Map JointState positions messages onto Float64MultiArray messages."""

# larm_id = [
#     'LARM_JOINT0',
#     'LARM_JOINT1',
#     'LARM_JOINT2',
#     'LARM_JOINT3',
#     'LARM_JOINT4',
#     'LARM_JOINT5',
# ]

# rarm_id = [
#     'RARM_JOINT0',
#     'RARM_JOINT1',
#     'RARM_JOINT2',
#     'RARM_JOINT3',
#     'RARM_JOINT4',
#     'RARM_JOINT5',
# ]

# torso_id = ['CHEST_JOINT0']

# head_id = [
#     'HEAD_JOINT0',
#     'HEAD_JOINT1',
# ]

controller_joints_torso = ['CHEST_JOINT0']
controller_joints_head = ['HEAD_JOINT0', 'HEAD_JOINT1']
controller_joints_arm_left = ['LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5']
controller_joints_arm_right = ['RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5']
controller_joints = controller_joints_arm_left + controller_joints_arm_right + controller_joints_torso + controller_joints_head

class Node:


    def __init__(self):

        # Setup ros node
        rospy.init_node('remap_joint_state_positions_to_float_array_node')

        # Setup publisher
        self.pub = rospy.Publisher('nextagea/streaming_controller/command', Float64MultiArray, queue_size=10)

        # Setup service
        self.sub = None
        self.sub_on = False
        rospy.loginfo('Remapper JointState.position -> Float64MultiArray initialized.')
        rospy.Service('toggle_remaper', Toggle, self.toggle_callback)

    def toggle_callback(self, req):
        if req.switch == 'on':
            success, info = self.start_callback(req)
        elif req.switch == 'off':
            success, info = self.stop_callback(req)
        else:
            info = 'failed to turn %s remapper' % req.switch
            rospy.logerr(info)
            success = False
        return ToggleResponse(success=success, info=info)

    def start_callback(self, req):

        success = True
        info = ''

        if self.sub_on:
            info = "recieved request to start remapper, but it is already running!"
            success = False
            rospy.logerr(info)
            return success, info

        # Start callback
        self.sub = rospy.Subscriber('/rpbi/nextage/joint_state', JointState, self.callback)
        self.sub_on = True
        rospy.loginfo('switched on remapper')
        return success, info

    def stop_callback(self, req):

        success = True
        info = ''

        if not self.sub_on:
            info = 'recieved request to stop remapper, but it is not running anyway!'
            success = False
            rospy.logerr(info)
            return success, info

        # Stop main loop
        self.sub.unregister()
        self.sub = None
        self.sub_on = False
        rospy.loginfo('switched off remapper')
        return success, info

    def callback(self, msg):
        # print(msg.name)
        cmd = []
        for i, name in enumerate(controller_joints):
            joint_index = msg.name.index(name)
            joint_position = msg.position[joint_index]
            cmd.append(joint_position)
        self.pub.publish(Float64MultiArray(data=cmd))

    def spin(self):
        rospy.spin()


def main():
    Node().spin()


if __name__ == '__main__':
    main()
