#!/usr/bin/env python3
import rospy
from std_srvs.srv import SetBool

def get_srv_handle(srv_name, srv_type=SetBool):
    rospy.wait_for_service(srv_name)
    return rospy.ServiceProxy(srv_name, srv_type)

def main():

    rospy.init_node('run_teleoperation_joint_demo')

    # Grab service handles
    toggle_operator_signal = get_srv_handle('operator_node/scalenode/toggle_callback')

    # Enable demo nodes
    toggle_operator_signal(True)

    rospy.loginfo('started teleoperation demo')

if __name__ == '__main__':
    main()
