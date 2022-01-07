#!/usr/bin/env python3
import rospy
from ros_pybullet_interface.srv import MatchSimToRobot, MatchSimToRobotRequest

name = 'nextage'
topic = '/nextagea/joint_states'
srv = 'match_sim_to_robot'

rospy.wait_for_service(srv)
try:
    srv_handle = rospy.ServiceProxy(srv, MatchSimToRobot)
    req = MatchSimToRobotRequest(robot_name=name, robot_joint_state_topic=topic)
    resp = srv_handle(req)
except rospy.ServiceException as e:
    print("Service call failed: %s"%e)

if resp.success:
    print("Match successful!")
else:
    print(">>Match unsuccessful!<<")

