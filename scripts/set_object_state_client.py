#!/usr/bin/env python3

import numpy as np
import sys
import rospy
from ros_pybullet_interface.srv import setObjectState, setObjectStateResponse

def main():
    # set the state of the object
    obj_name = "target"
    pos = [0, -0.6, 0.30]
    quat = [0, 0, 0, 1]
    lin_vel = [0.1, 0.0, 0.0]
    ang_vel = [0.1, 0.1, 0.1]
    rospy.wait_for_service('set_object_state')
    try:
        set_object_state = rospy.ServiceProxy('set_object_state', setObjectState)
        response = set_object_state(obj_name, pos, quat, lin_vel, ang_vel)
        print(response.success)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
