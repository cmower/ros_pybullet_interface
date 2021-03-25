#!/usr/bin/env python

from __future__ import print_function
import numpy as np
import sys
import rospy
from ros_pybullet_interface.srv import *


def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":

    # set the state of the object
    pos = [-1, 0, 0.1, 0, 0, 0, 1]
    vel = [0.8, 0, 0, 0, 0, 0]
    rospy.wait_for_service('set_object_state')
    try:
        set_object_state = rospy.ServiceProxy('set_object_state', setObjectState)
        response = set_object_state(pos, vel)
        print(response.success)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

