#!/usr/bin/env python3

import numpy as np
import sys
import rospy
from ros_pybullet_interface.srv import manualPybulletSteps, manualPybulletStepsResponse


def usage():
    return "%s please provide only one integer number"%sys.argv[0]


if __name__ == "__main__":

    # set the number of pybullet steps
    num_pybullet_steps = 0

    if len(sys.argv) == 2:
        num_pybullet_steps = int(sys.argv[1])
    else:
       print(usage())
       sys.exit(1)

    rospy.wait_for_service('manual_pybullet_steps')
    try:
        manual_pybullet_steps = rospy.ServiceProxy('manual_pybullet_steps', manualPybulletSteps)
        response = manual_pybullet_steps(num_pybullet_steps)
        print(response.success)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
