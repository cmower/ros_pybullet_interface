#!/usr/bin/env python3

import numpy as np
import sys
import rospy
import traceback
from ros_pybullet_interface.srv import ManualPybulletSteps, ManualPybulletStepsResponse


USAGE = """%s [please provide only one integer number]"""


def main():

    if len(sys.argv) == 2:
        num_pybullet_steps = int(sys.argv[1])
    else:
       print(USAGE % sys.argv[0])
       return 0

    rospy.wait_for_service('manual_pybullet_steps')
    try:
        manual_pybullet_steps = rospy.ServiceProxy('manual_pybullet_steps', ManualPybulletSteps)
        resp = manual_pybullet_steps(num_pybullet_steps)
        if not resp.success:
            rospy.logwarn(resp.info)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        sys.exit(1)
    except Exception:
        rospy.logerr("-"*70)
        traceback.print_exc(file=sys.stdout)
        rospy.logerr("-"*70)


if __name__ == "__main__":
    sys.exit(main())
