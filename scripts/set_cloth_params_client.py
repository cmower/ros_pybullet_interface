#!/usr/bin/env python3

import numpy as np
import sys
import rospy
from ros_pybullet_interface.srv import setClothConstraintParams, setClothConstraintParamsResponse


USAGE = """%s please provide \n
a 3D vector for the delta pos between the cloth and the robot end effector,
w.r.t the cloth in form of  \[ x,y,z\] and \n
a scalar for the erp of the constraint in form of a \n
a scalar for the maximum force of the constraint in form of b \n ]"""


def main():

    print(len(sys.argv))
    try:
        pos_arg = sys.argv[1]
        delta_pos = list(map(float, pos_arg.strip('[]').split(',')))
        print("The provide delta_pos is: ", delta_pos)
        #
        erp = sys.argv[2]
        print("The provide erp is: ", erp)
        #
        max_force = sys.argv[3]
        print("The provide maximum force is: ", max_force)
        #
        setClothParams(delta_pos, erp, max_force)
    except Exception:
        print(USAGE % sys.argv[0])
        return 0


def setClothParams(ChildPivotPt, erp, maxForce):
    rospy.wait_for_service('set_cloth_constraint_parameters')
    try:
        set_cloth_constraint_parameters = rospy.ServiceProxy(
            'set_cloth_constraint_parameters', setClothConstraintParams)
        response = set_cloth_constraint_parameters(ChildPivotPt, float(erp), float(maxForce))
        print(response.success)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == "__main__":
    sys.exit(main())
