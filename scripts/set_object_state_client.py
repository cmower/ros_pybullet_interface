#!/usr/bin/env python3

import numpy as np
import sys
import rospy
from ros_pybullet_interface.srv import setObjectState, setObjectStateResponse


USAGE = """%s please provide \n
a 3D vector for the pos in form of  \[ x,y,z\] and \n
a 4D vector for the orientation (quaternion) in form of \[ q1,q2,q3, w\] and \n
a 3D vector for the linear velocity in form of \[ dx,dy,dz\] and \n
a 3D vector for the angular velocity in form of \[ a,b,c\] and \n
the name of the object in form of a string ]"""


def main():

    print( len(sys.argv))
    if len(sys.argv) >= 1:
        pos_arg = sys.argv[1]
        pos = list(map(float, pos_arg.strip('[]').split(',')))
        print("The provide position is: ", pos)
        #
        quat_arg = sys.argv[2]
        quat = list(map(float, quat_arg.strip('[]').split(',')))
        print("The provide orientation is: ", quat)
        #
        lin_vel_arg = sys.argv[3]
        lin_vel = list(map(float, lin_vel_arg.strip('[]').split(',')))
        print("The provide linear velocity is: ", lin_vel)
        #
        ang_vel_arg = sys.argv[4]
        ang_vel = list(map(float, ang_vel_arg.strip('[]').split(',')))
        print("The provide angular velocity is: ", ang_vel)
        #
        obj_name = str(sys.argv[5])
        print("The provide object name is: ", obj_name)

        setObjState(pos, quat, lin_vel, ang_vel, obj_name)
    else:
       print(USAGE % sys.argv[0])
       return 0

def setObjState(pos, quat, lin_vel, ang_vel, obj_name ):

    # set the state of the object
    # obj_name = "target"
    # pos = [0.0, 0, 0.60]
    # quat = [0, 0, 0, 1]
    # lin_vel = [0.0, 0.0, 0.0]
    # ang_vel = [0.0, 0.0, 0.0]
    rospy.wait_for_service('set_object_state')
    try:
        set_object_state = rospy.ServiceProxy('set_object_state', setObjectState)
        response = set_object_state(obj_name, pos, quat, lin_vel, ang_vel)
        print(response.success)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)



if __name__ == "__main__":
    sys.exit(main())
