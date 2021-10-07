#!/usr/bin/env python3
import rospy
import numpy
from ros_pybullet_interface.srv import CreateDynamicObject

def main():
    args = (
        'test_object',                                              # name
        0.3,                                                        # base mass
        'box',                                                      # object_type
        numpy.random.uniform(0, 1, size=3).tolist() + [1],          # rgba_color
        [1,1,1],                                                    # mesh_scale, not used
        0.1,                                                        # radius, not used
        0.1,                                                        # height, not used
        [0.2, 0.1, 0.1],                                            # half_extends
        [20.0, 40.0, 40.0],                                         # init_base_orient_eulerXYZ
        [-0.6, 0.6, 0.4],                                           # init_position
        [0.2, -0.3, 0.1],                                            # init_linear_velocity
        [10, 1, 0.4],                                               # init_angular_velocity
        1.0,
        0.0,
        0.0,
        0.0,
        0.045,
        0.03,
        2000.0,
        0.6,
    )

    service = 'create_dynamic_object'
    rospy.wait_for_service(service)
    try:
        handle = rospy.ServiceProxy(service, CreateDynamicObject)
        handle(*args)
    except rospy.ServiceException as e:
        print("Service call failed:", e)

if __name__ == '__main__':
    main()
