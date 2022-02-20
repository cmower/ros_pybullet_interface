#!/usr/bin/env python3
import pybullet
from functools import partial

from ros_pybullet_interface.ros_node import RosNode
from ros_pybullet_interface.config import load_config
from ros_pybullet_interface.tf_interface import TfInterface
from ros_pybullet_interface.pybullet_instance import PybulletInstance
from ros_pybullet_interface.pybullet_visualizer import PybulletVisualizer
from ros_pybullet_interface.pybullet_robot import PybulletRobot
from ros_pybullet_interface.pybullet_visual_object import PybulletVisualObject
from ros_pybullet_interface.pybullet_dynamic_object import PybulletDynamicObject
from ros_pybullet_interface.pybullet_collision_object import PybulletCollisionObject

from cob_srvs.srv import SetString, SetStringResponse


class Node(RosNode):

    def __init__(self):

        # Initialize node
        RosNode.__init__(self, 'ros_pybullet_interface')

        # Setup tf interface
        self.tf = TfInterface()

        # Connect to pybullet
        self.pybullet_instance = PybulletInstance(pybullet, self)

        # Setup camera
        self.pybullet_visualizer = PybulletVisualizer(pybullet, self)

        # Collect pybullet objects
        self.pybullet_objects = {}
        self.add_pybullet_objects('~pybullet_visual_object_config_filenames', PybulletVisualObject)
        self.add_pybullet_objects('~pybullet_dynamic_object_config_filenames', PybulletDynamicObject)
        self.add_pybullet_objects('~pybullet_collision_object_config_filenames', PybulletCollisionObject)
        self.add_pybullet_objects('~pybullet_robot_config_filenames', PybulletRobot)

        # Start services
        self.Service('rpbi/add_pybullet_visual_object', SetString, partial(self.service_add_pybullet_object, object_type=PybulletVisualObject))
        self.Service('rpbi/add_pybullet_collision_object', SetString, partial(self.service_add_pybullet_object, object_type=PybulletCollisionObject))
        self.Service('rpbi/add_pybullet_dynamic_object', SetString, partial(self.service_add_pybullet_object, object_type=PybulletDynamicObject))
        self.Service('rpbi/add_pybullet_robot', SetString, partial(self.service_add_pybullet_object, object_type=PybulletRobot))
        self.Service('rpbi/remove_pybullet_object', SetString, self.service_remove_pybullet_object)

        # Start pybullet
        if self.pybullet_instance.start_pybullet_after_initialization:
            self.pybullet_instance.start()

    def add_pybullet_objects(self, parameter_name, object_type):
        for config_filename in self.get_param(parameter_name, []):
            obj = object_type(pybullet, self, load_config(config_filename))
            self.pybullet_objects[obj.name] = obj

    def service_add_pybullet_object(self, req, object_type):

        success = True

        try:

            # Load config
            config_filename = req.data
            config = load_config(config_filename)

            # Create visual object
            obj = object_type(pybullet, self, config)
            self.pybullet_objects[obj.name] = obj
            message = f'created {object_type.__name__} named "{obj.name}"'

        except Exception as e:
            success = False
            message = f'failed to create {object_type.__name__}, exception: ' + str(e)

        # Log message
        if success:
            self.loginfo(message)
        else:
            self.logerr(message)

        return SetStringResponse(success=success, message=message)

    def service_remove_pybullet_object(self, req):

        success = True

        try:
            object_name = req.data
            self.pybullet_objects[object_name].destroy()
            del self.pybullet_objects[object_name]
            message = 'removed Pybullet object'
        except Exception as e:
            success = False
            message = 'failed to remove Pybullet object, exception: ' + str(e)

        # Log message
        if success:
            self.loginfo(message)
        else:
            self.logerr(message)

        return SetStringResponse(message=message, success=success)


def main():
    Node().spin()

if __name__ == '__main__':
    main()
