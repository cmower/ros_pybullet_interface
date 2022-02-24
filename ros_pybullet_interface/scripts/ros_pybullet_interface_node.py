#!/usr/bin/env python3
import pybullet
from functools import partial

from rpbi.ros_node import RosNode
from rpbi.config import load_config
from rpbi.pybullet_instance import PybulletInstance
from rpbi.pybullet_visualizer import PybulletVisualizer
from rpbi.pybullet_robot import PybulletRobot
from rpbi.pybullet_visual_object import PybulletVisualObject
from rpbi.pybullet_dynamic_object import PybulletDynamicObject
from rpbi.pybullet_collision_object import PybulletCollisionObject

from cob_srvs.srv import SetString, SetStringResponse


class Node(RosNode):

    def __init__(self):

        # Initialize node
        RosNode.__init__(self, 'ros_pybullet_interface')

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
            self.add_pybullet_object(config_filename, object_type)


    def add_pybullet_object(self, config_filename, object_type):
        added_object = True
        config = load_config(config_filename)
        name = config['name']
        if name not in self.pybullet_objects.keys():
            self.pybullet_objects[name] = object_type(pybullet, self, load_config(config_filename))
            self.loginfo(f'added object "{name}"')
        else:
            added_object = False
            self.logerr(f'unable to add object "{name}"')
        return added_object


    def service_add_pybullet_object(self, req, object_type):

        success = True

        try:

            # Load config
            config_filename = req.data

            # Create visual object
            if not self.add_pybullet_object(config_filename, object_type):
                raise KeyError("object already exists")

            message = f'created {object_type.__name__}'

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
            if object_name in self.pybullet_objects.keys():
                self.pybullet_objects[object_name].destroy()
                del self.pybullet_objects[object_name]
                message = 'removed Pybullet object'
            else:
                success = False
                message = f'given object "{object_name}" does not exist!'
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
