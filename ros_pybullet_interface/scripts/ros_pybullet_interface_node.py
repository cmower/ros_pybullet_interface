#!/usr/bin/env python3
import pybullet
from functools import partial

from rpbi.ros_node import RosNode
from rpbi.config import load_config, load_configs
from rpbi.pybullet_instance import PybulletInstance
from rpbi.pybullet_visualizer import PybulletVisualizer
from rpbi.pybullet_robot import PybulletRobot
from rpbi.pybullet_visual_object import PybulletVisualObject
from rpbi.pybullet_dynamic_object import PybulletDynamicObject
from rpbi.pybullet_collision_object import PybulletCollisionObject

from ros_pybullet_interface.msg import PybulletObject
from ros_pybullet_interface.srv import AddPybulletObject, AddPybulletObjectResponse
from cob_srvs.srv import SetString, SetStringResponse

class PybulletObjects(dict):

    def __init__(self, node):
        super().__init__()
        self.node = node

    def add(self, config, object_type):
        name = config['name']
        self[name] = object_type(pybullet, self.node, config)

    def __setitem__(self, name, obj):
        if name in self:
            raise KeyError(f'{name} already exists, pybullet objects must be given unique names!')
        super().__setitem__(name, obj)
        self.loginfo(f'added pybullet object "{name}"')

    def __delitem__(self, name):
        self[name].destroy()
        super().__delitem__(name)


class Node(RosNode):

    def __init__(self):

        # Initialize node
        super().__init__('ros_pybullet_interface')

        # Get configuration
        self.config = self.get_param('~config')

        # Connect to pybullet
        self.pybullet_instance = PybulletInstance(pybullet, self)

        # Setup camera
        self.pybullet_visualizer = PybulletVisualizer(pybullet, self)

        # Collect pybullet objects
        self.pybullet_objects = PybulletObjects(self)

        def add_list(filenames, object_type):
            for filename in filenames:
                self.pybullet_objects.add(load_config(filename), object_type)

        add_list(self.config.get('visual_objects', []), PybulletVisualObject)
        add_list(self.config.get('collision_objects', []), PybulletCollisionObject)
        add_list(self.config.get('dynamic_objects', []), PybulletDynamicObject)
        add_list(self.config.get('robots', []), PybulletRobot)

        # Start services
        self.Service('rpbi/add_pybullet_object', AddPybulletObject, service_add_pybullet_object)
        self.Service('rpbi/remove_pybullet_object', SetString, self.service_remove_pybullet_object)

        # Start pybullet
        if self.pybullet_instance.start_pybullet_after_initialization:
            self.pybullet_instance.start()


    def service_add_pybullet_object(self, req):

        success = True
        message = 'added pybullet object'

        # Get object type
        if req.pybullet_object.object_type == PybulletObject.VISUAL:
            object_type = PybulletVisualObject
        elif req.pybullet_object.object_type == PybulletObject.COLLISION:
            object_type = PybulletCollisionObject
        elif req.pybullet_object.object_type == PybulletObject.DYNAMIC:
            object_type = PybulletDynamicObject
        elif req.pybullet_object.object_type == PybulletObject.ROBOT:
            object_type = PybulletRobot
        else:
            success = False
            message = f"did not recognize object type, given '{req.pybullet_object.object_type}', expected either 0, 1, 2, 3. See PybulletObject.msg"
            self.logerr(message)
            return AddPybulletObjectResponse(success=success, message=message)

        # Add using filename (if given)
        if req.pybullet_object.filename:
            try:
                self.pybullet_objects.add(load_config(req.pybullet_object.filename), object_type)
            except Exception as err:
                success = False
                message = str(err)
                self.logerr(message)
            return AddPybulletObjectResponse(success=success, message=message)

        # Add using config string
        if req.pybullet_object.config:
            try:
                self.pybullet_objects.add(load_configs(req.pybullet_object.config), object_type)
            except Exception as err:
                success = False
                message = str(err)
                self.logerr(message)
            return AddPybulletObjectResponse(success=success, message=message)

        success = False
        message = 'failed to add pybullet object, neither filename of config was given in request!'
        return AddPybulletObjectResponse(success=success, message=message)


    def service_remove_pybullet_object(self, req):

        success = True
        message = 'removed pybullet object'
        name = req.data

        try:
            del self.pybullet_objects[name]

        except KeyError:
            success = False
            message = f'given object "{name}" does not exist!'

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
