#!/usr/bin/env python3
import pybullet

# Import required classes for pybullet interface
from ros_pybullet_interface.ros_node import RosNode
from ros_pybullet_interface.config import load_config
from ros_pybullet_interface.utils import UniqueDict
from ros_pybullet_interface.pybullet_instance import PybulletInstance
from ros_pybullet_interface.pybullet_visualizer import PybulletVisualizer
from ros_pybullet_interface.pybullet_visual_object import PybulletVisualObject
from ros_pybullet_interface.pybullet_dynamic_object import PybulletDynamicObject
from ros_pybullet_interface.pybullet_collision_object import PybulletCollisionObject

class Node(RosNode):

    def __init__(self):

        # Initialize node
        RosNode.__init__(self, 'ros_pybullet_interface')

        # Connect to pybullet
        self.pybullet_instance = PybulletInstance(pybullet, self)

        # Setup camera
        self.pybullet_visualizer = PybulletVisualizer(pybullet, self)

        # Collect pybullet objects
        self.pybullet_objects = UniqueDict()
        self.add_pybullet_objects('~pybullet_visual_object_config_filenames', PybulletVisualObject)
        self.add_pybullet_objects('~pybullet_dynamic_object_config_filenames', PybulletDynamicObject)
        self.add_pybullet_objects('~pybullet_collision_object_config_filenames', PybulletCollisionObject)

        # Collect robots
        self.pybullet_robots = UniqueDict()
        for config_filename in self.get_param('~pybullet_robot_config_filenames', []):
            obj = PybulletRobot(pybullet, self, load_config(config_filename))
            self.pybullet_objects[obj.name] = obj

    def add_pybullet_objects(self, parameter_name, object_type):
        for config_filename in self.get_param(parameter_name, []):
            obj = object_type(pybullet, self, load_config(config_filename))
            self.pybullet_objects[obj.name] = obj


def main():
    Node().spin()

if __name__ == '__main__':
    main()
