# ros_pybullet_interface

This repository implements an interface between [ROS](https://www.ros.org/) and [Pybullet](https://pybullet.org/wordpress/).
In addition, several IK interfaces are provided for specific setups.

* `ros_pybullet_interface`: this package implements the basic functionality of the simulator.
* `ros_pybullet_interface_examples`: this package contains several examples.
* `rpbi_ik_interface`: several IK interfaces and supporting code for specific setups.

As of October 2021, the repository has undergone a re-vamp.
The newly re-implemented version requires some minor adjustments to config files - see class definitions in `ros_pybullet_interface/src` for documentation on what every object requires in its yaml configuration file.
The main `ros_pybullet_interface_node.py` script, and the contents of `ros_pybullet_interface/src` has been re-implemented with [ROS2](https://docs.ros.org/en/foxy/index.html) in mind.
When it becomes necessary to move to ROS2, the scripts in `ros_pybullet_interface/src` have been deisgned so they can remain untouched (apart from special cases `tf_interface.py` and `config.py`, see below), i.e. their functionality should remain the same.
The script `ros_pybullet_interface_node.py` should be the only code that will be reqiured to be re-implemented - apart from necessary restructuring of packages.
Re, the packages `ros_pybullet_interface_examples` and `rpbi_ik_interface` the suggestion for use will be to interface ROS1/ROS2 via [ros1_bridge](https://github.com/ros2/ros1_bridge).

*Special cases*: when moving to ROS2, the scripts `tf_interface.py` and `config.py` in `ros_pybullet_interface/src` will need attention.

Note, this package only supports Python3 + ROS Noetic or later.
There are custom solutions for other setups - see wiki.

# Install

1. Create catkin workspace, and `cd` into `src/`
1. `$ git clone git@github.com:cmower/ros_pybullet_interface.git`
1. `$ cd ros_pybullet_interface`
1. `$ rosdep update ; rosdep install --from-paths ./ -iry`
