.. _examples:

Examples
========

The examples for the ROS-PyBullet Interface are collected in a dedicated ROS package `rpbi_examples <https://github.com/cmower/ros_pybullet_interface/tree/main/rpbi_examples>`_.

Basic Examples
--------------

The basic examples simply demonstrate the current robots that can be loaded into PyBullet out-of-the-box.
Each example loads the given robot, and `a node <https://github.com/cmower/ros_pybullet_interface/blob/main/rpbi_examples/scripts/basic_robot_example_node.py>`_ that generates a standardized motion on the robot.
Some of the basic examples demonstrate different features of the library (e.g. recording a video, loading a URDF from the ROS parameter ``robot_description``).
The following list links to the launch file for all the currently available basic examples.

* `Kuka LWR <https://github.com/cmower/ros_pybullet_interface/blob/main/rpbi_examples/launch/basic_example_kuka_lwr.launch>`_
* `Talos <https://github.com/cmower/ros_pybullet_interface/blob/main/rpbi_examples/launch/basic_example_talos.launch>`_
* `Kinova <https://github.com/cmower/ros_pybullet_interface/blob/main/rpbi_examples/launch/basic_example_kinova.launch>`_
* `Human model <https://github.com/cmower/ros_pybullet_interface/blob/main/rpbi_examples/launch/basic_example_human_model.launch>`_

The basic examples are launch as follows.
  
.. code-block::

   $ roslaunch basic_example_[NAME].launch


Human Interaction
-----------------

Adding/removing PyBullet Objects Pragmatically
----------------------------------------------

Learning from demonstration
---------------------------

RGBD Sensor
-----------

Teleoperation
-------------


