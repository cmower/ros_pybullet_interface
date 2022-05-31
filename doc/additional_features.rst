Additional Features
===================

GUI Controls
------------

The GUI controls can be used to perform basic interactions with the interface.
The following is an image of the GUI controls that you can expect to see when it is launched.

.. image:: images/gui-controls.png
  :width: 400
  :alt: Alternative text

There are four buttons defined, and a text box. The effect of the buttons are as follows.

* *Start*: starts the simulation clock.
* *Step*: steps the simulation clock by the given sampling frequency specified in the :ref:`main configuration <mainconfig>`
* *Stop*: stops/pauses the simulation clock.
* *Send*: sends the robot to the initial configuration as specified in its configuration file. If the initial configuration is unpsecified then this defaults to zero.

The text box displays the status of the simulation. If it is ``1``, then the simulation is running. If it is ``0`` then the simulation is paused.
Ultimately, what is displayed here is reporting the current value of the ``rpbi/status`` topic with type ``std_msgs/Int64``.

Launch
******

Add the following to your launch file.

.. code-block:: xml

		<node pkg="rpbi_utils" name="controls" type="rpbi_controls_node.py" output="screen">
		  <rosparam param="config" file="path/to/config.yaml"/>
		</node>

Configuration
*************

The configuration for the GUI controls can be added to the :ref:`main yaml configuration <mainconfig>`.
An example is shown below.

.. code-block:: yaml

		controls:
		  robot_name: "ROBOT_NAME"

The ``robot_name`` tag should be the name of the robot as specified in the pybullet object configuration.
If the configuration is not given, then the *Send* button will have no effect.

Limitations and future development
**********************************

Currently, only a single robot is supported per node.
If you want to control multiple robots then you will need to launch multiple control nodes.

In the future we aim to re-implement the controls as an RQT pluggin. If you would like to contribute this please :ref:`submit a pull request <develop>`.

Interpolation
-------------

**TODO**: Theo to add short description.

ik_ros
------

.. image:: images/ik_ros_sys_overview.png
  :width: 600
  :alt: Alternative text

The `ik_ros <https://github.com/cmower/ik_ros>`_ package is a standardized interface for inverse kinematics using ROS.
Input data (e.g. end-effector task space goals) are directed to a problem setup node, that collects the information into a single message.
The setup node then publishes a problem message at a given frequency.
A solver node, that interfaces via a standardized plugin to an IK solver, then solves the problem and publishes the target joint state.

safe_robot
----------

A low-level `ROS package <https://github.com/cmower/safe_robot>`_ for the safe operation of robots.
Easily setup with a single launch file.
The ``safe_robot_node.py`` acts as a remapper.
Target joint states are passed through several safety checks, if safe then the command is sent to the robot, otherwise they are prevented.
Possible checks

* joint position limits
* joint velocity limits
* end-effector/link box limits
* self-collision check

custom_ros_tools
----------------

The `custom_ros_tools <https://github.com/cmower/custom_ros_tools>`_ package provides a collection of generic useful tools for ROS.
The package is extensively used in the ROS-PyBullet Interface.
