#!/usr/bin/env bash

#
#      This is a script for installing the ROS-PyBullet Interface. Please
#      inspect this script before running. Run in a terminal using
#
#         $ bash install.sh
#

echo "Installing the ROS-PyBullet Interface ..."

# ==========================================================================
# Clone required repositories
cd $(catkin locate)/src

git clone https://github.com/ros-pybullet/custom_ros_tools.git
git clone https://github.com/ros-pybullet/dmp.git
git clone https://github.com/ros-pybullet/ik_ros.git
git clone https://github.com/ros-pybullet/operator_node.git
git clone https://github.com/ros-pybullet/teleop.git
git clone https://github.com/ros-pybullet/ros-keyboard.git


git clone https://github.com/ros-pybullet/ros_kortex.git
mv -v ros_kortex/kortex_description .
rm -rfv ros_kortex

git clone https://github.com/ros-pybullet/nextagea.git
mv -v nextagea/nextagea_description .
rm -rfv nextagea

git clone -b UoE_SLMC https://github.com/ros-pybullet/talos_robot.git
git clone https://github.com/ros-pybullet/exotica.git

git clone https://github.com/ros-pybullet/trac_ik.git

# ==========================================================================
# Install dependancies

# Install SDL (required by ros-keyboard)
# https://www.libsdl.org
sudo apt install libsdl2-dev libsdl2-2.0-0

# Update rosdep and install deps
rosdep update
rosdep install --from-paths ./ -iry

# ==========================================================================
# Build/source workspace
catkin build -s

echo "Completed installation of the ROS-PyBullet Interface"
echo " "
echo " >>>> please run $ source $(catkin locate)/devel/setup.bash"
echo " "
echo "Please try out any of the following examples"
echo " "
echo "roslaunch rpbi_examples basic_example_kuka_lwr.launch"
echo "roslaunch rpbi_examples basic_example_human_model.launch"
echo "roslaunch rpbi_examples basic_example_talos.launch"
echo "roslaunch rpbi_examples pybullet_objects_example.launch"
echo "roslaunch rpbi_examples soft_body.launch"
echo "roslaunch rpbi_examples basic_example_kinova.launch"
echo "roslaunch rpbi_examples basic_example_nextage.launch"
echo "roslaunch rpbi_examples lfd.launch"
echo "roslaunch rpbi_examples pybullet_rgbd_example.launch"
echo "roslaunch rpbi_examples interpolation.launch"
