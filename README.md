# ros_pybullet_interface

Tutorial slides found [here](https://docs.google.com/presentation/d/1c7aYdl0kzYztaJyFgqGP7S1EfMuim9CGwq5VjEvhiQ8/edit?usp=sharing).

# Dependancies
* Ubuntu 20.04 (ROS Noetic)
* Python 3
* [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/)
* [rosinstall](http://wiki.ros.org/rosinstall)
* [rosdep](http://wiki.ros.org/rosdep)
* [ROS](http://wiki.ros.org/Installation)

# Install

1. [Create a catkin workspace](https://catkin-tools.readthedocs.io/en/latest/quick_start.html#initializing-a-new-workspace) or use an existing workspace. [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/) is the preferred build system.
2. `cd` to the `src` directory of your catkin workspace.
3. Clone this repository: `$ git clone https://github.com/cmower/ros_pybullet_interface.git`
4. Install source dependencies: `$ rosinstall . --catkin --nobuild`
5. Install binary dependencies: `$ rosdep update ; rosdep install --from-paths ./ -iry`
6. Compile the workspace: `$ catkin build -s`
7. Source the workspace: `$ source $(catkin locate)/devel/setup.bash`
