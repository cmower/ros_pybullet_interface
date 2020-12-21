cmake_minimum_required(VERSION 3.0.2)
project(ros_pybullet_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  tf2
  tf2_ros
  )

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

catkin_install_python(PROGRAMS
  src/ros_pybullet_interface.py
  src/test_ros_pybullet_interface.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
