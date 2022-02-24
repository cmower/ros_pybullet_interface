cmake_minimum_required(VERSION 3.0.2)
project(rpbi_utils)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf2
  tf2_ros
  geometry_msgs
  std_msgs
  )

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/interpolation_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
