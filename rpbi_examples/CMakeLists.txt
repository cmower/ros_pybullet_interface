cmake_minimum_required(VERSION 3.0.2)
project(rpbi_examples)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  ros_pybullet_interface
  ik_ros_examples
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/basic_robot_example_node.py
  scripts/pybullet_objects_example_node.py
  scripts/orbit_node.py
  scripts/run_teleop.py
  scripts/run_lfd.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY configs launch robots DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/)
