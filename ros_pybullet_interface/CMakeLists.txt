cmake_minimum_required(VERSION 3.0.2)
project(ros_pybullet_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  tf2
  tf2_ros
  std_msgs
  message_generation
  )

catkin_python_setup()

add_service_files(
  FILES
  ManualPybullet.srv
  CreateDynamicObject.srv
  PybulletBodyUniqueIds.srv
  PybulletRobotJointInfo.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

catkin_install_python(PROGRAMS
  scripts/ros_pybullet_interface_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
