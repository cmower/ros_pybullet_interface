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
  setObjectState.srv
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
  scripts/ros_rbdl_IK_interace_node.py
  scripts/ros_traj_consum_interpol_interface_node.py
  scripts/teleop_example_node.py
  scripts/test_ros_pybullet_interface.py
  scripts/test_ros_rbdlIK_interface.py
  scripts/test_ros_interpol_interface.py
  scripts/set_object_state_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
