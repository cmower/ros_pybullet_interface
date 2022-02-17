cmake_minimum_required(VERSION 3.0.2)
project(ros_pybullet_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
)

catkin_python_setup()

# add_service_files(
#   FILES
#   Service1.srv
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
