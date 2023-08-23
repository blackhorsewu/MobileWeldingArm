cmake_minimum_required(VERSION 3.0.2)
project(ur_connection_manager)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

catkin_python_setup()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_service_files(
  FILES
  MoveL.srv
  MoveJ.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

# catkin_install_python(PROGRAMS scripts/ur_connection_manager.py
#                       DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

