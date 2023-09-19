cmake_minimum_required(VERSION 2.8.3)
project(bunker_pro)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  nav_msgs
  sensor_msgs
)

## Declare catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy tf nav_msgs sensor_msgs
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(bunker_pro_base_sim_node src/bunker_pro/src/bunker_pro_base_sim_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(bunker_pro_base_sim_node ${catkin_LIBRARIES})
