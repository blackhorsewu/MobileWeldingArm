cmake_minimum_required(VERSION 2.8.3)
project(ugv_simulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  ugv_sdk
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Add executables
add_executable(bunker_pro_base_sim_node src/bunker_pro/src/bunker_pro_base_sim_node.cpp)
target_link_libraries(bunker_pro_base_sim_node ${catkin_LIBRARIES})