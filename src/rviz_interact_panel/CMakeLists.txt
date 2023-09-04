cmake_minimum_required(VERSION 3.0.2)
project(rviz_interact_panel)

# Find Qt5
find_package(Qt5 COMPONENTS Core Gui Widgets Concurrent REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  rviz
  pluginlib
  roscpp
  std_msgs
  rospy
  welding_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy welding_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Use Qt5 to add the necessary include directories and compile definitions
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Build Interact Panel
add_library(interact_panel src/interact_panel.cpp)
target_link_libraries(interact_panel ${catkin_LIBRARIES} Qt5::Widgets Qt5::Concurrent)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)