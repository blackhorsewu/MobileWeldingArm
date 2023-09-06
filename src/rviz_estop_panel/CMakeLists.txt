cmake_minimum_required(VERSION 3.0.2)
project(rviz_estop_panel)

# Find Qt5
find_package(Qt5 COMPONENTS Widgets REQUIRED)

# Find catkinmacros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rviz
  pluginlib
)

# Declare our catkin package
catkin_package()

# Specify locations of hearder files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Declare the C++ executable
# add_executable(rviz_estop_panel src/estop_panel.cpp)

# Use Qt5 to add the necessary include directories and compile definitions
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_library(estop_panel src/estop_panel.cpp)
target_link_libraries(estop_panel ${catkin_LIBRARIES} Qt5::Widgets)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)