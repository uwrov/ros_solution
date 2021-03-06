cmake_minimum_required(VERSION 2.8.3)
project(wb_sol)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Find dependencies
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  geometry_msgs
  std_msgs
  sensor_msgs
  rospy
)
find_package(gazebo REQUIRED)

# Declare dependencies
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_tutorial_1
#  CATKIN_DEPENDS gazebo_ros roscpp
#  DEPENDS system_libi
  DEPENDS
    roscpp
    gazebo_ros
    message_runtime
    rospy
)

# Link to gazebo
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

# Include dirs.
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

# Compile plugin
add_library(wb_plugin plugins/wb_plugin.cc)
target_link_libraries(wb_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

catkin_python_setup()

# Compile key_in
catkin_install_python(PROGRAMS scripts/key_in.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Compile camera_control
catkin_install_python(PROGRAMS scripts/cam_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Compile the simple subscriber
add_executable(simp src/simple_sub.cc)
target_link_libraries(simp ${catkin_LIBRARIES} ${roscpp_LIBRARIES})