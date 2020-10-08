cmake_minimum_required(VERSION 2.8.3)
project(wb_sol)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Find dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  sensor_msgs
  rospy
)

# Declare dependencies
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_tutorial_1
#  DEPENDS system_libi
  DEPENDS
    roscpp
    message_runtime
    rospy
)


# Include dirs.
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

catkin_python_setup()

# Compile key_in
catkin_install_python(PROGRAMS scripts/key_in.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Compile camera_control
catkin_install_python(PROGRAMS scripts/cam_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
