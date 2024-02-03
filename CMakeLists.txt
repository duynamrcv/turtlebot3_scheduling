cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_scheduling)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  message_generation
)

catkin_python_setup()

add_service_files(
  FILES
  NextPose.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES turtlebot3_scheduling
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/control_node.py
  src/scheduling_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)