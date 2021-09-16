cmake_minimum_required(VERSION 3.0.2)
project(straight_planner)

find_package(catkin REQUIRED COMPONENTS
  nav_core
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES straight_planner
#  CATKIN_DEPENDS nav_core roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_library(straight_planner_lib src/straight_planner.cpp)

