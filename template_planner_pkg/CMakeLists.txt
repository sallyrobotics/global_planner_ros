cmake_minimum_required(VERSION 3.0.2)
project(template_planner_pkg)

find_package(catkin REQUIRED COMPONENTS
  nav_core
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES template_planner_pkg
#  CATKIN_DEPENDS nav_core roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_library(template_planner_lib src/template_planner.cpp)

