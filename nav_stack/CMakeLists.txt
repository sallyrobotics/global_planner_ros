cmake_minimum_required(VERSION 3.0.2)
project(nav_stack)

find_package(catkin REQUIRED COMPONENTS
  move_base
  nav_msgs
  sensor_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nav_stack
#  CATKIN_DEPENDS move_base nav_msgs sensor_msgs tf
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

