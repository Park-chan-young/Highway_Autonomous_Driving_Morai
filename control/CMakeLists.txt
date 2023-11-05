cmake_minimum_required(VERSION 3.0.2)
project(control)

SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  tf2_msgs
  geometry_msgs
  message_generation
  morai_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs tf tf2_msgs geometry_msgs message_runtime
  morai_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(modules
  src/pid_control.cpp
  src/velocity_planning.cpp
  src/pure_pursuit.cpp
)

add_executable(control_node src/pure_pursuit.cpp)
target_link_libraries(control_node
  modules
  ${catkin_LIBRARIES}
)