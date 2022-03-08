cmake_minimum_required(VERSION 3.0.2)
project(hma_yolact_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  hma_yolact_msgs
  hma_yolact_action
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)