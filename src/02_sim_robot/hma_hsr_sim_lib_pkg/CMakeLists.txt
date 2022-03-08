cmake_minimum_required(VERSION 2.8.3)
project(hma_hsr_sim_lib_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)