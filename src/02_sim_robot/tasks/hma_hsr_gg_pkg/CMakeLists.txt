cmake_minimum_required(VERSION 3.0.2)
project(hma_hsr_gg_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  actionlib
  actionlib_msgs
)

add_action_files(
  DIRECTORY
    action
  FILES
    DetectDrivableArea.action
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
    actionlib_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)