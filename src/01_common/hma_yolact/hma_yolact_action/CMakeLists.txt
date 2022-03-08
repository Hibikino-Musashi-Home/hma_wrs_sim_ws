cmake_minimum_required(VERSION 3.0.2)
project(hma_yolact_action)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  hma_yolact_msgs
  message_generation
  actionlib
  actionlib_msgs
)

add_action_files(
  DIRECTORY
    action    
  FILES
    Yolact.action
)

generate_messages(
  DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
    hma_yolact_msgs
    actionlib_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)