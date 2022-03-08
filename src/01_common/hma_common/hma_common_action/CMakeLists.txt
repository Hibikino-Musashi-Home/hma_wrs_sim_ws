cmake_minimum_required(VERSION 2.8.3)
project(hma_common_action)

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
    GraspPoseEstimation.action
    MappingObject.action
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