cmake_minimum_required(VERSION 3.0.2)
project(hma_hsr_nav_action)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  actionlib
  actionlib_msgs
  hma_hsr_msgs
)

add_action_files(
  DIRECTORY
    action    
  FILES
    MotionSynthesis.action
    OmniPathFollower.action
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
    actionlib_msgs
    hma_hsr_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)