cmake_minimum_required(VERSION 3.0.2)
project(grid_phase2_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  dynamic_reconfigure
  actionlib_msgs
  message_generation
)

add_action_files(
  DIRECTORY action
  FILES bot.action
)

add_message_files(
  FILES
  Goal.msg
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
    DEPENDS
    actionlib_msgs
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
