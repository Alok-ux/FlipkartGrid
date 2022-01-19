cmake_minimum_required(VERSION 3.0.2)
project(grid_phase2_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  dynamic_reconfigure
  actionlib_msgs
)

add_action_files(
  DIRECTORY action
  FILES bot.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/pid.cfg
)

catkin_package(
  DEPENDS
  actionlib_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
