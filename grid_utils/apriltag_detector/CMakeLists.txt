cmake_minimum_required(VERSION 3.0.2)
project(apriltag_detector)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
)


#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  cfg/dynamic_reconfigure.cfg
)

catkin_package(
  DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


# make sure configure headers are built before any node using them
#add_dependencies(controller ${PROJECT_NAME}_gencfg)
