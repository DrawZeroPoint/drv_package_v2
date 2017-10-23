cmake_minimum_required(VERSION 2.8.3)
project(drv_recognize)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  drv_msgs
  image_transport
  roscpp
  rospy
  tf2
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
