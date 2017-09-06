cmake_minimum_required(VERSION 2.8.3)
project(drv_action)

aux_source_directory(./src SRC_LIST)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
    message(STATUS "No build type selected, default to ${CMAKE_BUILD_TYPE}")
endif()

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  cv_bridge
  drv_msgs
  dynamic_reconfigure
  geometry_msgs
  image_geometry
  image_transport
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  CATKIN_DEPENDS
  actionlib
  actionlib_msgs
  drv_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(drv_action ${SRC_LIST})
target_link_libraries(drv_action ${catkin_LIBRARIES})

add_dependencies(
  drv_action
  ${drv_msgs_EXPORTED_TARGETS}
)
