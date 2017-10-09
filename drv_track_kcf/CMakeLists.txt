cmake_minimum_required(VERSION 2.8.3)
project(drv_track_kcf)

aux_source_directory(./src SRC_LIST)

if(NOT WIN32)
ADD_DEFINITIONS("-std=c++0x -O3")
endif(NOT WIN32)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
    message(STATUS "No build type selected, default to ${CMAKE_BUILD_TYPE}")
endif()

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  drv_msgs
  dynamic_reconfigure
  geometry_msgs
  image_geometry
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  src
  ${catkin_INCLUDE_DIRS}
)

add_executable(drv_track_kcf ${SRC_LIST})

target_link_libraries(drv_track_kcf ${OpenCV_LIBS} ${catkin_LIBRARIES})
