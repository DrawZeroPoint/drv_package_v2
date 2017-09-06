cmake_minimum_required(VERSION 2.8.3)
project(drv_face_client)

aux_source_directory(./src SRC_LIST)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
    message(STATUS "No build type selected, default to ${CMAKE_BUILD_TYPE}")
endif()

find_package(OpenCV REQUIRED)


find_package(catkin REQUIRED COMPONENTS
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

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(drv_face_client ${SRC_LIST})

target_link_libraries(drv_face_client ${catkin_LIBRARIES} ${OpenCV_LIBS})
