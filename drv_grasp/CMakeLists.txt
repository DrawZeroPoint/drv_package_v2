cmake_minimum_required(VERSION 2.8.3)
project(drv_grasp)

#Comment this if don't want debug
set(CMAKE_BUILD_TYPE Debug)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
    message(STATUS "No build type selected, default to ${CMAKE_BUILD_TYPE}")
endif()

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  drv_msgs
  dynamic_reconfigure
  geometry_msgs
  gpd #
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

set(OpenCV_DIR "/usr/share/OpenCV")
find_package(OpenCV REQUIRED)

find_package(PCL 1.7 REQUIRED)

find_package(Caffe REQUIRED)
message("Caffe DIR is ${Caffe_DIR}")
include_directories(${Caffe_INCLUDE_DIRS})
add_definitions(${Caffe_DEFINITIONS})    # ex. -DCPU_ONLY

catkin_package()

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_library (drv_lib
src/fetchrgbd.cpp
src/getsourcecloud.cpp
src/obstacledetect.cpp
src/transform.cpp
src/utilities.cpp

src/fetchrgbd.h
src/getsourcecloud.h
src/obstacledetect.h
src/transform.h
src/utilities.h
)

add_executable(drv_grasp src/drv_grasp.cpp)
target_link_libraries(drv_lib ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Caffe_LIBRARIES})
target_link_libraries (drv_grasp drv_lib)

add_executable(drv_put src/drv_put.cpp)
target_link_libraries(drv_lib ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Caffe_LIBRARIES})
target_link_libraries (drv_put drv_lib)

add_executable(drv_frame_service src/drv_frame_service.cpp)
target_link_libraries(drv_lib ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Caffe_LIBRARIES})
target_link_libraries (drv_frame_service drv_lib)
