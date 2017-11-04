cmake_minimum_required(VERSION 2.8.3)
project(drv_recognize)

aux_source_directory(./src SRC_LIST)

set(CMAKE_BUILD_TYPE "Debug")
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
    message(STATUS "No build type selected, default to ${CMAKE_BUILD_TYPE}")
endif()

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  drv_msgs
  image_transport
  roscpp
  rospy
  tf2
)

find_package(Caffe REQUIRED)
message("Caffe DIR is ${Caffe_DIR}")
include_directories(${Caffe_INCLUDE_DIRS})
add_definitions(${Caffe_DEFINITIONS})    # ex. -DCPU_ONLY


set(OpenCV_DIR "/usr/share/OpenCV")
find_package(OpenCV REQUIRED )
message("Open CV version is ${OpenCV_VERSION}")

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(drv_recognize_color ${SRC_LIST})
target_link_libraries(drv_recognize_color ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Caffe_LIBRARIES})
