cmake_minimum_required(VERSION 2.8.3)
project(drv_face_service)

aux_source_directory(./src SRC_LIST)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
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
)

find_package(Boost COMPONENTS system filesystem regex REQUIRED)

find_package(Caffe REQUIRED)
message("Caffe DIR is ${Caffe_DIR}")
# set(Caffe_INCLUDE_DIRS "/home/aicrobo/Downloads/caffe/cmake_build/include") this works!
include_directories(${Caffe_INCLUDE_DIRS})
add_definitions(${Caffe_DEFINITIONS})    # ex. -DCPU_ONLY

set(OpenCV_DIR "/usr/share/OpenCV")
find_package(OpenCV REQUIRED )
message("Open CV version is ${OpenCV_VERSION}")

if(CUDA_FOUND)
include_directories(${CUDA_INCLUDE_DIRS})
endif()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Caffe_LIBRARIES} ${Boost_LIBRARIES})
