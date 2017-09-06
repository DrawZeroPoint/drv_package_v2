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
#cmake_minimum_required(VERSION 2.8.3)
#project(drv_recognize)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#aux_source_directory(./src SRC_LIST)

#if (NOT CMAKE_BUILD_TYPE)
#    set(CMAKE_BUILD_TYPE "Debug")
#    message(STATUS "No build type selected, default to ${CMAKE_BUILD_TYPE}")
#endif()

#set(PYTHON_INCLUDE_DIR "/usr/include/python2.7")
#set(CAFFE_INCLUDE_DIR "/home/aicrobo/py-faster-rcnn/caffe-fast-rcnn/distribute/include")
#set(CAFFE_LIBRARY_DIR "/home/aicrobo/py-faster-rcnn/caffe-fast-rcnn/distribute/lib")

#find_package(Boost COMPONENTS system filesystem regex REQUIRED)

#find_package(catkin REQUIRED COMPONENTS
#  cv_bridge
#  drv_msgs
#  image_transport
#  roscpp
#  rospy
#  tf2
#)

#find_package(OpenCV REQUIRED)
#find_package(CUDA REQUIRED)

#catkin_package()

#include_directories(${catkin_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIR} ${CAFFE_INCLUDE_DIR})
#link_directories(${CAFFE_LIBRARY_DIR})

#add_executable(${PROJECT_NAME} ${SRC_LIST} /home/aicrobo/py-faster-rcnn/lib/nms/nms_kernel.cu)
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Caffe_LIBRARIES} boost_system python2.7 glog)
