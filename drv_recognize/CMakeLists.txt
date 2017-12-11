cmake_minimum_required(VERSION 2.8.3)
project(drv_recognize)

# You only need add this to use C++ 11 features
add_compile_options(-std=c++11)

aux_source_directory(./src SRC_LIST)

set(CMAKE_BUILD_TYPE "Debug")
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
    message(STATUS "No build type selected, default to ${CMAKE_BUILD_TYPE}")
endif()

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  drv_msgs
  image_transport
  roscpp
  rospy
  tf2
  tf2_ros
)
find_package(Eigen3 REQUIRED)

set(Caffe_DIR "/home/aicrobo/video_prop_networks/lib/build-caffe-Desktop-Default")
find_package(Caffe REQUIRED)
message("Caffe DIR is ${Caffe_DIR}")
include_directories(${Caffe_INCLUDE_DIRS})
add_definitions(${Caffe_DEFINITIONS})    # ex. -DCPU_ONLY


set(OpenCV_DIR "/usr/share/OpenCV")
find_package(OpenCV REQUIRED )
message("Open CV version is ${OpenCV_VERSION}")

# Add dynamic reconfigure API
generate_dynamic_reconfigure_options(cfg/recog.cfg)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}
)

add_library (drv_recognize_lib
 src/detectcolor.cpp
 src/processdepth.cpp
 src/refinedepth.cpp
 src/transform.cpp
 
 src/detectcolor.h
 src/processdepth.h
 src/refinedepth.h
 src/transform.h
)

add_executable(drv_recognize_color src/drv_recognize_color.cpp)
target_link_libraries(drv_recognize_lib ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Caffe_LIBRARIES})
target_link_libraries (drv_recognize_color drv_recognize_lib)

add_executable(drv_recognize_hand src/drv_recognize_hand.cpp)
target_link_libraries(drv_recognize_lib ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
target_link_libraries (drv_recognize_hand drv_recognize_lib)

add_executable(drv_refine_depth src/drv_refine_depth.cpp)
target_link_libraries(drv_recognize_lib ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
target_link_libraries (drv_refine_depth drv_recognize_lib)
