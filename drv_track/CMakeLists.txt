cmake_minimum_required(VERSION 2.8.3)
project(drv_track)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
    message(STATUS "No build type selected, default to ${CMAKE_BUILD_TYPE}")
endif()

find_package(Caffe REQUIRED)
include_directories(${Caffe_INCLUDE_DIRS})
add_definitions(${Caffe_DEFINITIONS})    # ex. -DCPU_ONLY

find_package(OpenCV REQUIRED
  core
  imgproc
  highgui
  objdetect
  gpu
)

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

find_package(Boost COMPONENTS system filesystem regex REQUIRED)

message("Open CV version is ${OpenCV_VERSION}")

if(CUDA_FOUND)
include_directories(${CUDA_INCLUDE_DIRS})
endif()

set(GLOG_LIB glog)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  src
)


add_library (${PROJECT_NAME}
src/helper/bounding_box.cpp
src/train/example_generator.cpp
src/helper/helper.cpp
src/helper/high_res_timer.cpp
src/helper/image_proc.cpp
src/loader/loader_alov.cpp
src/loader/loader_imagenet_det.cpp
src/loader/loader_vot.cpp
src/network/regressor.cpp
src/network/regressor_base.cpp
src/network/regressor_train.cpp
src/network/regressor_train_base.cpp
src/tracker/tracker.cpp
src/tracker/tracker_manager.cpp
src/train/tracker_trainer.cpp
src/loader/video.cpp
src/loader/video_loader.cpp
src/native/vot.cpp
src/goturn.cpp
src/utilities.cpp

src/helper/bounding_box.h
src/train/example_generator.h
src/helper/helper.h
src/helper/high_res_timer.h
src/helper/image_proc.h
src/loader/loader_alov.h
src/loader/loader_imagenet_det.h
src/loader/loader_vot.h
src/network/regressor.h
src/network/regressor_base.h
src/network/regressor_train.h
src/network/regressor_train_base.h
src/tracker/tracker.h
src/tracker/tracker_manager.h
src/train/tracker_trainer.h
src/loader/video.h
src/loader/video_loader.h
src/native/vot.h
src/goturn.h
src/utilities.h
)

# the node name can't be the same as the project name
 add_executable(drv_track_node src/drv_track.cpp)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${catkin_LIBRARIES} ${Caffe_LIBRARIES} ${Boost_LIBRARIES} ${GLOG_LIB})
target_link_libraries (drv_track_node ${PROJECT_NAME})
