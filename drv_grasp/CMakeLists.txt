cmake_minimum_required(VERSION 2.8.3)
project(drv_grasp)

#Comment this if don't want debug
set(CMAKE_BUILD_TYPE Debug)

aux_source_directory(./src SRC_LIST)

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

set(OpenCV_DIR "/usr/share/OpenCV")
find_package(OpenCV REQUIRED)

find_package(PCL 1.7 REQUIRED)

catkin_package()

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(drv_grasp ${SRC_LIST})

target_link_libraries(drv_grasp ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
