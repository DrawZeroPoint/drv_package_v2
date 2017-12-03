#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <ros/ros.h>

#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/TransformStamped.h>

#include <sensor_msgs/PointCloud2.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Header.h>
#include <std_msgs/UInt16MultiArray.h>

#include <geometry_msgs/PoseStamped.h>

#include <math.h>
#include <string.h>

using namespace std;

class Transform
{
public:
  Transform();

  bool getTransform(string base_frame, string header_frame);
  
  void doTransform(Eigen::Vector3f p_in, Eigen::Vector3f &p_out);
  
private:
  ros::NodeHandle nh_;
  // These declaration must be after the node initialization
  tf2_ros::Buffer tf_buffer_;
  // This is mandatory and should be declared before while loop  
  tf2_ros::TransformListener tf_listener_;
  
  geometry_msgs::TransformStamped tf_handle_;
};

#endif // TRANSFORM_H

