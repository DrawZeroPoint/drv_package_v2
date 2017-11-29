#include <ros/ros.h>

#include <geometry_msgs/TransformStamped.h>

#include <math.h>
#include <string.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Header.h>
#include <std_msgs/UInt16MultiArray.h>
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/PoseStamped.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <image_geometry/pinhole_camera_model.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>

// PCL-ROS
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

// PCL
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>

// Custom message
#include <drv_msgs/recognized_target.h>
#include <drv_msgs/target_info.h>
#include "makeplan.h"
#include "getsourcecloud.h"
#include "obstacledetect.h"
#include "transform.h"
#include "utilities.h"

using namespace std;

// Publishers
ros::Publisher putPubStatus_;
ros::Publisher putPubPose_;
ros::Publisher putPubMarker_;
/* This location will be published if the target is out of range
   This describe the offset x, y value to be adjusted, z and
   orientation are not used */
ros::Publisher putPubLocation_;

// Status
enum ModeType{m_wander, m_search, m_track, m_put};
int modeType_ = m_wander;

string param_running_mode = "/status/running_mode";

bool hasPutPlan_ = false; // for output grasp info 
bool posePublished_ = false; // only works in simple mode
// Whether publish pose for once
bool pubPoseOnce_ = false;
bool offsetNeedPub_ = true;

// Marker type
uint32_t shape = visualization_msgs::Marker::ARROW;

// Transform frame
string map_frame_ = "map";
string base_frame_ = "base_link"; // Base frame that NVG link to
string camera_optical_frame_ = "vision_depth_optical_frame";

// Obstacle avoidance params
bool use_od_ = true;
float base_to_ground_ = 0.465;
float table_height_ = 0.9; // error: +-0.15m
float table_area_ = 0.01; // m^2
// Object grasp center to bottom
float center_to_bottom_ = 0.05;

/* The graspable area of robot left arm,
 * relative to base_link, in meter */
float x_min_ = 0.4;
float x_max_ = 0.6;
float y_min_ = 0.2;
float y_max_ = 0.35;
float tolerance_ = 0.03;

// Publish servo initial position
ros::Publisher servoPub_;
bool servo_init_ = false;

void publishMarker(geometry_msgs::PoseStamped pose)
{
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp. See the TF tutorials for information on these.
  marker.header.frame_id = pose.header.frame_id;
  marker.header.stamp = pose.header.stamp;
  
  // Set the namespace and id for this marker. This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "put";
  marker.id = 0;
  
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;
  
  // Set the pose of the marker, which is a full 6DOF pose
  // relative to the frame/time specified in the header
  marker.points.resize(2);
  // The point at index 0 is assumed to be the start point,
  // and the point at index 1 is assumed to be the end.
  marker.points[0].x = pose.pose.position.x;
  marker.points[0].y = pose.pose.position.y;
  marker.points[0].z = pose.pose.position.z;
  
  marker.points[1].x = pose.pose.position.x;
  marker.points[1].y = pose.pose.position.y;
  marker.points[1].z = pose.pose.position.z + 0.15; // arrow height = 0.15m

  // scale.x is the shaft diameter, and scale.y is the head diameter.
  // If scale.z is not zero, it specifies the head length.
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.01;
  marker.scale.y = 0.015;
  marker.scale.z = 0.04;
  
  // Set the color -- be sure to set alpha as non-zero value!
  // Use yellow to separate from grasp marker
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;
  
  putPubMarker_.publish(marker);
}

void pubServo(int pitch_angle, int yaw_angle, int power)
{
  std_msgs::UInt16MultiArray array;
  array.data.push_back(pitch_angle);
  array.data.push_back(yaw_angle);
  // The basic speed is 10 (0~255)
  array.data.push_back(10 * power);
  servoPub_.publish(array);
}

void objInfoCallback(const drv_msgs::target_infoConstPtr &info)
{
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drv_put");
  
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  pnh.getParam("base_frame_id", base_frame_);
  pnh.getParam("camera_optical_frame_id", camera_optical_frame_);
  pnh.getParam("pub_pose_once_id", pubPoseOnce_);
  
  // Get left arm movable area params
  pnh.getParam("left_arm_x_min", x_min_);
  pnh.getParam("left_arm_y_min", y_min_);
  pnh.getParam("left_arm_x_max", x_max_);
  pnh.getParam("left_arm_y_max", y_max_);
  
  // Public control value
  putPubPose_ = nh.advertise<geometry_msgs::PoseStamped>("/ctrl/vision/put/pose", 1);
  putPubLocation_ = nh.advertise<geometry_msgs::PoseStamped>("/ctrl/vision/put/location", 1);
  
  putPubStatus_ = nh.advertise<std_msgs::Int8>("/vision/status/put/feedback", 1);
  putPubMarker_ = nh.advertise<visualization_msgs::Marker>("/vision/put/marker", 1);
  
  // Subscribe info for putting object
  ros::Subscriber sub_obj = nh.subscribe<drv_msgs::target_info>("/vision/target_info", 1, objInfoCallback);

  // Object to perform obstacle detect
  float grasp_area_x = (x_min_ + x_max_)/2;
  float grasp_area_y = (y_min_ + y_max_)/2;
  ObstacleDetect m_od_(use_od_, base_frame_, base_to_ground_, table_height_, 
                       table_area_, grasp_area_x, grasp_area_y, tolerance_);

  ROS_INFO("Put planning function initialized.");
  
  while (ros::ok()) {
    if (ros::param::has(param_running_mode))
      ros::param::get(param_running_mode, modeType_);
    
    if (modeType_ != m_put) {
      // Reset the status
      hasPutPlan_ = false;
      posePublished_ = false;
      offsetNeedPub_ = true;
      pubPoseOnce_ = false;
      servo_init_ = false;
      continue;
    }
    
    ros::spinOnce();
    
    if (!servo_init_) {
      pubServo(60, 90, 2);
      servo_init_ = true;
    }
    
    geometry_msgs::PoseStamped put_pose;
    geometry_msgs::PoseStamped ref_pose;
    m_od_.setZOffset(center_to_bottom_);
    if (m_od_.detectPutTable(put_pose, ref_pose, offsetNeedPub_)) {
      // Table found
      if (offsetNeedPub_) {
        putPubLocation_.publish(put_pose);
        publishMarker(ref_pose);
        hasPutPlan_ = true;
        posePublished_ = false;
      }
      else {
        putPubPose_.publish(put_pose);
        publishMarker(put_pose);
        hasPutPlan_ = true;
        posePublished_ = true;
      }
      // Table not found, turn head to the next direction and search
      
    }
    
    std_msgs::Int8 flag;
    flag.data = -1; // Default return failed
    
    if (!hasPutPlan_)
      flag.data = -1;
    if (hasPutPlan_ && !posePublished_)
      flag.data = 0;
    if (hasPutPlan_ && posePublished_) {
      // Force pub_pose_once_ = true to prevent the pose being published twice
      pubPoseOnce_ = true;
      flag.data = 1;
    }
    
    putPubStatus_.publish(flag);
  }
  
  return 0;
}
