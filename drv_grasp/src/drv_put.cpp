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

/* The graspable area of robot left arm,
 * relative to base_link, in meter */
float x_min_ = 0.4;
float x_max_ = 0.6;
float y_min_ = 0.2;
float y_max_ = 0.35;
float tolerance_ = 0.03;

// Whether publish pose for once
bool pubPoseOnce_ = false;

bool offsetNeedPub_ = true;



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
  
  putPubStatus_ = nh.advertise<std_msgs::Int8>("status/put/feedback", 1);
  putPubMarker_ = nh.advertise<visualization_msgs::Marker>("/vision/put/marker", 1);
  
  // Object to perform transform
  Transform m_tf_;
  // Object to perform obstacle detect
  ObstacleDetect m_od_(use_od_, base_frame_, base_to_ground_,
                       table_height_, table_area_);

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
      continue;
    }
    
    ros::spinOnce();
    
    std_msgs::Int8 flag;
    flag.data = 0;
    
    if (hasPutPlan_ && !posePublished_)
      flag.data = 1;
    if (hasPutPlan_ && posePublished_) {
      // Force pub_pose_once_ = true to prevent the pose being published twice
      pubPoseOnce_ = true;
      flag.data = 2;
    }
    
    putPubStatus_.publish(flag);
  }
  
  return 0;
}
