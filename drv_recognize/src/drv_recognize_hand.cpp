#include <ros/ros.h>

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


// Custom message
#include <drv_msgs/recognized_target.h>
#include "processdepth.h"
#include "transform.h"

using namespace std;

ros::Publisher pubStatus_;
ros::Publisher pubPose_;
ros::Publisher pubMarker_;

enum ModeType{m_wander, m_search, m_track, m_put};
int modeType_ = m_wander;

string param_running_mode = "/status/running_mode";

bool hasHand_ = false; // for output grasp info 
bool posePublished_ = false; // only works in simple mode

// Marker type
uint32_t shape = visualization_msgs::Marker::SPHERE;

// Transform frame
string base_frame_ = "base_link"; // Base frame that NVG link to
string camera_optical_frame_ = "vision_depth_optical_frame";

// Depth image temp
cv_bridge::CvImagePtr depth_image_ptr_;

void publishMarker(float x, float y, float z, std_msgs::Header header)
{
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp. See the TF tutorials for information on these.
  marker.header.frame_id = base_frame_;
  marker.header.stamp = header.stamp;
  
  // Set the namespace and id for this marker. This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "hand";
  marker.id = 0;
  
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;
  
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.15;
  marker.scale.y = 0.15;
  marker.scale.z = 0.15;
  
  marker.color.a = 0.8; // Don't forget to set the alpha!
  marker.color.r = 0.6;
  marker.color.g = 0.5;
  marker.color.b = 0.4;

  pubMarker_.publish(marker);
}

void depthCallback(const sensor_msgs::ImageConstPtr& imageDepth)
{
  if (modeType_ != m_wander)
    return;
  
  if(!(imageDepth->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1) == 0)) {
    ROS_ERROR("Hand recognize: Depth image type is wrong.");
    return;
  }
  
  depth_image_ptr_ = cv_bridge::toCvCopy(imageDepth);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drv_recognize_hand");
  
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  pnh.getParam("base_frame_id", base_frame_);
  pnh.getParam("camera_optical_frame_id", camera_optical_frame_);
  
  // Public control value
  //pubPose_ = nh.advertise<geometry_msgs::PoseStamped>("/ctrl/vision/grasp/pose", 1);
  pubStatus_ = nh.advertise<std_msgs::Int8>("/ctrl/vision/recognize/hand", 1);
  pubMarker_ = nh.advertise<visualization_msgs::Marker>("/vision/hand/marker", 1);
    
  // Object to perform transform
  Transform m_tf_;

  ros::NodeHandle cnh;
  ros::NodeHandle depth_nh(nh, "depth");
  ros::NodeHandle depth_pnh(cnh, "depth");
  image_transport::ImageTransport depth_it(depth_nh);
  image_transport::TransportHints hintsDepth("compressedDepth", ros::TransportHints(), depth_pnh);
  
  image_transport::Subscriber sub_depth = depth_it.subscribe("/vision/depth/image_rect", 1, 
                                                             depthCallback, hintsDepth);
  
  ROS_INFO("Hand detection function initialized.");
  
  ProcessDepth pd_;
  
  while (ros::ok()) {
    if (ros::param::has(param_running_mode))
      ros::param::get(param_running_mode, modeType_);
    
    if (modeType_ != m_wander) {
      // Reset the status
      continue;
    }
    
    ros::spinOnce();

    if (depth_image_ptr_ == NULL)
      continue;
    
    vector<int> bbox;
    int gesture;
    Point3f center;
    if (pd_.detectHand(depth_image_ptr_->image, bbox, center, gesture)) {
      ROS_INFO_THROTTLE(3, "Recognize hand: Found gesture %d.", gesture);
      m_tf_.getTransform(base_frame_, camera_optical_frame_);
      Eigen::Vector3f p_in(center.x, center.y, center.z);
      Eigen::Vector3f p_out;
      m_tf_.doTransform(p_in, p_out);
      publishMarker(p_out.x(), p_out.y(), p_out.z(), depth_image_ptr_->header);
    }
    
    std_msgs::Int8 flag;
    flag.data = 0;
    
    if (hasHand_)
      flag.data = gesture;
    else {
      flag.data = -1;
    }
    pubStatus_.publish(flag);
  }
  return 0;
}
