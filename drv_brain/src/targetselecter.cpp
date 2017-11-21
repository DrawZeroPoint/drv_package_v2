#include "targetselecter.h"
#include <string>

TargetSelecter::TargetSelecter()
{
  targetNeedPub_ = false;
  ALPubROI_ = nh.advertise<drv_msgs::recognized_target>("search/recognized_target", 1);
  ALSubROI_ = nh.subscribe<std_msgs::Float32MultiArray>("/jarvis/roi", 1, &TargetSelecter::roiCallback, this);
  
  // Sub rectangle bbox from image_view2 (developed by jsk)
  std::string roi_str = "/vision/rgb/image_rect_color/screenrectangle";
  subImageView2ROI_ = nh.subscribe<geometry_msgs::PolygonStamped>(roi_str, 1, &TargetSelecter::IVROICallback, this);
  
  // Sub point from image_view2, only for cancel the target
  std::string p_str = "/vision/rgb/image_rect_color/screenpoint";
  subImageView2P_ = nh.subscribe<geometry_msgs::PointStamped>(p_str, 1, &TargetSelecter::IVPCallback, this);
}

void TargetSelecter::publishOnceIfTargetSelected(bool &is_tgt_set, bool &is_tgt_found)
{
  if (targetNeedPub_) {
    std::string param_target_set = "/comm/param/control/target/is_set";
    ros::param::set(param_target_set, true);
    is_tgt_set = true;
    ALPubROI_.publish(rt_);
    targetNeedPub_ = false;
    is_tgt_found = true;
  }
}

void TargetSelecter::roiCallback(const std_msgs::Float32MultiArrayConstPtr &roi_msg)
{
  if (roi_msg->data.size() == 4) {
    std_msgs::UInt16MultiArray array;
    for (size_t i = 0; i < 4; i++)
    {
      int v = roi_msg->data[i];
      array.data.push_back(v);
    }
    
    rt_.tgt_bbox_center.data.clear();

    rt_.label.data = "object";
    rt_.tgt_bbox_array = array;
    rt_.tgt_bbox_center.data.push_back((array.data[0] + array.data[2])/2);
    rt_.tgt_bbox_center.data.push_back((array.data[1] + array.data[3])/2);
    targetNeedPub_ = true;
    ROS_INFO("User selected target received.");
  }
  else {
    ROS_INFO("Target cancelled by JARVIS.");
    std::string param_target_set = "/comm/param/control/target/is_set";
    ros::param::set(param_target_set, false);
    targetNeedPub_ = false;
  }
}

void TargetSelecter::IVROICallback(const geometry_msgs::PolygonStampedConstPtr &roi_msg)
{
  if (roi_msg->polygon.points.size() == 2) {
    std_msgs::UInt16MultiArray array;
    for (size_t i = 0; i < 2; i++) {
      geometry_msgs::Point32 p = roi_msg->polygon.points[i];
      int x = p.x;
      int y = p.y;
      array.data.push_back(x);
      array.data.push_back(y);
    }
    
    rt_.tgt_bbox_center.data.clear();

    rt_.label.data = "object";
    rt_.tgt_bbox_array = array;
    rt_.tgt_bbox_center.data.push_back((array.data[0] + array.data[2])/2);
    rt_.tgt_bbox_center.data.push_back((array.data[1] + array.data[3])/2);
    targetNeedPub_ = true;
    ROS_INFO("User selected target received.");
  }
  else {
    ROS_INFO("Rect from ImageView2 should have 2 points.");
    std::string param_target_set = "/comm/param/control/target/is_set";
    ros::param::set(param_target_set, false);
    targetNeedPub_ = false;
  }
}

void TargetSelecter::IVPCallback(const geometry_msgs::PointStampedConstPtr &p_msg)
{
  ROS_INFO("Target cancelled by ImageView2.");
  std::string param_target_set = "/comm/param/control/target/is_set";
  ros::param::set(param_target_set, false);
  targetNeedPub_ = false;
}
