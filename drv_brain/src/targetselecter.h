#ifndef ANDROIDLISTENER_H
#define ANDROIDLISTENER_H

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt16MultiArray.h>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>

#include <drv_msgs/recognized_target.h>

class TargetSelector
{
public:
  TargetSelector();

  ros::NodeHandle nh;
  void publishOnceIfTargetSelected(bool &is_tgt_set, bool &is_tgt_found);

private:
  ros::Publisher ALPubROI_;
  
  // ALSubROI_ sub roi from Android App and process it with roiCallback
  ros::Subscriber ALSubROI_;
  void APPROICallback(const std_msgs::Float32MultiArrayConstPtr &roi_msg);
  
  // subImageView2ROI_ sub roi from ImageView2 and process it with 
  ros::Subscriber subImageView2ROI_;
  void IVROICallback(const geometry_msgs::PolygonStampedConstPtr &roi_msg);
  
  ros::Subscriber subImageView2P_;
  void IVPCallback(const geometry_msgs::PointStampedConstPtr &p_msg);

  drv_msgs::recognized_target rt_;
  bool targetNeedPub_;

};

#endif // ANDROIDLISTENER_H
