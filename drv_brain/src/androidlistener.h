#ifndef ANDROIDLISTENER_H
#define ANDROIDLISTENER_H

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt16MultiArray.h>

#include <drv_msgs/recognized_target.h>

class AndroidListener
{
public:
  AndroidListener();

  ros::NodeHandle nh;
  void publishOnceIfTargetSelected(bool is_tgt_set, bool &is_tgt_found);

private:
  ros::Publisher ALPubROI_;
  ros::Subscriber ALSubROI_;
  void roiCallback(const std_msgs::Float32MultiArrayConstPtr &roi_msg);

  drv_msgs::recognized_target rt_;
  bool targetNeedPub_;

};

#endif // ANDROIDLISTENER_H
