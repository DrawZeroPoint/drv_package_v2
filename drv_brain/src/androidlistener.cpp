#include "androidlistener.h"

AndroidListener::AndroidListener()
{
  targetNeedPub_ = false;
  ALPubROI_ = nh.advertise<drv_msgs::recognized_target>("search/recognized_target", 1);
  ALSubROI_ = nh.subscribe<std_msgs::Float32MultiArray>("/jarvis/roi", 1, &AndroidListener::roiCallback, this);
}

void AndroidListener::publishOnceIfTargetSelected(bool is_tgt_set, bool &is_tgt_found)
{
  if (is_tgt_set && targetNeedPub_)
  {
    ALPubROI_.publish(rt_);
    targetNeedPub_ = false;
    is_tgt_found = true;
  }
}

void AndroidListener::roiCallback(const std_msgs::Float32MultiArrayConstPtr &roi_msg)
{
  if (roi_msg->data.size() == 4)
  {
    std_msgs::UInt16MultiArray array;
    for (size_t i = 0; i < 4; i++)
    {
      int v = roi_msg->data[i];
      array.data.push_back(v);
    }

    rt_.label.data = "object";
    rt_.tgt_bbox_array = array;
    rt_.tgt_bbox_center.data[0] = int((roi_msg->data[0] + roi_msg->data[2])/2);
    rt_.tgt_bbox_center.data[1] = int((roi_msg->data[1] + roi_msg->data[3])/2);
    targetNeedPub_ = true;
    ROS_INFO("User selected target received.\n");
  }
  else
    targetNeedPub_ = false;
}
