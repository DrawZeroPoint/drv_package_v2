#ifndef TARGETSELECT_H
#define TARGETSELECT_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16MultiArray.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <cv.h>

#include <drv_msgs/user_select.h>
#include <drv_msgs/recognize.h>

using namespace std;
using namespace cv;

class TargetSelect
{
public:
  TargetSelect();

  int select(string targetLabel, drv_msgs::recognizeResponse response,
             sensor_msgs::Image img_in, cv_bridge::CvImagePtr &img_out, int &choosed_id);

private:
  ros::NodeHandle nh;
  ros::NodeHandle pnh;
  image_transport::ImageTransport rgb_it;

  ros::Publisher searchPubInfo_;
  image_transport::Publisher searchPubImage_; // publish labeled image for user judgement
  ros::ServiceClient client;

  int callService(int num);
  void paintTarget(Mat &img, int id, int fc, vector<std_msgs::UInt16MultiArray> box_array);

  inline string intToString(int number)
  {
    stringstream ss;
    ss << number;
    return ss.str();
  }

  inline void pubInfo(string info)
  {
    ROS_INFO(info.c_str());
    std_msgs::String msg;
    msg.data = info;
    searchPubInfo_.publish(msg);
  }
};

#endif // TARGETSELECT_H
