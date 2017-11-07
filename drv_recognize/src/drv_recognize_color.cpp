#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <dynamic_reconfigure/server.h>

#include <drv_msgs/recognize.h>
#include <drv_recognize/recogConfig.h>
#include "detectcolor.h"

#include <cstdlib>
#include <stdio.h>
#include <vector>

using namespace std;
using namespace cv;

// Target color in HSV color space
int hue_low_ = 40;
int sat_low_ = 200;
int val_low_ = 200;
int hue_high_ = 70;
int sat_high_ = 255;
int val_high_ = 255;

DetectColor dc_;

void configCallback(drv_recognize::recogConfig &config, uint32_t level)
{
  hue_low_ = config.hue_low_cfg;
  sat_low_ = config.sat_low_cfg;
  val_low_ = config.val_low_cfg;
  hue_high_ = config.hue_high_cfg;
  sat_high_ = config.sat_high_cfg;
  val_high_ = config.val_high_cfg;
}

bool recognize_color(drv_msgs::recognize::Request  &req,
                     drv_msgs::recognize::Response &res)
{
  cv_bridge::CvImagePtr src = cv_bridge::toCvCopy(req.img_in, "bgr8");

  // Set color region
  dc_.setHSV(hue_low_, sat_low_, val_low_, hue_high_, sat_high_, val_high_);
  
  // Detect certain color in input image
  Mat img_out;
  vector<vector<int> > bbox_array;
  if (!dc_.detect(src->image, img_out, bbox_array)) {
    ROS_INFO("Nothing matched.");
    // The service should only return false when something
    // unexpected happen
    return true;
  }

  // Convert cv::Mat to sensor_msgs::Image
  cv_bridge::CvImage img_cv;
  img_cv.image = img_out;
  res.img_out = *img_cv.toImageMsg();

  // Fill in the response
  drv_msgs::recognized_objects obj_info;
  for (size_t i = 0; i < bbox_array.size(); ++i) {
    // Xmin, Ymin, Xmax, Ymax
    std_msgs::UInt16MultiArray arr;
    for (size_t j = 0; j < bbox_array[i].size(); ++j) {
      arr.data.push_back(bbox_array[i][j]);
    }
    obj_info.bbox_arrays.push_back(arr);

    // Fill label
    std_msgs::String str;
    str.data = "color";
    obj_info.labels.push_back(str);
  }

  res.obj_info = obj_info;
  return true; // Return bool is necessary for service
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "color_recognize_server");
  ros::NodeHandle n;
  ros::NodeHandle pnh("~");

  pnh.getParam("hue_low", hue_low_);
  pnh.getParam("sat_low", sat_low_);
  pnh.getParam("val_low", val_low_);
  pnh.getParam("hue_high", hue_low_);
  pnh.getParam("sat_high", sat_low_);
  pnh.getParam("val_high", val_low_);

  // Set up dynamic reconfigure callback
  dynamic_reconfigure::Server<drv_recognize::recogConfig> server;
  dynamic_reconfigure::Server<drv_recognize::recogConfig>::CallbackType f;

  f = boost::bind(&configCallback, _1, _2);
  server.setCallback(f);

  ros::ServiceServer srv = n.advertiseService("drv_recognize_color", recognize_color);
  ROS_INFO("Ready to recognize color.");
  ros::spin();

  return 0;
}
