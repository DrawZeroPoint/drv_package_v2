#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <drv_msgs/recognize.h>
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

bool recognize_color(drv_msgs::recognize::Request  &req,
                     drv_msgs::recognize::Response &res)
{
  cv_bridge::CvImagePtr src = cv_bridge::toCvCopy(req.img_in, "bgr8");

  // Set color region
  dc_.setHSV(hue_low_, sat_low_, val_low_, hue_high_, sat_high_, val_high_);
  
  // Detect certain color in input image
  Mat img_out;
  vector<vector<int> > bbox_array;
  if (!dc_.detect(src->image, img_out, bbox_array))
    return false;

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

  ros::ServiceServer srv = n.advertiseService("drv_recognize_color", recognize_color);
  ROS_INFO("Ready to recognize color.");
  ros::spin();

  return 0;
}
