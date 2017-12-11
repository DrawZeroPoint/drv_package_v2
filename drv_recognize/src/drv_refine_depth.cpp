#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <dynamic_reconfigure/server.h>

#include <cstdlib>
#include <stdio.h>
#include <vector>

#include "refinedepth.h"
#include "drv_msgs/refine_depth.h"

using namespace std;
using namespace cv;

RefineDepth m_rd_;

bool refine_depth(drv_msgs::refine_depth::Request  &req,
                  drv_msgs::refine_depth::Response &res)
{
  cv_bridge::CvImagePtr rgb = cv_bridge::toCvCopy(req.rgb_in, "bgr8");
  cv_bridge::CvImagePtr depth = cv_bridge::toCvCopy(req.depth_in);

  Mat depth_refined;
  m_rd_.refineDepth(rgb->image, depth->image, depth_refined);

  // Convert cv::Mat to sensor_msgs::Image
  cv_bridge::CvImage img_cv;
  img_cv.image = depth_refined;
  img_cv.encoding = req.depth_in.encoding;
  //cerr << img_cv.encoding << endl; 32FC1
  res.depth_out = *img_cv.toImageMsg();
  
  return true; // Return bool is necessary for service
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "refine_depth_server");
  ros::NodeHandle n;
  ros::NodeHandle pnh("~");

  ros::ServiceServer srv = n.advertiseService("drv_refine_depth", refine_depth);
  ROS_INFO("Ready to refine depth image.");
  ros::spin();

  return 0;
}
