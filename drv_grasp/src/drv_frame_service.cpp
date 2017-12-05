#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <drv_msgs/frame_transform.h>

#include <cstdlib>
#include <stdio.h>

using namespace std;

bool frame_transform(drv_msgs::frame_transform::Request &req,
                     drv_msgs::frame_transform::Response &res)
{ 
  if (req.type == 0) {
    // The point in target frame and the transform from 
    // target frame to base frame is known, 
    // get the point's coordinate in base frame
    for (size_t i = 0; i < req.p_in_tgt.size(); ++i) {
      
    }
  }
  else if (req.type == 1) {
    // The point in base frame and the transform from 
    // base frame to target frame is known, 
    // get the point's coordinate in target frame
    for (size_t i = 0; i < req.p_in_base.size(); ++i) {
      
    }
  }
  else if (req.type == 2) {
    // The point's coordinate in base frame and the target frame is known, 
    // get the translation from base frame to target frame
  }
  else
    return false;
  return true; // Return bool is necessary for service
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drv_frame_server");
  ros::NodeHandle n;
  ros::NodeHandle pnh("~");
  
  ros::ServiceServer service = n.advertiseService("drv_frame_service", frame_transform);
  ROS_INFO("Ready to do transform between frames.");
  while(ros::ok()) {
    ros::spinOnce();
    
  }
  
  return 0;
}
