#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <drv_msgs/face_recognize.h>

#include <cstdlib>
#include <stdio.h>

#include "processface.h"

char* drv_path_env = std::getenv("DRV");

std::string drv_path_ = std::string(drv_path_env);

std::string prototxt = drv_path_ + "/supplements/face_recognize/face_deploy.prototxt";
std::string caffemodel = drv_path_ + "/supplements/face_recognize/face_deploy.caffemodel";

ProcessFace pf_(prototxt, caffemodel, 0, false);
bool pf_init = false;

float face_th_ = 0.9;

bool recognize_face(drv_msgs::face_recognize::Request  &req,
                    drv_msgs::face_recognize::Response &res)
{
  // Incase the model is not generated when initialize pf
//  if (!pf_init) {
//    pf_.initClassifier(prototxt, caffemodel, 0);
//    pf_init = true;
//  }
  
  size_t im = req.images_in.size();
  vector<int> result_id;
  for (size_t i = 0;i < im; i++) {
    cv_bridge::CvImagePtr src = cv_bridge::toCvCopy(req.images_in[i], "bgr8");
    int id = 0;
    float result_trust = 0.0;
    pf_.processFace(src->image, id, result_trust);
    
    // result_id starting from 0, when result_id=0 the face is "unknown",
    // while known person id starting from 1 which is result_id+1
    if (result_trust < face_th_)
      result_id.push_back(0);
    else
      result_id.push_back(id + 1);
  }
  res.face_label_ids = result_id;
  return true; // return bool is necessary for service
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "face_recognize_server");
  ros::NodeHandle n;
  ros::NodeHandle pnh("~");
  
  pnh.getParam("face_likelihood", face_th_);
  
  ros::ServiceServer service = n.advertiseService("drv_face_service", recognize_face);
  ROS_INFO("Ready to recognize face.");
  ros::spin();
  
  return 0;
}
