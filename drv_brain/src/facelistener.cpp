#include "facelistener.h"

FaceListener::FaceListener()
{
  control_param_need_recog =   "/comm/param/control/face/need_recognize";
  local_param_need_recog = "/vision/face/need_recognize";
}

void FaceListener::isNeedRecognizeFace()
{
//  if (ros::param::has(local_param_need_recog))
//  {
//    bool need_recog = false;
//    ros::param::get(local_param_need_recog, need_recog);
//    if (need_recog)
//    {
//      ros::param::set(local_param_need_recog, true);
//    }
//    else
//    {
//      ros::param::set(local_param_need_recog, false);
//    }
//  }
//  else
//    ros::param::set(local_param_need_recog, false);
}
