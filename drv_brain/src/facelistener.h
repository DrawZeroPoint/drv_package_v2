#ifndef FACELISTENER_H
#define FACELISTENER_H

#include <ros/ros.h>

using namespace std;

class FaceListener
{
public:
  FaceListener();
  void isNeedRecognizeFace();

private:
  string control_param_need_recog;
  string local_param_need_recog;
};

#endif // FACELISTENER_H
