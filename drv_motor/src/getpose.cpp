#include "getpose.h"

GetPose::GetPose()
{
  min_yaw_ = 0;
  max_yaw_ = 180;
  yaw_step_ = 20;
  min_pitch_ = 60;
  max_pitch_ = 80;
  pitch_step_ = 10;

  moveDirection = true;
}

bool GetPose::getNextPosition(int &pitch, int &yaw)
{
  pitch = 60;
  if (moveDirection) {
    if (yaw + yaw_step_ > max_yaw_) {
      yaw -= yaw_step_;
      moveDirection = !moveDirection;
      return true;
    }
    else {
      yaw += yaw_step_;
    }
  }
  else {
    if (yaw - yaw_step_ < min_yaw_) {
      moveDirection = true; // reset
      return false;
    }
    else {
      yaw -= yaw_step_;
    }
  }
  return true;
}
