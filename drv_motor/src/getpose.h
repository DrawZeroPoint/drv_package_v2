#ifndef GETPOSE_H
#define GETPOSE_H

#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <iostream>

using namespace std;

class GetPose
{
public:
  GetPose();

  bool getNextPosition(int &pitch, int &yaw);

private:
  int min_yaw_;
  int max_yaw_;
  int yaw_step_;
  int min_pitch_;
  int max_pitch_;
  int pitch_step_;

  bool moveDirection;
};

#endif // GETPOSE_H
