#ifndef SMOOTHSERVO_H
#define SMOOTHSERVO_H

#include <ros/ros.h>

#include <std_msgs/UInt16MultiArray.h>

using namespace std;

class SmoothServo
{
public:
  SmoothServo();

  bool moveServoTo(int pitch, int yaw);
  void getCurrentServoAngle(int &pitch, int &yaw);

private:
  int pitch_temp_;
  int yaw_temp_;
  int step;

  ros::NodeHandle nh;
  ros::Publisher servoPubSearch_;

  bool smooth(vector<vector<int> > &path, int p, int y);

};

#endif // SMOOTHSERVO_H
