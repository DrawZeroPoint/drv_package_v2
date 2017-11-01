#include "smoothservo.h"

SmoothServo::SmoothServo()
{
  pitch_temp = 60;
  yaw_temp = 90;
  step = 1; // minium step for single move
  servoPubSearch_ = nh.advertise<std_msgs::UInt16MultiArray>("/vision/servo", 1);
}

void SmoothServo::getCurrentServoAngle(int pitch, int yaw)
{
  pitch_temp = pitch;
  yaw_temp = yaw;
}

bool SmoothServo::moveServoTo(int pitch, int yaw)
{
  vector<vector<int> > path;

  if (smooth(path, pitch, yaw)) {
    for (size_t j = 0; j < path[0].size(); j++)
    {
      // Notice that this array should be cleaned in each loop
      std_msgs::UInt16MultiArray array;
      array.data.push_back(path[0][j]);
      array.data.push_back(yaw_temp);
      servoPubSearch_.publish(array);
      ros::Duration(0.1).sleep();
    }
    for (size_t j = 0; j < path[1].size(); j++)
    {
      // Notice that this array should be cleaned in each loop
      std_msgs::UInt16MultiArray array;
      array.data.push_back(pitch);
      array.data.push_back(path[1][j]);
      servoPubSearch_.publish(array);
      ros::Duration(0.1).sleep();
    }
    ros::Duration(0.2).sleep(); // give the servo time for getting to position
  }
  return true;
}

bool SmoothServo::smooth(vector<vector<int> > &path, int p, int y)
{
  //  0 < p < 150, 0 <= y <=180
  if (p < 0 || p > 150 || y < 0 || y > 180)
    return false;

  int mp = (p - pitch_temp) / step;
  int my = (y - yaw_temp) / step;

  if (mp == 0 && my == 0)
    return false;

  vector<int> pv;
  vector<int> yv;

  for (size_t i = 1; i < abs(mp) + 1; i++)
    pv.push_back(pitch_temp + mp / abs(mp) * i * step);
  pv.push_back(p);

  for (size_t j = 1; j < abs(my) + 1;  j++)
    yv.push_back(yaw_temp + my / abs(my) * j * step);
  yv.push_back(y);

  // path: pitch_value: p1,p2,p3...; yaw_value: y1,y2,y3...
  path.push_back(pv);
  path.push_back(yv);

  return true;
}
