#ifndef SEARCH_H
#define SEARCH_H

#include <ros/ros.h>

#include <std_msgs/UInt16MultiArray.h>

#include <iostream>

using namespace std;

class Search
{
public:
  Search();
  Search(int minpx, int maxpx, int xstep, int minpy, int maxpy, int ystep);

  bool getNextPosition(int & yaw, int & pitch);
  bool getTargetPosition(std::vector<std_msgs::UInt16MultiArray> bbox_array,
                         int num, int & delta_pitch, int &delta_yaw);

private:

  int minPX;
  int maxPX;
  int xStep;
  int minPY;
  int maxPY;
  int yStep;

  bool moveDirection;

  float apRatio; // pitch/yaw angle to roll per pixel
};

#endif // SEARCH_H

