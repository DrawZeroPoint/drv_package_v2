#include "search.h"

Search::Search()
{
  minPX = 0;
  maxPX = 180;
  xStep = 20;
  minPY = 60;
  maxPY = 80;
  yStep = 10;

  moveDirection = true;

  apRatio = 0.1;
}

Search::Search(int minpx = 0, int maxpx = 180, int xstep = 30,
               int minpy = 60, int maxpy = 80, int ystep = 10)
{
  minPX = minpx;
  maxPX = maxpx;
  xStep = xstep;
  minPY = minpy;
  maxPY = maxpy;
  yStep = ystep;
}

bool Search::getNextPosition(int & yaw, int & pitch)
{
  if (moveDirection) {
    if (yaw + xStep > maxPX) {
      yaw -= xStep;
      moveDirection = !moveDirection;
      return true;
    }
    else {
      yaw += xStep;
    }
  }
  else {
    if (yaw - xStep < minPX) {
      moveDirection = true; // reset
      return false;
    }
    else {
      yaw -= xStep;
    }
  }
  return true;
}


bool Search::getTargetPosition(vector<std_msgs::UInt16MultiArray> bbox_array,
                               int num, int &delta_pitch, int &delta_yaw)
{
  // Array index starts from 0, while num starts from 1
  std_msgs::UInt16MultiArray array = bbox_array[num - 1];
  int roix = (array.data[0] + array.data[2]) / 2;
  int roiy = (array.data[1] + array.data[3]) / 2;

  // Convert image pixel distance to angle
  int d_x = roix - 320;
  int d_y = roiy - 240;

  // Offset angle the robot head needs to turn
  delta_yaw = - int(d_x * apRatio);
  delta_pitch = - int(d_y * apRatio);
}
