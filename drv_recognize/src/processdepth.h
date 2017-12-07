#ifndef PROCESSDEPTH_H
#define PROCESSDEPTH_H

//STL
#include <iostream>
#include <math.h>
#include <vector>

//OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace std;
using namespace cv;

class ProcessDepth
{
public:
  ProcessDepth();
  
  bool detectHand(Mat depth, vector<int> &bbox, 
                  Point3f &center, int &gesture);
  
private:
  bool getContentInRange(Mat depth, Mat &depth_out, vector<int> &bbox, 
                         float range_max, float range_min = 0.3);
  
  float getDepth(const Mat &depthImage, int x, int y, bool smoothing, 
                 float maxZError, bool estWithNeighborsIfNull);
  
  bool analyseGesture(Mat depth, vector<int> bbox, 
                      Point3f &center, int &gesture);
  
  template<class T>
  inline bool uIsFinite(const T & value) {
    return std::isfinite(value);
  }
};

#endif // PROCESSDEPTH_H
