#ifndef DETECTCOLOR_H
#define DETECTCOLOR_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cstdlib>
#include <stdio.h>
#include <vector>

using namespace std;
using namespace cv;

class DetectColor
{
public:
  DetectColor();
  bool detect(Mat img_in, Mat &img_out, vector<vector<int> > &bbox_array);
  void setHSV(int h_l, int s_l, int v_l, int h_h, int s_h, int v_h);
  
private:
  Scalar hsv_low_;
  Scalar hsv_high_;

  void morphOps(Mat &thresh);
  bool process(Mat img_in, Mat threshold, Mat &img_out,
               vector<vector<int> > &bbox_array);
};

#endif // DETECTCOLOR_H
