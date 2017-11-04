#include "detectcolor.h"

DetectColor::DetectColor()
{
}

bool DetectColor::detect(Mat img_in, Mat &img_out,
                         vector<vector<int> > &bbox_array)
{

  return true;
}

void DetectColor::setHSV(int h_l, int s_l, int v_l, 
                         int h_h, int s_h, int v_h)
{
  hsv_low_ = Scalar(h_l, s_l, v_l);
  hsv_high_ = Scalar(h_h, s_h, v_h);
}
