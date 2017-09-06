#include "segment.h"

using namespace cv;

Segment::Segment() : it_(nh_)
{
  pubImage_ = it_.advertise("search/labeled_image", 1);
}

void Segment::segment(cv_bridge::CvImageConstPtr imagePtr, cv_bridge::CvImageConstPtr depthPtr)
{
  Mat rgb,  rgb_edge;
  rgb = imagePtr->image;
  cannyDetect(rgb, rgb_edge, 40, 120);

  Mat depth, depth_edge;
  depthPtr->image.convertTo(depth,CV_8UC1, 0.05);//scale factor 0.05
  cannyDetect(depth, depth_edge, 0, 20);

  Mat combine;
  bitwise_and(rgb_edge, depth_edge, combine);

  imshow("c", combine);
  waitKey();
}

void Segment::cannyDetect(Mat img_in, Mat &img_out, int t1, int t2)
{
  Mat gray_blur;

  /// Reduce noise with a kernel 3x3
  blur(img_in, gray_blur, Size(3,3) );

  Canny(gray_blur, img_out, t1, t2);

  dilate(img_out, img_out, Mat(3,3,CV_8U));

  imwrite("/home/aicrobo/Documents/3.png", img_out);

  imshow("Edge", img_out);
  waitKey();
}
