#ifndef UTILITIES_H
#define UTILITIES_H

#include <opencv2/opencv.hpp>

class Utilities
{
public:
  Utilities();

  static void drawObject(int x, int y, cv::Mat &frame);
  static void markImage(cv::Mat img_in, cv::Rect roi, cv::Mat &img_out,
                        std::vector<unsigned int> &mask_id, float &color_mean);
  static std::string intToString(int number);
  static void expandGt(cv::Rect &gt, int margin);
};

#endif // UTILITIES_H
