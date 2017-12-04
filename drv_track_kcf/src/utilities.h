#ifndef UTILITIES_H
#define UTILITIES_H

#include <opencv2/opencv.hpp>

// Default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;

// Minimum and maximum object area
const int MIN_OBJECT_AREA = 200;
const int MAX_OBJECT_AREA =  200000;
const float COLOR_DIFF = 10;

class Utilities
{
public:
  Utilities();

  static void drawCross(int x, int y, cv::Mat &frame);

  static void markImage(cv::Mat img_in, cv::Rect roi, cv::Mat &img_out,
                        std::vector<unsigned int> &mask_id, float &color_mean);

  static void markImage(cv::Rect roi, cv::Mat &img_out);

  static std::string intToString(int number);

  static void expandGt(cv::Rect &gt, int margin);
  
  static bool tryExpandROI(int &minx, int &miny, int &maxx, int &maxy, int pad, 
                           int width = 640, int height = 480);
};

#endif // UTILITIES_H
