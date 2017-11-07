#include "detectcolor.h"

// Minimum and maximum object area defined by pixel number
const int MIN_OBJECT_AREA = 400;
const int MAX_OBJECT_AREA =  204800;

DetectColor::DetectColor()
{
}

bool DetectColor::detect(Mat img_in, Mat &img_out,
                         vector<vector<int> > &bbox_array)
{
  // HSV image
  Mat HSV;
  // Binary image
  Mat threshold;

  // Convert frame from BGR to HSV colorspace
  cvtColor(img_in, HSV, COLOR_BGR2HSV);

  inRange(HSV, hsv_low_, hsv_high_, threshold);

  morphOps(threshold);

  return process(img_in, threshold, img_out, bbox_array);
}

void DetectColor::setHSV(int h_l, int s_l, int v_l, 
                         int h_h, int s_h, int v_h)
{
  hsv_low_ = Scalar(h_l, s_l, v_l);
  hsv_high_ = Scalar(h_h, s_h, v_h);
}

void DetectColor::morphOps(Mat &thresh)
{
  // Create structuring element that will be used to "dilate" and "erode" image.
  // the element chosen here is a 3px by 3px rectangle

  Mat erodeElement = getStructuringElement(MORPH_RECT, Size(5, 5));
  // Dilate with larger element so make sure object is nicely visible
  Mat dilateElement = getStructuringElement(MORPH_RECT, Size(7, 7));

  erode(thresh,thresh,erodeElement);
  erode(thresh,thresh,erodeElement);

  dilate(thresh,thresh,dilateElement);
  dilate(thresh,thresh,dilateElement);
}

bool DetectColor::process(Mat img_in, Mat threshold, Mat &img_out,
                          vector<vector<int> > &bbox_array)
{
  Mat temp;
  threshold.copyTo(temp);

  img_in.copyTo(img_out);

  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

  double refArea = 0;
  bool objectFound = false;
  // Selected contour
  vector<Point> contour;

  if (hierarchy.size() > 0) {
    int numObjects = hierarchy.size();
    // If number of objects greater than MAX_NUM_OBJECTS we have a noisy filter

    for (int index = 0; index >= 0; index = hierarchy[index][0]) {
      Moments moment = moments((Mat)contours[index]);
      double area = moment.m00;

      /* If the area is less than 20 px by 20px then it is probably just noise
         if the area is the same as the 3/2 of the image size, probably just a bad filter
         we only want the object with the largest area so we safe a reference area each
         iteration and compare it to the area in the next iteration. */
      if(area > MIN_OBJECT_AREA && area < MAX_OBJECT_AREA && area > refArea) {
        contour = contours[index];
        objectFound = true;
        refArea = area;
      }
    }

    if(objectFound) {
      // Generate labeled image
      int x_max = 0;
      int x_min = 640;
      int y_max = 0;
      int y_min = 480;
      for (size_t i = 0; i < contour.size(); ++i) {
        Point p = contour[i];
        if (p.x > x_max)
          x_max = p.x;
        if (p.x < x_min)
          x_min = p.x;
        if (p.y > y_max)
          y_max = p.y;
        if (p.y < y_min)
          y_min = p.y;
      }
      vector<int> bbox;
      bbox.push_back(x_min);
      bbox.push_back(y_min);
      bbox.push_back(x_max);
      bbox.push_back(y_max);
      bbox_array.push_back(bbox);

      Rect rect(x_min, y_min, x_max - x_min, y_max - y_min);
      rectangle(img_out, rect, Scalar(0, 255, 0), 2);
    }
  }
  return objectFound;
}
