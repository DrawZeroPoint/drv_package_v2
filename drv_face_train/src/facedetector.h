#ifndef FACEDETECTOR_H
#define FACEDETECTOR_H

#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <stdio.h>

using namespace std;
using namespace cv;

class FaceDetector
{
public:
  FaceDetector(string path);
  bool getOneFace(Mat img_in, Mat &img_out);
  bool Process(Mat img_in, Mat &img_out, vector<Rect> &faceRoi, std::vector<Mat> &face_imgs);

private:
  void detectAndDraw(Mat img_in, Mat& img_out, double scale, bool tryflip, vector<Rect> &face_roi);
  void getFaceFromRoi(Mat img_in, vector<Rect> face_roi, vector<Mat> &face_imgs);
  bool trySquareRoi(Mat img_in, Rect face_roi, Rect &square_roi);
  bool isOnCenter(Rect rect, Mat img_in);
  
  CascadeClassifier cascade_;
};

#endif // FACEDETECTOR_H
