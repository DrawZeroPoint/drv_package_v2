#include "facedetector.h"

FaceDetector::FaceDetector(string path)
{
  cascade_.load(path + "/supplements/face_recognize/haarcascade_frontalface_alt.xml");
  nestedCascade_.load(path + "/supplements/face_recognize/haarcascade_eye_tree_eyeglasses.xml");
}

bool FaceDetector::Process(Mat img_in, Mat &img_out, vector<Rect> &faceRoi, std::vector<Mat> &face_imgs)
{
  detectAndDraw(img_in, img_out, 1, 0, faceRoi);
  if (!faceRoi.size())
    return false;
  else
  {
    getFaceFromRoi(img_in, faceRoi, face_imgs);
    return true;
  }
}

void FaceDetector::detectAndDraw(Mat img_in, Mat &img_out, double scale,
                                 bool tryflip, vector<Rect> &face_roi)
{
  int i = 0;

  vector<Rect> faces, faces2;
  Scalar color = Scalar(0, 255, 0);
  Mat gray, smallImg(cvRound (img_in.rows/scale), cvRound(img_in.cols/scale), CV_8UC1);
  img_out = img_in.clone();

  cvtColor(img_in, gray, CV_BGR2GRAY);
  resize(gray, smallImg, smallImg.size(), 0, 0, INTER_LINEAR);
  equalizeHist(smallImg, smallImg);

  cascade_.detectMultiScale(smallImg, faces, 1.05, 6, 0 | CV_HAAR_SCALE_IMAGE, Size(80, 80));
  if(tryflip)
  {
    flip(smallImg, smallImg, 1);
    cascade_.detectMultiScale(smallImg, faces2, 1.05, 6, 0 | CV_HAAR_SCALE_IMAGE, Size(80, 80));
    for(vector<Rect>::const_iterator r = faces2.begin(); r != faces2.end(); r++)
    {
      faces.push_back(Rect(smallImg.cols - r->x - r->width, r->y, r->width, r->height));
    }
  }
  face_roi = faces; // get all face regions

  for(vector<Rect>::const_iterator r = faces.begin(); r != faces.end(); r++, i++)
  {
    rectangle(img_out, cvPoint(cvRound(r->x*scale), cvRound(r->y*scale)),
              cvPoint(cvRound((r->x + r->width-1)*scale), cvRound((r->y + r->height-1)*scale)),
              color, 2, 8, 0);
  }
}

void FaceDetector::getFaceFromRoi(Mat img_in, vector<Rect> face_roi, vector<Mat> &face_imgs)
{
  for (size_t i = 0; i < face_roi.size(); i++)
  {
    Rect square_roi;
    if (trySquareRoi(img_in, face_roi[i], square_roi)) {
      Mat face_temp = img_in(square_roi);
      face_imgs.push_back(face_temp.clone());
    }
  }
}

bool FaceDetector::trySquareRoi(Mat img_in, Rect face_roi, Rect &square_roi)
{
  int w = face_roi.width;
  int h = face_roi.height;

  int margin = 20; // expand the roi
  if (w >= h)
  {
    if (face_roi.y + h/2 - w/2 - margin >= 0)
    {
      square_roi.y = cvRound(face_roi.y + h/2 - w/2 - margin);
    }
    else
    {
      square_roi.y = 0;
    }
    if (face_roi.y + h/2 + w/2 + margin < img_in.rows)
    {
      square_roi.height = w + 2*margin;
    }
    else
    {
      square_roi.height = img_in.rows - square_roi.y + margin;
    }
    square_roi.x = face_roi.x;
    square_roi.width = w;
  }
  else
  {
    if (face_roi.x + w/2 - h/2 - margin >=0)
    {
      square_roi.x = cvRound(face_roi.x + w/2 - h/2 - margin);
    }
    else
    {
      square_roi.x = 0;
    }
    if (face_roi.x + w/2 + h/2 + margin < img_in.cols)
    {
      square_roi.width = h + 2 * margin;
    }
    else
    {
      square_roi.width = img_in.cols - square_roi.x - margin;
    }
    square_roi.y = face_roi.y;
    square_roi.height = h;
  }
  if (square_roi.height + square_roi.y >= img_in.rows || square_roi.width + square_roi.x >= img_in.cols)
    return false;
  else
    return true;
}
