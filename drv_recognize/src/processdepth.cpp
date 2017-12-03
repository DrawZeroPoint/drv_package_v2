#include "processdepth.h"

ProcessDepth::ProcessDepth()
{
}

bool ProcessDepth::detectHand(Mat depth, vector<int> &bbox, 
                              Point3f &center, int &gesture)
{
  Mat depth_in_range;
  if (getContentInRange(depth, depth_in_range, bbox, 0.45)) {
    return analyseGesture(depth_in_range, bbox, center, gesture);
  }
  else
    return false;
}

bool ProcessDepth::getContentInRange(Mat depth, Mat &depth_out, vector<int> &bbox, 
                                     float range_max, float range_min)
{
  depth_out = Mat(depth.size(), CV_32FC1, Scalar(0.0));
  int rmin = 640;
  int cmin = 480;
  int rmax = 0;
  int cmax = 0;
  bool is_empty = true;
  for (size_t r = 0; r < depth.rows; ++r) {
    for (size_t c = 0; c < depth.cols; ++c) {
      float val = depth.at<float>(r, c);
      if (val > range_min && val < range_max) {
        is_empty = false;
        depth_out.at<float>(r, c) = val;
        if (r < rmin) rmin = r;
        if (c < cmin) cmin = c;
        if (r > rmax) rmax = r;
        if (c > cmax) cmax = c;
      }
    }
  }
  
  // In order xmin, ymin, xmax, ymax
  bbox.push_back(cmin);
  bbox.push_back(rmin);
  bbox.push_back(cmax);
  bbox.push_back(rmax);
  return !is_empty;
}

bool ProcessDepth::analyseGesture(Mat depth, vector<int> bbox, 
                                  Point3f &center, int &gesture)
{
  Mat depth_b(depth.size(), CV_32FC1, Scalar(0.0));
  threshold(depth, depth_b, 0.0, 255.0, THRESH_BINARY);
  
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  
  // Find contours
  Mat depth_u(depth.size(), CV_8UC1, Scalar(0));
  //  FindContours support only 8uC1 and 32sC1 images
  depth_b.convertTo(depth_u, CV_8UC1);
  findContours(depth_u, contours, hierarchy, CV_RETR_TREE, 
               CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
  
  // Find the convex hull object for the largest contour
  vector<vector<Point> >hull(contours.size());
  vector<Point> contour_max;
  vector<Point> hull_max;
  float rt = 0.0;
  Point2f center_t;
  for(int i = 0; i < contours.size(); i++ ) {
    convexHull(Mat(contours[i]), hull[i], false);
    Point2f ct;
    float rad;
    minEnclosingCircle(hull[i], ct, rad);
    if (rad > rt) {
      rt = rad;
      center_t = ct;
      contour_max = contours[i];
      hull_max = hull[i];
    }
  }
  
  // Check the size of max hull and compare with bbox size
  float area_hull = contourArea(hull_max);
  float area_contour = contourArea(contour_max);
  float ratio = area_contour / area_hull;
  if (ratio < 0.2)
    return false;
  
  float fx = 579.77;
  float fy = 584.94;
  float cx = 333.77;
  float cy = 237.86;
  float d = depth.at<float>(int(center_t.y), int(center_t.x));
  center.x = (center_t.x - cx) * d / fx;
  center.y = (center_t.y - cy) * d / fy;
  center.z = d;
  
  if (ratio > 0.6) {
    gesture = 0;
    return true;
  }
  else {
    gesture = 5;
    return true;
  }
}

float ProcessDepth::getDepth(const Mat &depthImage, int x, int y,
                             bool smoothing, float maxZError, 
                             bool estWithNeighborsIfNull)
{
  int u = x;
  int v = y;
  
  // Inspired from RGBDFrame::getGaussianMixtureDistribution() method from
  // https://github.com/ccny-ros-pkg/rgbdtools/blob/master/src/rgbd_frame.cpp
  // Window weights:
  //  | 1 | 2 | 1 |
  //  | 2 | 4 | 2 |
  //  | 1 | 2 | 1 |
  int u_start = max(u - 1, 0);
  int v_start = max(v - 1, 0);
  int u_end = min(u + 1, depthImage.cols - 1);
  int v_end = min(v + 1, depthImage.rows - 1);
  
  float depth = 0.0f;
  depth = depthImage.at<float>(v, u);
  
  if((depth == 0.0f || !uIsFinite(depth)) && estWithNeighborsIfNull) {
    // all cells no2 must be under the zError to be accepted
    float tmp = 0.0f;
    int count = 0;
    for(int uu = u_start; uu <= u_end; ++uu) {
      for(int vv = v_start; vv <= v_end; ++vv) {
        if((uu == u && vv != v) || (uu != u && vv == v)) {
          float d = 0.0f;
          
          d = depthImage.at<float>(vv, uu);
          if(d != 0.0f && uIsFinite(d)) {
            if(tmp == 0.0f) {
              tmp = d;
              ++count;
            }
            else if(fabs(d - tmp) < maxZError) {
              tmp += d;
              ++count;
            }
          }
        }
      }
    }
    if(count > 1) {
      depth = tmp / float(count);
    }
  }
  
  if(depth != 0.0f && uIsFinite(depth)) {
    if(smoothing) {
      float sumWeights = 0.0f;
      float sumDepths = 0.0f;
      for(int uu = u_start; uu <= u_end; ++uu) {
        for(int vv = v_start; vv <= v_end; ++vv) {
          if(!(uu == u && vv == v)) {
            float d = depthImage.at<float>(vv,uu);
            
            // ignore if not valid or depth difference is too high
            if(d != 0.0f && uIsFinite(d) && fabs(d - depth) < maxZError) {
              if(uu == u || vv == v) {
                sumWeights += 2.0f;
                d *= 2.0f;
              }
              else {
                sumWeights+=1.0f;
              }
              sumDepths += d;
            }
          }
        }
      }
      // set window weight to center point
      depth *= 4.0f;
      sumWeights += 4.0f;
      
      // mean
      depth = (depth+sumDepths)/sumWeights;
    }
  }
  else 
    depth = 0;
  return depth;
}
