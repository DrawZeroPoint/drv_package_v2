#ifndef GOTURN_H
#define GOTURN_H

#include <string>

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "helper/high_res_timer.h"
#include "network/regressor.h"
#include "loader/loader_alov.h"
#include "loader/loader_vot.h"
#include "tracker/tracker.h"
#include "tracker/tracker_manager.h"

#include "utilities.h"

using namespace std;
using namespace cv;

class Goturn
{
public:
  Goturn(string test_proto, string caffe_model, int gpu_id,
         const bool do_train, const bool show_output);

  bool tracker_initialized_;

  bool goProcess(Mat img_in, Rect gt, Mat &img_out, Rect &detection, std::vector<unsigned int> &mask_id);

private:
  Regressor regressor_;
  Tracker tracker_;
  float color_mean_temp_;

  void goInit(Mat img, Rect gt);
  Rect goTrack(Mat img);
};

#endif // GOTURN_H
