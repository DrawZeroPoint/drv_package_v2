#ifndef PROCESSFACE_H
#define PROCESSFACE_H

#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "classifier.h"

using namespace std;

class ProcessFace
{
public:
    ProcessFace(string test_proto, string caffe_model, int gpu_id, bool do_train);

    void processFace(cv::Mat img_in, int &result_id, float &result_trust);

private:
    Classifier classifier_;
};

#endif // PROCESSFACE_H
