#include "processface.h"

ProcessFace::ProcessFace(string test_proto, string caffe_model, int gpu_id, bool do_train) :
    classifier_(test_proto, caffe_model, gpu_id, do_train)
{

}

void ProcessFace::processFace(cv::Mat img_in, int &result_id, float &result_trust)
{
    classifier_.Classify(img_in,result_id, result_trust);
}
