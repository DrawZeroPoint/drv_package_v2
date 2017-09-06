#ifndef CLASSIFIER_H
#define CLASSIFIER_H

#include <caffe/caffe.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iosfwd>
#include <memory>
#include <string>
#include <utility>
#include <vector>


class Classifier
{
public:
    Classifier(const std::string& train_deploy_proto,
               const std::string& caffe_model,
               const int gpu_id,
               const bool do_train);

    void Classify(cv::Mat image_in, int &id, float &trust);

protected:
  boost::shared_ptr<caffe::Net<float> > net_;

  // Get the features corresponding to the output of the network.
  void GetOutput(int &id, float &trust);

  // Get the features in the network with the given name, and copy their values to the output.
  void GetFeatures(const std::string& feature_name, int &id, float &trust);

private:
    // Set up a network with the architecture specified in deploy_proto,
    // with the model weights saved in caffe_model.
    void SetupNetwork(const std::string& deploy_proto,
                      const std::string& caffe_model,
                      const int gpu_id,
                      const bool do_train);

    // Set the mean input (used to normalize the inputs to be 0-mean).
    void SetMean();

    // Wrap the input layer of the network in separate cv::Mat objects
    // (one per channel).
    void WrapInputLayer(std::vector<cv::Mat>* target_channels);

    // Set the inputs to the network.
    void Preprocess(const cv::Mat& img, std::vector<cv::Mat>* input_channels);

    std::vector<int> Argmax(std::vector<float> v, int N);

private:
    // Folder containing the model parameters.
    std::string caffe_model_;
    // Size of the input images.
    cv::Size input_geometry_;

    // Number of image channels: normally either 1 (black and white) or 3 (color).
    int num_channels_;

    // Mean image, used to make the input 0-mean.
    cv::Mat mean_;
};

#endif // CLASSIFIER_H
