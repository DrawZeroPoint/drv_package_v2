#include "classifier.h"

using caffe::Blob;
using caffe::Net;
using std::string;

Classifier::Classifier(const string& deploy_proto,
                       const string& caffe_model,
                       const int gpu_id,
                       const bool do_train)
    : caffe_model_(caffe_model)
{
    SetupNetwork(deploy_proto, caffe_model, gpu_id, do_train);
}

void Classifier::SetupNetwork(const string& deploy_proto,
                              const string& caffe_model,
                              const int gpu_id,
                              const bool do_train) {
#ifdef CPU_ONLY
    printf("Setting up Caffe in CPU mode\n");
    caffe::Caffe::set_mode(caffe::Caffe::CPU);
#else
    printf("Setting up Caffe in GPU mode with ID: %d\n", gpu_id);
    caffe::Caffe::SetDevice(gpu_id);
    caffe::Caffe::set_mode(caffe::Caffe::GPU);
#endif

    if (do_train) {
            printf("Setting phase to train\n");
            net_.reset(new Net<float>(deploy_proto, caffe::TRAIN));
        } else {
            printf("Setting phase to test\n");
            net_.reset(new Net<float>(deploy_proto, caffe::TEST));
        }

    if (caffe_model != "NONE") {
            net_->CopyTrainedLayersFrom(caffe_model_);
        } else {
            printf("Not initializing network from pre-trained model\n");
        }

    Blob<float>* input_layer = net_->input_blobs()[0];

    num_channels_ = input_layer->channels();
    CHECK(num_channels_ == 3 || num_channels_ == 1) << "Input layer should have 1 or 3 channels.";
    input_geometry_ = cv::Size(input_layer->width(), input_layer->height());

    // Load the binaryproto mean file.
    SetMean();
}

void Classifier::SetMean() {
    // Set the mean image.
    mean_ = cv::Mat(input_geometry_, CV_32FC3, cv::Scalar(104, 117, 123));
}

void Classifier::Classify(cv::Mat image_in, int &id, float &trust) {
    assert(net_->phase() == caffe::TEST);

    // Reshape the input blobs to be the appropriate size.
    Blob<float>* input_image = net_->input_blobs()[0];
    input_image->Reshape(1, num_channels_, input_geometry_.height, input_geometry_.width);

    // Forward dimension change to all layers.
    net_->Reshape();

    // Process the inputs so we can set them.
    std::vector<cv::Mat> image_channels;
    WrapInputLayer(&image_channels);

    Preprocess(image_in, &image_channels);

    // Perform a forward-pass in the network.
    float *loss = new float;
    net_->ForwardPrefilled(loss);
    //  std::cerr<< "Loss is: " << *loss << std::endl;

    // Get the network output.
    GetOutput(id, trust);
}

// Wrap the input layer of the network in separate cv::Mat objects
// (one per channel). This way we save one memcpy operation and we
// don't need to rely on cudaMemcpy2D. The last preprocessing
// operation will write the separate channels directly to the input
// layer.
void Classifier::WrapInputLayer(std::vector<cv::Mat>* target_channels) {
    Blob<float>* input_layer_target = net_->input_blobs()[0];

    int target_width = input_layer_target->width();
    int target_height = input_layer_target->height();
    float* target_data = input_layer_target->mutable_cpu_data();
    for (int i = 0; i < input_layer_target->channels(); ++i) {
            cv::Mat channel(target_height, target_width, CV_32FC1, target_data);
            target_channels->push_back(channel);
            target_data += target_width * target_height;
        }
}

void Classifier::Preprocess(const cv::Mat& img, std::vector<cv::Mat>* input_channels) {
    // Convert the input image to the input image format of the network.
    cv::Mat sample;
    if (img.channels() == 3 && num_channels_ == 1)
        cv::cvtColor(img, sample, CV_BGR2GRAY);
    else if (img.channels() == 4 && num_channels_ == 1)
        cv::cvtColor(img, sample, CV_BGRA2GRAY);
    else if (img.channels() == 4 && num_channels_ == 3)
        cv::cvtColor(img, sample, CV_BGRA2BGR);
    else if (img.channels() == 1 && num_channels_ == 3)
        cv::cvtColor(img, sample, CV_GRAY2BGR);
    else
        sample = img;

    // Convert the input image to the expected size.
    cv::Mat sample_resized;
    if (sample.size() != input_geometry_)
        cv::resize(sample, sample_resized, input_geometry_);
    else
        sample_resized = sample;

    // Convert the input image to the expected number of channels.
    cv::Mat sample_float;
    if (num_channels_ == 3)
        sample_resized.convertTo(sample_float, CV_32FC3);
    else
        sample_resized.convertTo(sample_float, CV_32FC1);

    // Subtract the image mean to try to make the input 0-mean.
    cv::Mat sample_normalized;
    cv::subtract(sample_float, mean_, sample_normalized);

    // This operation will write the separate BGR planes directly to the
    // input layer of the network because it is wrapped by the cv::Mat
    // objects in input_channels.
    cv::split(sample_normalized, *input_channels);

    /*CHECK(reinterpret_cast<float*>(input_channels->at(0).data)
        == net_->input_blobs()[0]->cpu_data())
    << "Input channels are not wrapping the input layer of the network.";*/
}

void Classifier::GetOutput(int &id, float &trust) {
    // Get the fc8 output features of the network (this contains the estimated bounding box).
    GetFeatures("prob", id, trust);
}

void Classifier::GetFeatures(const string& feature_name, int &id, float &trust) {
    //printf("Getting %s features\n", feature_name.c_str());

    // Get a pointer to the requested layer.
    const boost::shared_ptr<Blob<float> > layer = net_->blob_by_name(feature_name.c_str());

    // Copy all elements in this layer to a vector.
    const float* begin = layer->cpu_data();

    std::vector<float> featureList;
    featureList.resize(layer->offset(1));

    for (int i=0;i<layer->offset(1);i++)
        {
            featureList[i]=begin[i];
        }
    std::vector<int> top_ids = Argmax(featureList, featureList.size());
    // only output the id with highest trust
    id = top_ids[0];
    trust = featureList[id];
}

bool PairCompare(std::pair<float, int>& lhs, std::pair<float, int>& rhs)
{
  return lhs.first > rhs.first;
}

/* Return the indices of the top N values of vector v. */
std::vector<int> Classifier::Argmax(std::vector<float> v, int N) {
    std::vector<std::pair<float, int> > pairs;
    for (size_t i = 0; i < v.size(); ++i)
        pairs.push_back(std::make_pair(v[i], i));
    std::partial_sort(pairs.begin(), pairs.begin() + N, pairs.end(), PairCompare);

    std::vector<int> result;
    for (int i = 0; i < N; ++i)
        result.push_back(pairs[i].second);
    return result;
}


