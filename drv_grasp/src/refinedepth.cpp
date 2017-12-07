#include "refinedepth.h"

using caffe::Blob;
using caffe::Net;

const int feature_num = 5; // y cr cb r c
const int w_ = 640;
const int h_ = 480;

RefineDepth::RefineDepth()
{
  int gpu_id = 0;
#ifdef CPU_ONLY
  printf("Setting up Caffe in CPU mode\n");
  caffe::Caffe::set_mode(caffe::Caffe::CPU);
#else
  caffe::Caffe::SetDevice(gpu_id);
  caffe::Caffe::set_mode(caffe::Caffe::GPU);
#endif
  char* drv_path_env = getenv("DRV");
  string drv_path_ = string(drv_path_env) + "/supplements/refine_depth/";
  string proto_file;
  proto_file = drv_path_ + "refine_depth.prototxt";

  if (!boost::filesystem::is_regular_file(proto_file))
    cerr << "Proto file " << proto_file << " not exist." << endl;
  net_.reset(new Net<float>(proto_file, caffe::TEST));
  setParams();
}

void RefineDepth::refineDepth(Mat rgb_in, Mat depth_in, Mat &depth_out)
{
  vector<float> depth_values_in;

  vector<vector<float> > features_in;
  vector<vector<float> > features_out;

  depth_values_in.resize(w_ * h_);
  features_in.resize(feature_num);
  features_out.resize(feature_num);

  for (size_t e = 0; e < feature_num; ++e) {
    features_in[e].resize(w_ * h_);
    features_out[e].resize(w_ * h_);
  }

  CHECK_EQ(rgb_in.size(), Size(w_, h_));
  CHECK_EQ(depth_in.size(), Size(w_, h_));

  cv::Mat yCrCb;
  cv::cvtColor(rgb_in, yCrCb, CV_BGR2YCrCb);

  for (size_t r = 0; r < h_; ++r) {
    for (size_t c = 0; c < w_; ++c) {
      float depth_val = depth_in.at<float>(r, c);
      if (depth_val == 0) {
        // In place where depth is not known, put its features to features_out
        features_out[0][r * w_ + c] = yCrCb.at<Vec3b>(r, c)[0];
        features_out[1][r * w_ + c] = yCrCb.at<Vec3b>(r, c)[1];
        features_out[2][r * w_ + c] = yCrCb.at<Vec3b>(r, c)[2];
        features_out[3][r * w_ + c] = r;
        features_out[4][r * w_ + c] = c;
      }
      else {
        features_in[0][r * w_ + c] = yCrCb.at<Vec3b>(r, c)[0];
        features_in[1][r * w_ + c] = yCrCb.at<Vec3b>(r, c)[1];
        features_in[2][r * w_ + c] = yCrCb.at<Vec3b>(r, c)[2];
        features_in[3][r * w_ + c] = r;
        features_in[4][r * w_ + c] = c;

        depth_values_in[r * w_ + c] = depth_val;
      }
    }
  }

  depth_out = Mat(h_, w_, CV_32FC1, Scalar(0.0));

  Mat depth_filled(h_, w_, CV_32FC1, Scalar(0.0));
  forward(depth_values_in, features_in, features_out, depth_filled);

  for (size_t r = 0; r < h_; ++r) {
    for (size_t c = 0; c < w_; ++c) {
      float depth_val = depth_in.at<float>(r, c);
      if (depth_val == 0) {
        depth_out.at<float>(r, c) = depth_filled.at<float>(r, c);
      }
      else {
        depth_out.at<float>(r, c) = depth_val;
      }
    }
  }
  namedWindow("Refined", WINDOW_NORMAL);
  imshow("Refined", depth_out);
  waitKey();
}

void RefineDepth::forward(vector<float> depth_in, vector<vector<float> > features_in,
                          vector<vector<float> > features_out, Mat &depth_out)
{
  wrapInputLayer(depth_in, features_in, features_out);
  net_->Forward();
  getOutput(depth_out);
}

void RefineDepth::setParams()
{
  const vector<boost::shared_ptr<Blob<float> > >& net_params = net_->params();
  boost::shared_ptr<Blob<float> > out_seg1 = net_params[0];
  float* out_seg1_data = out_seg1->mutable_cpu_data();
  // dim of out_seg1 param: 1 X 1 X 1 X 1
  out_seg1_data[0] = 1.0;

  boost::shared_ptr<Blob<float> > out_seg2 = net_params[2];
  float* out_seg2_data = out_seg2->mutable_cpu_data();
  // dim of out_seg1 param: 1 X 1 X 1 X 1
  out_seg2_data[0] = 1.0;

  boost::shared_ptr<Blob<float> > sp_out_seg1 = net_params[4];
  float* sp_out_seg1_data = sp_out_seg1->mutable_cpu_data();
  // dim of sp_out_seg1 param: 1 X 2 X 1 X 1
  sp_out_seg1_data[0] = 1.0; // This value only need be larger
  sp_out_seg1_data[1] = 0.0; // than this value
}

void RefineDepth::wrapInputLayer(vector<float> depth_in, vector<vector<float> > features_in,
                                 vector<vector<float> > features_out)
{
  // Input depth shape: num_pixel
  // Net shape: 1 1 1 num_pixel
  Blob<float>* input_unary = net_->input_blobs()[0];
  float* unary_data = input_unary->mutable_cpu_data();

  for (size_t e = 0; e < depth_in.size(); ++e) {
    *unary_data = depth_in[e];
    ++unary_data;
  }

  // Input in_features shape : feature_num_ num_pixel
  // Net shape : 1 feature_num_ 1 num_pixel
  Blob<float>* input_in_features = net_->input_blobs()[1];
  float* in_features_data = input_in_features->mutable_cpu_data();

  vector<float> in_features_c; // collection of data in in_features
  int sz = w_ * h_;
  for (size_t c = 0; c < feature_num; ++c) {
    for (size_t e = 0; e < sz; ++e) {
      in_features_c.push_back(features_in[c][e]);
    }
  }
  for (size_t e = 0; e < in_features_c.size(); ++e) {
    *in_features_data = in_features_c[e];
    ++in_features_data;
  }

  // Input out_features shape: feature_num_ num_pixel
  // Net shape : 1 feature_num_ 1 num_pixel
  Blob<float>* input_out_features = net_->input_blobs()[2];
  float* out_features_data = input_out_features->mutable_cpu_data();

  vector<float> out_features_c; // collection of data in out_features
  for (size_t c = 0; c < feature_num; ++c) {
    for (size_t e = 0; e < sz; ++e) {
      out_features_c.push_back(features_out[c][e]);
    }
  }
  for (size_t e = 0; e < out_features_c.size(); ++e) {
    *out_features_data = out_features_c[e];
    ++out_features_data;
  }

  // Scales1 1 feature_num_ 1 1
  Blob<float>* input_scales1 = net_->input_blobs()[3];
  float* scales1_data = input_scales1->mutable_cpu_data();
  // Scales2 1 feature_num_ 1 1
  Blob<float>* input_scales2 = net_->input_blobs()[4];
  float* scales2_data = input_scales2->mutable_cpu_data();

  float s1[] = {0.06, 0.2, 0.2, 0.02, 0.02};
  float s2[] = {0.09, 0.3, 0.3, 0.03, 0.03};
  vector<float> scales1(s1, s1 + sizeof(s1) / sizeof(float));
  for (size_t e = 0; e < feature_num; ++e) {
    *scales1_data = scales1[e];
    ++scales1_data;
  }
  vector<float> scales2(s2, s2 + sizeof(s2) / sizeof(float));
  for (size_t e = 0; e < feature_num; ++e) {
    *scales2_data = scales2[e];
    ++scales2_data;
  }
}

void RefineDepth::getOutput(Mat &depth_out)
{
  int num_elements = 1;
  const string out_depth_name = "out_depth1";
  const boost::shared_ptr<Blob<float> > out_depth_blob =
      net_->blob_by_name(out_depth_name.c_str());

  num_elements = 1;
  for (int i = 0; i < out_depth_blob->num_axes(); ++i) {
    const int elements_in_dim = out_depth_blob->shape(i);
    num_elements *= elements_in_dim;
  }

  vector<float> temp_depth;
  for (int n = 0; n < num_elements; ++n) {
    const float* out_depth_blob_data = out_depth_blob->cpu_data() + n;
    temp_depth.push_back(*out_depth_blob_data);
  }

  for (int i = 0; i < temp_depth.size(); ++i) {
    int r = i / w_;
    int c = i % w_;
    depth_out.at<float>(r, c) = float(temp_depth[i]);
  }
}
