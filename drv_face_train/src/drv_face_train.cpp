#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/UInt16MultiArray.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <drv_msgs/face_train.h>

#include "facedetector.h"

#include <cstdlib>
#include <stdio.h>
#include <fstream>
#include <openssl/sha.h>
#include <rosauth/Authentication.h>
#include <sstream>
#include <string>

#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>

using namespace std;
using namespace cv;
using namespace image_transport;

char* drv_path_env = std::getenv("DRV");

string drv_path_ = string(drv_path_env) + "/supplements/face_recognize/";

string image_path_ = drv_path_ + "images/";

ros::Publisher faceTrainPubStatus_;

enum ModeType{m_wander, m_search, m_track};
int modeType_ = m_wander;
string param_running_mode = "/status/running_mode";

string param_face_need_train_ = "/vision/face/need_train";
string param_face_train_name_ = "/vision/face/train/name";

string faceName_ = "";
Mat faceROI_;

int image_num_ = 100;
int imageCount_ = 0;

bool nameAdded_ = false;
int currNameId_ = 0;

vector<string> nameVec_;

cv_bridge::CvImagePtr imagePtr_;

FaceDetector fd_(drv_path_);

// Authority
ros::ServiceClient cl_auth_;
bool need_authority_ = false;
bool authenticated_ = false;
string param_password_ = "/password";
string password_ = ""; // default: admin


void resetStatus()
{
  faceName_ = "";
  imageCount_ = 0;
  
  nameAdded_ = false;
  currNameId_ = 0;
  
  nameVec_.clear();
  
  authenticated_ = false;
}

bool checkAuthority()
{
  string rand = "xyzabc";
  ros::Time now = ros::Time::now();
  string user_level = "admin";
  ros::Time end = ros::Time::now();
  end.sec += 120;
  
  // create the string to hash
  stringstream ss;
  ss << password_ << rand << now.sec << user_level << end.sec;
  string local_hash = ss.str();
  unsigned char sha512_hash[SHA512_DIGEST_LENGTH];
  SHA512((unsigned char *)local_hash.c_str(), local_hash.length(), sha512_hash);
  
  // convert to a hex string to compare
  char hex[SHA512_DIGEST_LENGTH * 2];
  // make the request
  rosauth::Authentication srv;
  srv.request.mac = string(hex);
  srv.request.rand = rand;
  srv.request.t = now;
  srv.request.level = user_level;
  srv.request.end = end;
  
  if (!cl_auth_.call(srv)) {
    ROS_ERROR("Face train: Authentication server not actived.");
    return false;
  }
  else {
    if (!srv.response.authenticated)
      ROS_ERROR_THROTTLE(9, "Face train: Password is not correct.");
  }
  return srv.response.authenticated;
}

void saveCurrentName()
{
  nameVec_.clear();
  currNameId_ = 0;
  string file = drv_path_ + "names.txt";
  if (!boost::filesystem::exists(file)) {
    std::ofstream outfile(file.c_str());
    outfile << faceName_;
    nameVec_.push_back(faceName_);
    outfile.close();
  }
  else {
    ifstream infile(file.c_str());
    string line;
    
    bool repeated = false;
    while (getline(infile, line)) {
      nameVec_.push_back(line);
      
      if (line == faceName_) {
        ROS_WARN("Current name is already exist!");
        repeated = true;
      }
      else
        currNameId_++;
    }
    infile.close();
    
    if (!repeated) {
      ofstream outfile(file.c_str(), ofstream::app);
      outfile << endl << faceName_;
      nameVec_.push_back(faceName_);
      outfile.close();
    }
  }
}

void generateTrainList()
{
  /* Train list is a file list which contains lines in such format:
    /path/to/image.jpg image_label
    example:
    /caffe/data/images/8.jpg 1
  */
  string train_list = drv_path_ + "train.txt";
  // Cover original training file with trunc
  ofstream outfile(train_list.c_str(), ofstream::trunc);
  // image_num_ is total training image num for one person
  for (size_t k = 0; k < image_num_; ++k) {
    for (size_t i = 0; i < nameVec_.size(); ++i) {
      outfile << image_path_ << i << "/" << k << ".jpg " << i << endl;
    }
  }
  outfile.close();
}

void saveImage(Mat image, int num)
{
  std::stringstream ss;
  ss << num;
  string save_dir = image_path_ + ss.str() + "/";
  if (!boost::filesystem::exists(save_dir)) {
    boost::filesystem::create_directories(save_dir);
  }
  
  std::stringstream ssc;
  ssc << imageCount_;
  imwrite(save_dir + ssc.str() + ".jpg", image);
}

void imageCallback(const sensor_msgs::ImageConstPtr & image_msg)
{
  if (modeType_ != m_wander || !nameAdded_)
    return;
  
  imagePtr_ = cv_bridge::toCvCopy(image_msg, "bgr8");
  
  if (!imagePtr_->image.cols) {
    ROS_ERROR("Face train: No image message recieved.");
    return;
  }
  
  if (fd_.getOneFace(imagePtr_->image, faceROI_)) {
    // If captured image contains only one face, save it
    if (!faceROI_.empty()) {
      imshow("Face image", faceROI_);
      waitKey(10);
    }
    saveImage(faceROI_, currNameId_);
    ROS_INFO("Face image number %d captured.", imageCount_);
    imageCount_++;
    
    /* If captured image reached demanded number, 
     * stop capturing and try starting training */
    if (imageCount_ == image_num_) {
      ROS_INFO("Capturing face image finished.");
      resetStatus();

      ros::param::set(param_face_need_train_, true);
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "face_train_client");
  
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  // Get training image number of one person
  pnh.getParam("image_num", image_num_);
  pnh.getParam("need_authentication_id", need_authority_);
  
  ros::NodeHandle inh;
  ros::NodeHandle rgb_nh(nh, "rgb");
  ros::NodeHandle rgb_pnh(inh, "rgb");
  ImageTransport it_rgb_sub(rgb_nh);
  TransportHints hints_rgb("compressed", ros::TransportHints(), rgb_pnh);
  Subscriber sub_rgb = it_rgb_sub.subscribe("/vision/rgb/image_rect_color",
                                            1, imageCallback, hints_rgb);
  
  faceTrainPubStatus_ = nh.advertise<std_msgs::Bool>("status/face/train/feedback", 1);
  
  cl_auth_ = nh.serviceClient<rosauth::Authentication>("authenticate");
  ros::ServiceClient client = nh.serviceClient<drv_msgs::face_train>("face_train_service");
  
  ROS_INFO("Ready to train the face recognition network.");
  
  while (ros::ok()) {
    if (ros::param::has(param_running_mode))
      ros::param::get(param_running_mode, modeType_);
    
    if (modeType_ != m_wander) {
      resetStatus();
      continue;
    }
    
    if (ros::param::has(param_face_train_name_)) {
      ros::param::get(param_face_train_name_, faceName_);
      ros::param::del(param_face_train_name_);
    }
    
    if (faceName_ != "") {
      /* faceName is not null means the user wants to add 
         a new person to the dataset, so
         check if the request have authority */
      if (need_authority_ && !authenticated_) {
        ros::param::get(param_password_, password_);
        if (!checkAuthority()) {
          resetStatus();
          continue;
        }
        else
          authenticated_ = true;
      }
      
      /* If authentication is passed, check whether the name 
       * has been added, the added name will be retrained */
      if (!nameAdded_) {
        // If faceName has not been added, add it to name.txt
        // and generate training file
        saveCurrentName();
        generateTrainList();
        
        // Reset faceName after generating files needed
        faceName_ = "";
        nameAdded_ = true;
        ROS_INFO("Name added, start capturing face.");
      }
    }
    
    ros::spinOnce();
    
    bool needTrain = false;
    // After capturing all images, defaultly set needTrain=true
    if (ros::param::has(param_face_need_train_)) {
      ros::param::get(param_face_need_train_, needTrain);
    }
    
    if (needTrain) {
      // Call training service
      drv_msgs::face_train srv;
      
      bool faceTrainResult = false;
      if (client.call(srv)) {
        faceTrainResult = true;
        ROS_INFO("Face train: Finished training of face data.");
      }
      else {
        faceTrainResult = false;
        ROS_ERROR("Failed to call face train service.");
      }
      
      std_msgs::Bool flag;
      flag.data = faceTrainResult;
      faceTrainPubStatus_.publish(flag);
      
      // Reset status if training successed
      resetStatus();
      ros::param::set(param_face_need_train_, false);
      faceTrainResult = false;
    }
  }
  return 0;
}
