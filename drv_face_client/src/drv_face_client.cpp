#include <ros/ros.h>

#include <math.h>
#include <iostream>
#include <fstream>

#include <std_msgs/Bool.h>
#include <std_msgs/UInt16MultiArray.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <drv_msgs/recognized_faces.h>
#include <drv_msgs/face_recognize.h>

#include "facedetector.h"

using namespace std;

ros::Publisher faceRecogPubStatus_;
ros::Publisher facePubFace_;
image_transport::Publisher facePubImage_;

enum ModeType{m_wander, m_search, m_track};
int modeType_ = m_wander;
string param_running_mode = "/status/running_mode";

string param_need_recognize = "/vision/face/need_recognize";
bool needRecognize_ = false;

vector<string> names;

char* drv_path_env = std::getenv("DRV");
string drv_path_ = std::string(drv_path_env);
string name_path_ = drv_path_ + "/supplements/face_recognize/names.txt";

cv_bridge::CvImagePtr imagePtr_;

bool faceSearchResult_ = false;


void imageCallback(const sensor_msgs::ImageConstPtr & image_msg)
{
  // for now, face recognition run in wander mode
  if (modeType_ != m_wander)
    return;

  imagePtr_ = cv_bridge::toCvCopy(image_msg, "bgr8");
}

void resetStatus()
{
  needRecognize_ = false;
}

void imgToSensorMsg(vector<Mat> face_imgs, vector<sensor_msgs::Image> &face_msgs)
{
  for (size_t i = 0; i < face_imgs.size(); i++) {
    sensor_msgs::Image face_msg;
    cv_bridge::CvImage cv_face;
    cv_face.encoding = sensor_msgs::image_encodings::BGR8;
    cv_face.image = face_imgs[i];
    cv_face.toImageMsg(face_msg);
    face_msgs.push_back(face_msg);
  }
}

void drawText(Mat &img, vector<Rect> face_roi, vector<std_msgs::String> names)
{
  // here we assume all rois contain face, known or unknown
  if (face_roi.size() != names.size()) {
    ROS_ERROR("Number of ROIs and names mismatch.");
    faceSearchResult_ = false;
    return;
  }
  for (size_t i = 0; i < names.size(); i++) {
    Scalar color = Scalar(0, 255, 0); // bgr order, don't miss 'Scalar'
    putText(img, names[i].data,
            Point(face_roi[i].x + 20, face_roi[i].y + face_roi[i].height - 20),
            1, 1, color, 2);
  }
  faceSearchResult_ = true;
}

bool loadNames()
{
  ifstream name_file;
  name_file.open(name_path_.c_str());

  string line;
  if (name_file.is_open()) {
    names.push_back("Unknown");
    while (getline (name_file, line))
      names.push_back(line);
    name_file.close();
    return true;
  }
  else
    return false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drv_face_client");

  ros::NodeHandle nh;
  ros::NodeHandle pnh;
  ros::NodeHandle rgb_nh(nh, "rgb");
  ros::NodeHandle rgb_pnh(pnh, "rgb");
  image_transport::ImageTransport it_rgb_sub(rgb_nh);
  image_transport::TransportHints hints_rgb("compressed", ros::TransportHints(), rgb_pnh);

  faceRecogPubStatus_ = nh.advertise<std_msgs::Bool>("status/face/feedback", 1);
  facePubFace_ = nh.advertise<drv_msgs::recognized_faces>("face/recognized_faces", 1);
  image_transport::ImageTransport rgb_it(nh);
  facePubImage_ = rgb_it.advertise("search/labeled_image", 1);

  image_transport::Subscriber sub_rgb = it_rgb_sub.subscribe("/vision/rgb/image_rect_color", 1,
                                                             imageCallback, hints_rgb);

  ros::ServiceClient client = nh.serviceClient<drv_msgs::face_recognize>("drv_face_service");

  FaceDetector fd(drv_path_);
  if (!loadNames()) {
    ROS_WARN("No name file, maybe train the network first.");
    return 0;
  }

  ROS_INFO("Face recognition function initialized!");

  while (ros::ok())
  {
    if (ros::param::has(param_running_mode))
      ros::param::get(param_running_mode, modeType_);

    if (modeType_ != m_wander)
    {
      resetStatus();
      continue;
    }

    if (ros::param::has(param_need_recognize))
      ros::param::get(param_need_recognize, needRecognize_);

    ros::spinOnce(); // get source image

    if (!needRecognize_)
      continue;

    if (imagePtr_ == NULL)
      continue;

    // get face region from source image, if no face found, continue
    Mat img_out;
    vector<Rect> face_roi;
    std::vector<Mat> face_imgs;
    if (!fd.Process(imagePtr_->image, img_out, face_roi, face_imgs))
      continue;

    vector<sensor_msgs::Image> face_msgs;
    imgToSensorMsg(face_imgs, face_msgs);

    // call face recognize service
    drv_msgs::face_recognize srv;

    srv.request.images_in = face_msgs;

    if (client.call(srv)) {
      drv_msgs::recognized_faces rf;
      rf.name_ids = srv.response.face_label_ids;
      for (size_t i = 0; i < rf.name_ids.size(); i++)
      {
        std_msgs::String name;
        name.data = names[rf.name_ids[i]];
        rf.names.push_back(name);
      }
      facePubFace_.publish(rf);

      // put name string on image and publish
      drawText(img_out, face_roi, rf.names);
      cv_bridge::CvImage cv_pub;
      cv_pub.encoding = sensor_msgs::image_encodings::BGR8;
      cv_pub.image = img_out;
      facePubImage_.publish(cv_pub.toImageMsg());
    }
    else {
      faceSearchResult_ = false;
      ROS_ERROR("Failed to call face recognize service.");
    }

    needRecognize_ = false;

    std_msgs::Bool flag;
    flag.data = faceSearchResult_;
    faceRecogPubStatus_.publish(flag);
  }

  return 0;
}


