#include "utilities.h"

using namespace cv;

Utilities::Utilities()
{
  ros::NodeHandle rgb_nh(nh_, "rgb");
  ros::NodeHandle depth_nh(nh_, "depth");
  ros::NodeHandle rgb_pnh(pnh_, "rgb");
  ros::NodeHandle depth_pnh(pnh_, "depth");
  image_transport::ImageTransport rgb_it(rgb_nh);
  image_transport::ImageTransport depth_it(depth_nh);
  // !Use compressed message to speed up -- necessary!
  image_transport::TransportHints hintsRgb("compressed", ros::TransportHints(), rgb_pnh);
  image_transport::TransportHints hintsDepth("compressedDepth", ros::TransportHints(), depth_pnh);

  subImage_.subscribe(rgb_it, rgb_nh.resolveName("image"), 1, hintsRgb);
  subDepth_.subscribe(depth_it, depth_nh.resolveName("image"), 1, hintsDepth);
  subCameraInfo_.subscribe(rgb_nh, "camera_info", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MyApproxSyncDepthPolicy;
  message_filters::Synchronizer<MyApproxSyncDepthPolicy> * approxSyncDepth_;

  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MyExactSyncDepthPolicy;
  message_filters::Synchronizer<MyExactSyncDepthPolicy> * exactSyncDepth_;

  bool approxSync = true;
  int queueSize = 5;

  if(approxSync)
  {
    approxSyncDepth_ = new message_filters::Synchronizer<MyApproxSyncDepthPolicy>(MyApproxSyncDepthPolicy(queueSize),
                                                                                  subImage_, subDepth_, subCameraInfo_);
    approxSyncDepth_->registerCallback( boost::bind( &Utilities::imgCb, this, _1 ,  _2 ,  _3) );
  }
  else
  {
    exactSyncDepth_ = new message_filters::Synchronizer<MyExactSyncDepthPolicy>(MyExactSyncDepthPolicy(queueSize),
                                                                                subImage_, subDepth_, subCameraInfo_);
    exactSyncDepth_->registerCallback(  boost::bind( &Utilities::imgCb, this, _1 ,  _2 ,  _3) );
  }
}

void Utilities::imgCb(
    const sensor_msgs::ImageConstPtr& image,
    const sensor_msgs::ImageConstPtr& imageDepth,
    const sensor_msgs::CameraInfoConstPtr& cameraInfo)
{
  if(!(image->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1) ==0 ||
       image->encoding.compare(sensor_msgs::image_encodings::MONO8) ==0 ||
       image->encoding.compare(sensor_msgs::image_encodings::MONO16) ==0 ||
       image->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
       image->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0) &&
     !(imageDepth->encoding.compare(sensor_msgs::image_encodings::TYPE_16UC1)==0 ||
       imageDepth->encoding.compare(sensor_msgs::image_encodings::TYPE_32FC1)==0 ||
       imageDepth->encoding.compare(sensor_msgs::image_encodings::MONO16)==0))
  {
    ROS_ERROR("Input type must be image=mono8,mono16,rgb8,bgr8 and image_depth=32FC1,16UC1,mono16");
    return;
  }

  if(image->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1)==0)
  {
    imagePtr_ = cv_bridge::toCvShare(image);
  }
  else if(image->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
          image->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0)
  {
    imagePtr_ = cv_bridge::toCvShare(image, "mono8");
  }
  else
  {
    imagePtr_ = cv_bridge::toCvShare(image, "bgr8");
  }

  imageDepthPtr_ = cv_bridge::toCvShare(imageDepth);
}
