#ifndef UTILITIES_H
#define UTILITIES_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16MultiArray.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>

#include <cv_bridge/cv_bridge.h>
#include <cv.h>


class Utilities
{
public:
  Utilities();

  cv_bridge::CvImageConstPtr imagePtr_;
  cv_bridge::CvImageConstPtr imageDepthPtr_;

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  image_transport::SubscriberFilter subImage_;
  image_transport::SubscriberFilter subDepth_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> subCameraInfo_;

  void imgCb(const sensor_msgs::ImageConstPtr& image,
             const sensor_msgs::ImageConstPtr& imageDepth,
             const sensor_msgs::CameraInfoConstPtr& cameraInfo);
};

#endif // UTILITIES_H
