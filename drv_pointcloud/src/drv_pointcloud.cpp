#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>

#include <pcl_ros/point_cloud.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <image_geometry/pinhole_camera_model.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>

// dynamic configure
#include <dynamic_reconfigure/server.h>
#include <drv_pointcloud/pointcloudConfig.h>

#include <sstream>

#include <opencv2/imgproc/imgproc.hpp>

#include "getsourcecloud.h"

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

namespace enc = sensor_msgs::image_encodings;


float fx_ = 538.77;
float fy_ = 540.23;
float cx_ = 314.76;
float cy_ = 239.95;

double min_depth_ = 0.5;
double max_depth_ = 2.5;

bool approxSync = true;

ros::Publisher cloudPub_ ;
ros::Publisher cloudHeaderPub_;
std::string pointCloudSourceTopic_ = "/point_cloud";

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MyApproxSyncDepthPolicy;
message_filters::Synchronizer<MyApproxSyncDepthPolicy> * approxSyncDepth_;


typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo> MyExactSyncDepthPolicy;
message_filters::Synchronizer<MyExactSyncDepthPolicy> * exactSyncDepth_;


void processAndPublish(PointCloud::Ptr & pclCloud, const std_msgs::Header & header)
{
		sensor_msgs::PointCloud2 rosCloud;
		pcl::toROSMsg(*pclCloud, rosCloud);
		rosCloud.header.stamp = header.stamp;
		rosCloud.header.frame_id = header.frame_id;

		cloudPub_.publish(rosCloud);
}

void configCallback(drv_pointcloud::pointcloudConfig &config, uint32_t level)
{
    ROS_INFO("Reconfigure Request: min depth: %f, max depth: %f.\n", config.min_depth, config.max_depth);

    min_depth_ = config.min_depth;
    max_depth_ = config.max_depth;
}

void depthCallback(
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

		if(cloudHeaderPub_.getNumSubscribers())// only publish when someone is hearing
				{
						cv_bridge::CvImageConstPtr imagePtr;
						if(image->encoding.compare(sensor_msgs::image_encodings::TYPE_8UC1)==0)
								{
										imagePtr = cv_bridge::toCvShare(image);
								}
						else if(image->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
										image->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0)
								{
										imagePtr = cv_bridge::toCvShare(image, "mono8");
								}
						else
								{
										imagePtr = cv_bridge::toCvShare(image, "bgr8");
								}

						cv_bridge::CvImageConstPtr imageDepthPtr = cv_bridge::toCvShare(imageDepth);

						image_geometry::PinholeCameraModel model;
						model.fromCameraInfo(*cameraInfo);
						fx_ = model.fx();
						fy_ = model.fy();
						cx_ = model.cx();
						cy_ = model.cy();

						PointCloud::Ptr pclCloud (new PointCloud());

						if (GetSourceCloud::getCloud(imagePtr->image, imageDepthPtr->image, fx_, fy_, cx_, cy_, max_depth_, min_depth_, pclCloud))
								{
										cloudHeaderPub_.publish(imagePtr->header);
										processAndPublish(pclCloud, imagePtr->header);
										///pclCloud size:640*480 isdense:false isorgnized;true
								}
						else
								{
										ROS_ERROR("Get cloud failed!\n");
								}
				}
}


int main(int argc, char **argv)
{
		ros::init(argc, argv, "drv_pointcloud");

		ros::NodeHandle nh;
		ros::NodeHandle pnh;

		// set up dynamic reconfigure callback
		dynamic_reconfigure::Server<drv_pointcloud::pointcloudConfig> server;
		dynamic_reconfigure::Server<drv_pointcloud::pointcloudConfig>::CallbackType f;

		f = boost::bind(&configCallback, _1, _2);
		server.setCallback(f);

		// set up source point cloud publisher
		cloudHeaderPub_ = nh.advertise<std_msgs::Header>("point_cloud/header", 3);
		cloudPub_ = nh.advertise<sensor_msgs::PointCloud2>(pointCloudSourceTopic_, 1);

		// listen to image topic and get cloud
		int queueSize = 5;
		image_transport::SubscriberFilter imageSub_;
		image_transport::SubscriberFilter imageDepthSub_;
		message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoSub_;

		ros::NodeHandle rgb_nh(nh, "rgb");
		ros::NodeHandle depth_nh(nh, "depth");
		ros::NodeHandle rgb_pnh(pnh, "rgb");
		ros::NodeHandle depth_pnh(pnh, "depth");
		image_transport::ImageTransport rgb_it(rgb_nh);
		image_transport::ImageTransport depth_it(depth_nh);
		// !Use compressed message to speed up -- necessary!
		image_transport::TransportHints hintsRgb("compressed", ros::TransportHints(), rgb_pnh);
		image_transport::TransportHints hintsDepth("compressedDepth", ros::TransportHints(), depth_pnh);

		imageSub_.subscribe(rgb_it, rgb_nh.resolveName("image"), 1, hintsRgb);
		imageDepthSub_.subscribe(depth_it, depth_nh.resolveName("image"), 1, hintsDepth);
		cameraInfoSub_.subscribe(rgb_nh, "camera_info", 1);

		if(approxSync)
				{
						approxSyncDepth_ = new message_filters::Synchronizer<MyApproxSyncDepthPolicy>(MyApproxSyncDepthPolicy(queueSize), imageSub_, imageDepthSub_, cameraInfoSub_);
						approxSyncDepth_->registerCallback(depthCallback);
				}
		else
				{
						exactSyncDepth_ = new message_filters::Synchronizer<MyExactSyncDepthPolicy>(MyExactSyncDepthPolicy(queueSize), imageSub_, imageDepthSub_, cameraInfoSub_);
						exactSyncDepth_->registerCallback(depthCallback);
				}

		ROS_INFO("Publishing pointcloud initialized.\n");

		while (ros::ok())
				{
						ros::spinOnce();
				}

		return 0;
}

