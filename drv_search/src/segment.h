#ifndef SEGMENT_H
#define SEGMENT_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16MultiArray.h>

#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

class Segment
{
public:
    Segment();

    void segment(cv_bridge::CvImageConstPtr imagePtr, cv_bridge::CvImageConstPtr depthPtr);
    void cannyDetect(cv::Mat img_in, cv::Mat &img_out, int t1, int t2);

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    image_transport::Publisher pubImage_;
};

#endif // SEGMENT_H
