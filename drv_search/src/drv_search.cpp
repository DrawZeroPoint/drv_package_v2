#include <ros/ros.h>

#include <math.h>

#include <std_msgs/Int8.h>
#include <std_msgs/UInt16MultiArray.h>

#include <sensor_msgs/CompressedImage.h>

#include <drv_msgs/recognized_target.h>
#include <drv_msgs/recognize.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include "search.h"
#include "targetselect.h"
#include "smoothservo.h"

using namespace std;

string param_target_label = "/vision/target/label";

ros::Publisher searchPubServo_; // publish servo angle rotated for search
ros::Publisher searchPubStatus_;
ros::Publisher searchPubTarget_; // publish target info to track function
//image_transport::Publisher searchPubImage_; // publish labeled image for user judgement

enum ModeType{m_wander, m_search, m_track};
int modeType_ = m_wander; // record current mode status
string param_running_mode = "/status/running_mode";


// servo position angle
string param_servo_pitch = "/status/servo/pitch";
string param_servo_yaw = "/status/servo/yaw";
int yawAngle_ = 90;
int pitchAngle_ = 60;

string targetLabel_;

// global status control params
bool servoInitialized_ = false;
// feedback, -1 indicate no result around,
// 0 indicate current no result, 1 has result
int searchResult_ = 0;
int selectedNum_ = 0; // selected target number

// main lock to prevent search run twice when successed
bool lock_ = false;

sensor_msgs::Image img_msg_;

cv_bridge::CvImageConstPtr imagePtr_;
cv_bridge::CvImageConstPtr depthPtr_;


void imageCallback(const sensor_msgs::ImageConstPtr & image_msg)
{
  if (modeType_ != m_search)
    return;

  if (image_msg->height != 480) {
    ROS_ERROR_THROTTLE(5, "RGB image size is wrong.");
    return;
  }

  img_msg_ = *image_msg;
  imagePtr_ = cv_bridge::toCvShare(image_msg, "bgr8");
}

void resetStatus()
{
  servoInitialized_ = false;
  searchResult_ = 0;
  selectedNum_ = 0;
}

bool checkRunningMode()
{
  if (ros::param::has(param_running_mode))
    ros::param::get(param_running_mode, modeType_);

  if (modeType_ == m_search)
    return true;
  else
  {
    resetStatus();
    return false;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drv_search");

  ros::NodeHandle nh;
  ros::NodeHandle pnh;
  ros::NodeHandle rgb_nh(nh, "rgb");
  ros::NodeHandle rgb_pnh(pnh, "rgb");
  image_transport::ImageTransport it_rgb_sub(rgb_nh);
  image_transport::TransportHints hints_rgb("compressed", ros::TransportHints(), rgb_pnh);

  searchPubStatus_ = nh.advertise<std_msgs::Int8>("status/search/feedback", 1);
  searchPubTarget_ = nh.advertise<drv_msgs::recognized_target>("search/recognized_target", 1, true);

  image_transport::Subscriber sub_rgb = it_rgb_sub.subscribe("image_rect_color", 1,
                                                             imageCallback, hints_rgb);

  ros::ServiceClient client = nh.serviceClient<drv_msgs::recognize>("drv_recognize");

  ROS_INFO("Search function initialized!");

  Search sh;
  TargetSelect ts;
  SmoothServo ss;
  //Segment sg;
  //Utilities ut;

  while (ros::ok())
  {
    if (ros::param::has(param_servo_pitch))
      ros::param::get(param_servo_pitch, pitchAngle_);

    if (ros::param::has(param_servo_yaw))
      ros::param::get(param_servo_yaw, yawAngle_);

    if (ros::param::has(param_running_mode))
      ros::param::get(param_running_mode, modeType_);

    if (!checkRunningMode())
      continue;

    // Reset the pitch value before searching
    if (!servoInitialized_) {
      ss.getCurrentServoAngle(pitchAngle_, yawAngle_);
      ss.moveServoTo(60, yawAngle_);
      pitchAngle_ = 60;
      servoInitialized_ = true;
    }

    if (ros::param::has(param_target_label))
      ros::param::get(param_target_label, targetLabel_);

    ros::spinOnce();

    // Initialize request of recognize service
    drv_msgs::recognize srv;
    srv.request.img_in = img_msg_;

    vector<std_msgs::UInt16MultiArray> bbox_arrays_;
    int choosed_id = -1;

    // Call object recognize service
    if (client.call(srv))
    {
      // re-check the running mode, incase that mode didn't change quickly
      if (!checkRunningMode())
        continue;

      cv_bridge::CvImagePtr img_labeled;
      selectedNum_ = ts.select(targetLabel_, srv.response, img_msg_, img_labeled, choosed_id);

      int a_s = srv.response.obj_info.bbox_arrays.size();
      bbox_arrays_.resize(a_s);
      bbox_arrays_ = srv.response.obj_info.bbox_arrays;

      if (selectedNum_)
        searchResult_ = 1;
      else
        searchResult_ = 0;
    }
    else {
      ROS_ERROR("Failed to call recognize service.");
      searchResult_ = 0;
    }

    ss.getCurrentServoAngle(pitchAngle_, yawAngle_);

    if (!searchResult_) {
      int pitch_angle = pitchAngle_;
      int yaw_angle = yawAngle_;
      bool has_next_pos = sh.getNextPosition(yaw_angle, pitch_angle);
      ROS_INFO("Search angle: yaw = %d, pitch = %d.", yaw_angle, pitch_angle);

      if (!has_next_pos)
        searchResult_ = -1;

      // Turn camera to the next search direction (according to the criteria above)
      ss.moveServoTo(pitch_angle, yaw_angle);
    }
    else {
      // Label the detected target with bounding area
      // sg.segment(imagePtr_, depthPtr_);

      // Publish goal info for tracking
      if (srv.response.obj_info.bbox_arrays.size() > choosed_id) {
        drv_msgs::recognized_target tgt;
        tgt.header = srv.response.obj_info.header;
        tgt.tgt_bbox_array = srv.response.obj_info.bbox_arrays[choosed_id];
        tgt.label = srv.response.obj_info.labels[choosed_id];
        searchPubTarget_.publish(tgt);
        lock_ = true;
      }
    }

    std_msgs::Int8 flag;
    flag.data = searchResult_;
    searchPubStatus_.publish(flag);
  }

  return 0;
}
