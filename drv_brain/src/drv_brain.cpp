#include <ros/ros.h>
#include <ros/console.h>

#include <math.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

//Custom message
#include <drv_msgs/target_info.h>

#include <stdio.h>

#include "androidlistener.h"
#include "facelistener.h"
#include "targetlistener.h"

using namespace std;

/* global params */
// feedback
string param_vision_feedback = "/comm/param/feedback/vision/overall"; // 0:not working, 1:working, 2:failed, 3:finished with success
string param_vision_feedback_mode = "/comm/param/feedback/vision/mode"; // 0:wandering, 1:searching, 2:tracking
string param_vision_feedback_search = "/comm/param/feedback/vision/search"; // 0:no search, 1:found, -1:current not found. -2:around not found
string param_vision_feedback_track = "/comm/param/feedback/vision/track"; // 0:no track, 1:tracking, -1:lost
string param_vision_feedback_grasp = "/comm/param/feedback/vision/grasp"; // 0:no grasp, 1:found location, -1:can't find location
string param_vision_feedback_face = "/comm/param/feedback/vision/face"; // 0:no face recognize, 1:found faces, -1:failed to recognize
// vision shared
string param_vision_shared_switch = "/comm/param/shared/vision/switch"; // true:switch on to execute tasks, false:switch off and run in wander mode

bool centralSwitch_ = true; // main switch

// target properties
bool targetSetTemp = false;
string param_target_label = "/vision/target/label";
enum TargetType{t_null, t_onTable, t_onGround, t_onHead, t_onHand};
string targetTypeName[5] = {"in air", "on the table", "on the ground", "on the face", "in the hand"};
int tgtType_ = t_null;
string param_target_type = "/status/target/type";


// target status control
string targetLabel_ = "";
bool isTargetSet_ = false;

//target status feedback
bool foundTarget_ = false;

// publish servo initial position
ros::Publisher servoPub_;
bool servo_initialized_ = false;

// servo position angle
int pitchAngle_ = 70;
int yawAngle_ = 90;
string param_servo_pitch = "/status/servo/pitch";
string param_servo_yaw = "/status/servo/yaw";

// mode control params
enum ModeType{m_wander, m_search, m_track};
int modeType_ = m_wander;
int modeTypeTemp_ = -1;
string modeName[3] = {"wandering", "searching", "tracking"};
string param_running_mode = "/status/running_mode";
ros::Publisher drvPubMode_; // vision system mode publisher

//general infomation publisher
ros::Publisher drvPubInfo_;


void pubServo(int pitch_angle, int yaw_angle)
{
  std_msgs::UInt16MultiArray array;
  array.data.push_back(pitch_angle);
  array.data.push_back(yaw_angle);
  servoPub_.publish(array);
}

void pubInfo(string info)
{
  ROS_INFO(info.c_str());
  std_msgs::String msg;
  msg.data = info;
  drvPubInfo_.publish(msg);
}

void teleOpCallback(const std_msgs::Int32MultiArrayConstPtr &msg)
{
  if (msg->data.empty())
    return;

  int pitch_temp = pitchAngle_ + msg->data[0];
  if (pitch_temp < 40 || pitch_temp > 130)
    return;
  else
    pitchAngle_ = pitch_temp;

  int yaw_temp = yawAngle_ - msg->data[1];
  if (yaw_temp < 0 || pitch_temp > 180)
    return;
  else
    yawAngle_ = yaw_temp;

  pubServo(pitchAngle_, yawAngle_);
}

void servoCallback(const std_msgs::UInt16MultiArrayConstPtr &msg)
{
  // this callback should always active
  pitchAngle_ = msg->data[0];
  yawAngle_ = msg->data[1];

  ros::param::set(param_servo_pitch, pitchAngle_);
  ros::param::set(param_servo_yaw, yawAngle_);
}

void searchCallback(const std_msgs::Int8ConstPtr &msg)
{
  if (modeType_ == m_search)
  {
    if (msg->data == -1)
    {
      pubInfo("Search around didn't get target, continue searching...");
      foundTarget_ = false;
      ros::param::set(param_vision_feedback_search, -2);
      ros::param::set(param_vision_feedback, 2);
    }
    else if (msg->data == 0)
    {
      pubInfo("Currently didn't find target, continue searching...");
      ros::param::set(param_vision_feedback_search, -1);
      ros::param::set(param_vision_feedback, 1);
      foundTarget_ = false;
    }
    else
    {
      foundTarget_ = true;
      ros::param::set(param_vision_feedback_search, 1);
      ros::param::set(param_vision_feedback, 1);
    }
  }
}


void trackCallback(const std_msgs::BoolConstPtr &msg)
{
  if (modeType_ == m_track)
  {
    if (!msg->data)
    {
      ROS_INFO_THROTTLE(21, "Target lost!");
      foundTarget_ = false;
      ros::param::set(param_vision_feedback_track, -1);
      ros::param::set(param_vision_feedback, 1);
    }
    else
    {
      ROS_INFO_THROTTLE(21, "Tracking the target...");
      foundTarget_ = true;
      ros::param::set(param_vision_feedback_track, 1);
      ros::param::set(param_vision_feedback, 1);
    }
  }
}

void graspCallback(const std_msgs::BoolConstPtr &msg)
{
  if (modeType_ == m_track)
  {
    if (!msg->data)
    {
      ROS_INFO_THROTTLE(21, "Failed to locate the target.");
      ros::param::set(param_vision_feedback_grasp, -1);
      ros::param::set(param_vision_feedback, 1);
    }
    else
    {
      ROS_INFO_THROTTLE(21, "Target location confirmed.");
      ros::param::set(param_vision_feedback_grasp, 1);
      ros::param::set(param_vision_feedback, 3);
    }
  }
}

void faceRecognizeCallback(const std_msgs::BoolConstPtr &msg)
{
  if (modeType_ == m_wander)
  {
    if (!msg->data)
    {
      pubInfo("Failed to recognize face.");
      ros::param::set(param_vision_feedback_face, -1);
      ros::param::set(param_vision_feedback, 2);
    }
    else
    {
      pubInfo("Face recognition finished.");
      ros::param::set(param_vision_feedback_face, 1);
      ros::param::set(param_vision_feedback, 3);
    }
  }
}


void resetStatus()
{
  // reset global params
  ros::param::set(param_vision_feedback, 0);
  ros::param::set(param_vision_feedback_search, 0);
  ros::param::set(param_vision_feedback_track, 0);
  ros::param::set(param_vision_feedback_grasp, 0);
  ros::param::set(param_vision_feedback_face, 0);

  tgtType_ = t_null;
  targetLabel_ = "";

  isTargetSet_ = false;
  foundTarget_ = false;

  modeType_ = m_wander;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drv_brain");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  servoPub_ = nh.advertise<std_msgs::UInt16MultiArray>("servo", 1, true);
  drvPubMode_ = nh.advertise<std_msgs::String>("/comm/msg/vision/mode", 1); // used for mode info output
  drvPubInfo_ = nh.advertise<std_msgs::String>("/comm/msg/vision/info", 1);

  // don't change the order without reason
  ros::Subscriber sub_servo_ctrl = nh.subscribe<std_msgs::Int32MultiArray>("/joy_teleop/servo", 2, teleOpCallback);
  ros::Subscriber sub_servo = nh.subscribe<std_msgs::UInt16MultiArray>("servo", 1, servoCallback);
  //		ros::Subscriber sub_tgt = nh.subscribe<drv_msgs::target_info>("recognize/target", 1, targetCallback);
  ros::Subscriber sub_sh = nh.subscribe<std_msgs::Int8>("status/search/feedback", 1, searchCallback);
  ros::Subscriber sub_tk = nh.subscribe<std_msgs::Bool>("status/track/feedback", 1, trackCallback);
  ros::Subscriber sub_gp = nh.subscribe<std_msgs::Bool>("status/grasp/feedback", 1, graspCallback);
  ros::Subscriber sub_fr = nh.subscribe<std_msgs::Bool>("status/face/feedback", 1, faceRecognizeCallback);

  AndroidListener al;
  FaceListener fl;
  TargetListener tl;

  // initialize
  resetStatus();

  pubInfo("Deep Robot Vision system initialized!");

  while (ros::ok())
  {
    // main on/off control
    if (ros::param::has(param_vision_shared_switch))
    {
      bool temp = true;
      ros::param::get(param_vision_shared_switch, temp);
      if (temp)
      {
        ROS_WARN_COND(!centralSwitch_, "Central switch is ON.\n");
      }
      centralSwitch_ = temp;
    }

    if (!centralSwitch_)
    {
      ROS_WARN_THROTTLE(31, "Central switch is OFF.\n");
      resetStatus();
      continue;
    }

    // Initialize servo position
    if (!servo_initialized_)
    {
      pubServo(70, 90);
      servo_initialized_ = true;
      ROS_INFO("Servo initialized.\n");
    }

    // get feedback from search and track to determine is target found
    ros::spinOnce();

    // get target if params were set
    tl.getTargetStatus(isTargetSet_, targetLabel_);
    ros::param::set(param_target_label, targetLabel_);
    if (isTargetSet_ != targetSetTemp)
    {
      if (isTargetSet_)
      {
        pubInfo("Target set to be '" + targetLabel_ + "'.");
      }
      else
      {
        pubInfo("Target cancelled.");
        resetStatus();
      }
      targetSetTemp = isTargetSet_;
    }

    // if user select target on cellphone, publish the target
    al.publishOnceIfTargetSelected(isTargetSet_, foundTarget_);

    //mode selection, NOTICE that modeType_ should only be set by central control
    if (isTargetSet_)
    {
      if (foundTarget_)
      {
        modeType_ = m_track;
      }
      else
      {
        if (targetLabel_ == "user selected object")
          resetStatus();
        else
          modeType_ = m_search;
      }
    }
    else
    {
      modeType_ = m_wander;
      fl.isNeedRecognizeFace(); // only recognize face in wander mode
    }

    // set mode
    ros::param::set(param_running_mode, modeType_);

    if (modeType_ != modeTypeTemp_)
    {
      std_msgs::String mode_msg;
      mode_msg.data = modeName[modeType_];
      drvPubMode_.publish(mode_msg);
      ROS_INFO("Current mode: %s.\n", modeName[modeType_].c_str());
      modeTypeTemp_ = modeType_;
    }
  }

  return 0;
}

