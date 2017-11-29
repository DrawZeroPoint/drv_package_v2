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

// Customized message
#include <drv_msgs/target_info.h>

#include <stdio.h>

#include "targetselecter.h"
#include "facelistener.h"
#include "targetlistener.h"

using namespace std;
using namespace std_msgs;

/* Global params */
// Feedback
// 0:not working, 1:working, 2:failed, 3:finished with success
string param_vision_feedback = "/comm/param/feedback/vision/overall";
// 0:wandering, 1:searching, 2:tracking
string param_vision_feedback_mode = "/comm/param/feedback/vision/mode"; 
// 0:no search, 1:found, -1:current not found. -2:around not found
string param_vision_feedback_search = "/comm/param/feedback/vision/search";
// 0:no track, 1:tracking, -1:lost
string param_vision_feedback_track = "/comm/param/feedback/vision/track"; 
// 0:no grasp, 1:found location, -1:can't find location
string param_vision_feedback_grasp = "/comm/param/feedback/vision/grasp";
// 0:no face recognize, 1:found faces, -1:failed to recognize
string param_vision_feedback_face = "/comm/param/feedback/vision/face"; 
// vision shared
// true:switch on to execute tasks, false:switch off and run in wander mode
string param_vision_shared_switch = "/comm/param/shared/vision/switch"; 

bool centralSwitch_ = true; // main switch

// Target properties
bool targetSetTemp_ = false;
// Param inside vision system
string param_target_label = "/vision/target/label";

enum TargetType{t_null, t_onTable, t_onGround, t_onHead, t_onHand};
string targetTypeName[5] = {"in air", "on the table", "on the ground", 
                            "on the face", "in the hand"};
int tgtType_ = t_null;
string param_target_type = "/status/target/type";

string param_comm_target_set =   "/comm/param/control/target/is_set";
string param_comm_target_label = "/comm/param/control/target/label";
string param_comm_is_put = "/comm/param/ctrl/is_put";  // Is put down object in hand

// Target status control
bool isTargetSet_ = false;
string targetLabel_ = "";
bool isPut_ = false;

// Target status feedback
bool foundTarget_ = false;
int putSuccess_ = 0;

// Publish servo initial position
ros::Publisher servoPub_;
bool servo_initialized_ = false;

// Servo initial angles
int pitchAngle_ = 70;
int yawAngle_ = 90;
// Range of servo angles
int pitch_min_ = 60;
int pitch_max_ = 140;
int yaw_min_ = 0;
int yaw_max_ = 180;

// Mode control params
enum ModeType{m_wander, m_search, m_track, m_put};
int modeType_ = m_wander;
int modeTypeTemp_ = -1;
string modeName[4] = {"wandering", "searching", "tracking", "putting"};
string param_running_mode = "/status/running_mode";
ros::Publisher drvPubMode_; // vision system mode publisher

// General information publisher
ros::Publisher drvPubInfo_;

// Result publisher
ros::Publisher drvPubPutResult_;

void resetStatus()
{
  // Reset global params
  ros::param::set(param_vision_feedback, 0);
  ros::param::set(param_vision_feedback_search, 0);
  ros::param::set(param_vision_feedback_track, 0);
  ros::param::set(param_vision_feedback_grasp, 0);
  ros::param::set(param_vision_feedback_face, 0);
  
  tgtType_ = t_null;
  targetLabel_ = "";
  
  isTargetSet_ = false;
  foundTarget_ = false;
  putSuccess_ = 0;
  
  modeType_ = m_wander;
  
  ros::param::set(param_comm_target_set, false);
  ros::param::set(param_comm_target_label, "");
  ros::param::set(param_comm_is_put, false);
}

/**
 * @brief pubServo This function is used for servo initialization and
 *                 publishing servo angles received from cellphone
 * @param pitch_angle 
 * @param yaw_angle
 * @param power multiplies with basic speed
 */
void pubServo(int pitch_angle, int yaw_angle, int power)
{
  std_msgs::UInt16MultiArray array;
  array.data.push_back(pitch_angle);
  array.data.push_back(yaw_angle);
  // The basic speed is 10 (0~255)
  array.data.push_back(10 * power);
  servoPub_.publish(array);
}

/**
 * @brief pubInfo Publish info for Android display
 * @param info
 */
void pubInfo(string info)
{
  ROS_INFO(info.c_str());
  String msg;
  msg.data = info;
  drvPubInfo_.publish(msg);
}

/**
 * @brief teleOpCallback
 * Receive servo value from JARVIS the Android App
 * @param msg
 * Contains 2 int representing increment in x and y direction
 * with range in [-1, 1]
 */
void teleOpCallback(const Int32MultiArrayConstPtr &msg)
{
  if (msg->data.empty() || (msg->data[0] == 0 && msg->data[1] == 0))
    return;
  
  int pitch_temp = pitchAngle_ + msg->data[0];
  if (pitch_temp < pitch_min_ || pitch_temp > pitch_max_)
    return;
  else
    pitchAngle_ = pitch_temp;
  
  int yaw_temp = yawAngle_ - msg->data[1];
  if (yaw_temp < yaw_min_ || yaw_temp > yaw_max_)
    return;
  else
    yawAngle_ = yaw_temp;
  
  int power = 4;
  pubServo(pitchAngle_, yawAngle_, power);
}

void searchCallback(const Int8ConstPtr &msg)
{
  if (modeType_ == m_search) {
    if (msg->data == -1) {
      pubInfo("Search report: Search around didn't find target.");
      foundTarget_ = false;
      ros::param::set(param_vision_feedback_search, -2);
      ros::param::set(param_vision_feedback, 2);
    }
    else if (msg->data == 0) {
      pubInfo("Search report: In searching...");
      ros::param::set(param_vision_feedback_search, -1);
      ros::param::set(param_vision_feedback, 1);
      foundTarget_ = false;
    }
    else {
      pubInfo("Search report: Found target.");
      foundTarget_ = true;
      ros::param::set(param_vision_feedback_search, 1);
      ros::param::set(param_vision_feedback, 1);
    }
  }
}


void trackCallback(const BoolConstPtr &msg)
{
  if (modeType_ == m_track) {
    if (!msg->data) {
      ROS_INFO("Track report: Target lost!");
      foundTarget_ = false;
      ros::param::set(param_vision_feedback_track, -1);
      ros::param::set(param_vision_feedback, 1);
    }
    else {
      ROS_INFO_THROTTLE(7, "Track report: Tracking...");
      foundTarget_ = true;
      ros::param::set(param_vision_feedback_track, 1);
      ros::param::set(param_vision_feedback, 1);
    }
  }
}

void graspCallback(const Int8ConstPtr &msg)
{
  if (modeType_ == m_track) {
    if (!msg->data) {
      ROS_INFO_THROTTLE(3, "Grasp report: Failed.");
      ros::param::set(param_vision_feedback_grasp, -1);
      ros::param::set(param_vision_feedback, 1);
    }
    else if (msg->data == 1) {
      ROS_INFO_THROTTLE(3, "Grasp report: Reaching to the target...");
      ros::param::set(param_vision_feedback_grasp, 1);
      ros::param::set(param_vision_feedback, 1);
    }
    else {
      ROS_INFO("Grasp report: Target in reach.");
      // Reset if grasping has been performed
      resetStatus();
    }
  }
}

void putCallback(const Int8ConstPtr &msg)
{
  if (modeType_ == m_put) {
    if (msg->data < 0) {
      ROS_INFO_THROTTLE(3, "Put report: Failed.");
      drvPubPutResult_.publish(msg);
    }
    else if (msg->data == 1) {
      ROS_INFO_THROTTLE(3, "Put report: Succeeded.");
      drvPubPutResult_.publish(msg);
    }
    else {
      ROS_INFO_THROTTLE(11, "Put report: Putting the target...");
    }
    putSuccess_ = msg->data;
  }
}

void faceRecognizeCallback(const BoolConstPtr &msg)
{
  if (modeType_ == m_wander) {
    if (!msg->data) {
      pubInfo("Face report: Failed.");
      ros::param::set(param_vision_feedback_face, -1);
      ros::param::set(param_vision_feedback, 2);
    }
    else {
      pubInfo("Face report: Succeeded.");
      ros::param::set(param_vision_feedback_face, 1);
      ros::param::set(param_vision_feedback, 3);
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drv_brain");
  
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  // Servo's max angle to rotate
  pnh.getParam("pitch_min", pitch_min_);
  pnh.getParam("pitch_max", pitch_max_);
  pnh.getParam("yaw_min", yaw_min_);
  pnh.getParam("yaw_max", yaw_max_);
  
  servoPub_ = nh.advertise<UInt16MultiArray>("servo", 1);
  // For publishing mode info
  drvPubMode_ = nh.advertise<String>("/comm/msg/vision/mode", 1);
  drvPubInfo_ = nh.advertise<String>("/comm/msg/vision/info", 1);
  
  // Function feedback publishers
  drvPubPutResult_ = nh.advertise<Int8>("/feed/vision/put/result", 1);
  
  // Don't change the order without reason
  ros::Subscriber sub_servo_ctrl = nh.subscribe<Int32MultiArray>("/joy_teleop/servo", 1, teleOpCallback);
  //ros::Subscriber sub_tgt = nh.subscribe<drv_msgs::target_info>("recognize/target", 1, targetCallback);
  ros::Subscriber sub_sh = nh.subscribe<Int8>("status/search/feedback", 1, searchCallback);
  ros::Subscriber sub_tk = nh.subscribe<Bool>("status/track/feedback", 1, trackCallback);
  ros::Subscriber sub_gp = nh.subscribe<Int8>("status/grasp/feedback", 1, graspCallback);
  ros::Subscriber sub_pt = nh.subscribe<Int8>("status/put/feedback", 1, putCallback);
  ros::Subscriber sub_fr = nh.subscribe<Bool>("status/face/feedback", 1, faceRecognizeCallback);
  
  FaceListener fl;
  TargetListener tl;
  TargetSelector ts;
  
  resetStatus();
  
  pubInfo("Deep Robot Vision system initialized!");
  
  while (ros::ok()) {
    // Main on/off control
    if (ros::param::has(param_vision_shared_switch)) {
      bool temp = true;
      ros::param::get(param_vision_shared_switch, temp);
      if (temp)
        ROS_WARN_COND(!centralSwitch_, "Brain: Central switch is ON.");
      centralSwitch_ = temp;
    }
    
    if (!centralSwitch_) {
      ROS_WARN_THROTTLE(9, "Brain: Central switch is OFF.");
      resetStatus();
      continue;
    }
    
    // Initialize servo position
    if (!servo_initialized_) {
      pubServo(90, 90, 1);
      servo_initialized_ = true;
      ROS_INFO("Brain: Servo reset.");
    }
    
    // Get feedback to determine whether target were found
    ros::spinOnce();
    
    // Get target label if the params were set
    tl.getTargetStatus(isTargetSet_, targetLabel_, isPut_);
    ros::param::set(param_target_label, targetLabel_);
    
    if (isTargetSet_ != targetSetTemp_) {
      if (isTargetSet_)
        pubInfo("Brain: Target set to be '" + targetLabel_ + "'.");
      else {
        pubInfo("Brain: Target cancelled.");
        resetStatus();
      }
      targetSetTemp_ = isTargetSet_;
    }
    
    // If user selects target on cellphone, publish the target
    ts.publishOnceIfTargetSelected(isTargetSet_, foundTarget_);
    
    // Mode selection, notice that modeType_ should only be set by central control
    if (isTargetSet_) {
      if (foundTarget_) {
        // No matter whether the target is searched or selected by user,
        // as long as it was found, track it.
        modeType_ = m_track;
      }
      else {
        if (targetLabel_ == "user selected object") {
          // If the target is selected by user but no more been found,
          // which means tracking has lost it, reset.
          pubInfo("Brain: Tracking did not follow.");
          resetStatus();
        }
        else
          modeType_ = m_search;
      }
    }
    else {
      if (isPut_) {
        if (putSuccess_ == 0) {
          modeType_ = m_put;
        }
        else
          modeType_ = m_wander;
      }
      else {
        modeType_ = m_wander;
        fl.isNeedRecognizeFace(); // Only recognize face in wander mode
      }
    }
    
    // Set private running mode
    ros::param::set(param_running_mode, modeType_);
    
    if (modeType_ != modeTypeTemp_) {
      String mode_msg;
      mode_msg.data = modeName[modeType_];
      drvPubMode_.publish(mode_msg);
      ROS_INFO("Brain: Current mode: %s.", modeName[modeType_].c_str());
      modeTypeTemp_ = modeType_;
      if (modeType_ == m_wander) {
        // Reset head pose
        servo_initialized_ = false;
      }
    }
  }
  
  return 0;
}

