/*
 * This code subscribe motor control messages, process it and send them out,
 * so that can be recived by Arduino and executed by motors of NVG 2.0
*/
#include <signal.h>
#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#include <ros/console.h>

#include <math.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>


#include <stdio.h>

using namespace std;

// Publish motor control params
ros::Publisher motorPub_;

// The rosparam that store pitch and yaw values
string param_servo_pitch = "/status/servo/pitch";
string param_servo_yaw = "/status/servo/yaw";

int pitchAngle_ = 90;
int yawAngle_ = 90;

const float pi = 3.14159265359;

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

float to_rad(float angle_deg)
{
  return angle_deg / 180 * pi;
}

float to_deg(float angle_rad)
{
  return angle_rad * 180 / pi;
}

float calculate(float ab, float bc, float cd, float ad, float theta_a)
{
  float bd = sqrt(pow(ab, 2) + pow(ad, 2) - 2*ab*ad*cos(theta_a));
  float gamma_b = acos((pow(bc, 2) + pow(bd, 2) - pow(cd, 2)) / (2 * bc * bd));
  float phi_d = asin(ab / bd * sin(theta_a));
  float gamma_d = asin(bc / cd * sin(gamma_b));
  float result = phi_d + gamma_d;
  return to_deg(result);
}

void setBoundary(int &pitch_angle, int &yaw_angle)
{
  // Set the boundry of pitch value for NVG 2.0
  if (pitch_angle > 140) pitch_angle = 140;
  else if (pitch_angle < 60) pitch_angle = 60;
  if (yaw_angle > 180) yaw_angle = 180;
  else if (yaw_angle < 0) yaw_angle = 0;
}

void motorPublish(int pitch_angle, int yaw_angle,
                  int pitch_speed, int yaw_speed)
{
  std_msgs::UInt16MultiArray array;
  array.data.push_back(pitch_angle);
  array.data.push_back(yaw_angle);
  array.data.push_back(pitch_speed);
  array.data.push_back(yaw_speed);
  motorPub_.publish(array);
  
  ros::param::set(param_servo_pitch, pitch_angle);
  ros::param::set(param_servo_yaw, yaw_angle);
}

void servoCallback(const std_msgs::UInt16MultiArrayConstPtr &msg)
{
  // This callback should always active
  int pitchSpeed = 50;
  int yawSpeed = 50;
  
  if (msg->data.size() == 2) {
    pitchAngle_ = msg->data[0];
    yawAngle_ = msg->data[1];
  }
  else if (msg->data.size() == 3) {
    pitchAngle_ = msg->data[0];
    yawAngle_ = msg->data[1];
    pitchSpeed = msg->data[2];
    yawSpeed = msg->data[2];
  }
  else if (msg->data.size() == 4) {
    pitchAngle_ = msg->data[0];
    yawAngle_ = msg->data[1];
    pitchSpeed = msg->data[2];
    yawSpeed = msg->data[3];
  }
  else {
    ROS_WARN_THROTTLE(3, "Servo params invalid.");
  }

  setBoundary(pitchAngle_, yawAngle_);
  motorPublish(pitchAngle_, yawAngle_, pitchSpeed, yawSpeed);
}

/**
 * @brief nextCallback
 * When the msg was received, the servo will try to move to next
 * predefined position based on current state
 * @param msg
 */
void nextCallback(const std_msgs::UInt16MultiArrayConstPtr &msg)
{
  // TODO: change to service
}

// Replacement SIGINT handler
void mySigIntHandler(int sig)
{
  g_request_shutdown = 1;
}

// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
    num_params = params.size();
  if (num_params > 1) {
    string reason = params[1];
    ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
    g_request_shutdown = 1; // Set flag
  }

  result = ros::xmlrpc::responseInt(1, "", 0);
}

int main(int argc, char **argv)
{
  // Override SIGINT handler
  ros::init(argc, argv, "drv_motor", ros::init_options::NoSigintHandler);
  signal(SIGINT, mySigIntHandler);

  // Override XMLRPC shutdown
  ros::XMLRPCManager::instance()->unbind("shutdown");
  ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

  ros::NodeHandle nh;

  motorPub_ = nh.advertise<std_msgs::UInt16MultiArray>("motor", 5, true);

  // This node should be launched in namespace /vision
  ros::Subscriber sub_servo_cmd = nh.subscribe<std_msgs::UInt16MultiArray>("servo", 1, servoCallback);
  ros::Subscriber sub_to_pos = nh.subscribe<std_msgs::UInt16MultiArray>("servo/next_pos", 1, nextCallback);

  while (!g_request_shutdown)
  {
    ros::spinOnce();
  }

  // Reset head pose before each shut down
  motorPublish(90, 90, 40, 30);
  ros::Duration(1).sleep();

  ros::shutdown();
  return 0;
}

