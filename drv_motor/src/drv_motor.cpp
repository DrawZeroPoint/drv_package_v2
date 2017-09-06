/*
 * This code subscribe motor control messages, process it and send them out,
 * so that can be recived by Arduino and executed by motors of NVG 2.0
*/

#include <ros/ros.h>
#include <ros/console.h>

#include <math.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>


#include <stdio.h>

using namespace std;

// publish motor control params
ros::Publisher motorPub_;

const float pi = 3.14159265359;

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

void calculatePitch(float &pitch_angle)
{
  // Set the boundry of pitch value for NVG 2.0
  if (pitch_angle > 110) pitch_angle = 110.0;
  if (pitch_angle < 55) pitch_angle = 55.0;
  float target_angle = pitch_angle;

  // structure params
  float ab = 41.23;
  float bc = 68;
  float cd = 26;
  float ad = 84.49;
  float phi_d = 141.13;
  float mu_d = 2.58;

  float theta_a = to_rad(phi_d - target_angle - mu_d);
  // output in degree
  float result = calculate(ab, bc, cd, ad, theta_a);
  pitch_angle = 180 - result + mu_d;
}

void motorPublish(float pitch_angle, float yaw_angle, float pitch_speed, float yaw_speed)
{
  std_msgs::Float32MultiArray array;
  array.data.push_back(pitch_angle);
  array.data.push_back(yaw_angle);
  array.data.push_back(pitch_speed);
  array.data.push_back(yaw_speed);
  motorPub_.publish(array);
}

void servoCallback(const std_msgs::UInt16MultiArrayConstPtr &msg)
{
  // this callback should always active
  float pitchAngle = 90.0;
  float yawAngle = 90.0;
  float pitchSpeed = 70.0;
  float yawSpeed = 70.0;
  if (msg->data.size() == 2) {
    pitchAngle = msg->data[0];
    yawAngle = msg->data[1];
  }
  else if (msg->data.size() == 3) {
    pitchAngle = msg->data[0];
    yawAngle = msg->data[1];
    pitchSpeed = msg->data[2];
    yawSpeed = msg->data[2];
  }
  else if (msg->data.size() == 4) {
    pitchAngle = msg->data[0];
    yawAngle = msg->data[1];
    pitchSpeed = msg->data[2];
    yawSpeed = msg->data[3];
  }
  else {
    ROS_WARN_THROTTLE(11, "Servo message has wrong number of params.");
  }

  calculatePitch(pitchAngle);
  motorPublish(pitchAngle, yawAngle, pitchSpeed, yawSpeed);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "drv_motor");

  ros::NodeHandle nh;

  motorPub_ = nh.advertise<std_msgs::Float32MultiArray>("motor", 1, true);

  // This node should be launched in namespace /vision
  ros::Subscriber sub_servo_cmd = nh.subscribe<std_msgs::UInt16MultiArray>("servo", 2, servoCallback);

  while (ros::ok())
  {
    ros::spinOnce();
  }

  return 0;
}

