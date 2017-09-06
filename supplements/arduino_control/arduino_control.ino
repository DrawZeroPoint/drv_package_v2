#include <ros.h>

#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <ros/time.h>
#include <std_msgs/Float32.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/UInt16MultiArray.h>
#include <Servo.h>

#include <Wire.h>

ros::NodeHandle  nh;

#define DEVICE (0x53)
#define TO_READ (6)

Servo servo1; //pitch
Servo servo2; //yaw
byte buff[TO_READ];
char str[512];
int regAddress = 0x32;
int y, z;

std_msgs::Float32 pitch_msg;
ros::Publisher pub_pitch("camera_pitch", &pitch_msg);

void servo_cb( const std_msgs::UInt16MultiArray&  cmd_msg)
{
  servo1.write(cmd_msg.data[0]); //0-180
  servo2.write(cmd_msg.data[1]); 
}

ros::Subscriber<std_msgs::UInt16MultiArray> sub("servo", servo_cb);

void setup() {
  Wire.begin();
  Serial.begin(57600);
  pinMode(13, OUTPUT);

  writeTo(DEVICE, 0x2D, 0);      
  writeTo(DEVICE, 0x2D, 16);
  writeTo(DEVICE, 0x2D, 8);
  
  nh.initNode();
  nh.advertise(pub_pitch);
  nh.subscribe(sub);
  
  servo1.attach(12);
  servo2.attach(13);
}

void loop() {

  readFrom(DEVICE, regAddress, TO_READ, buff);
  y = (((int)buff[3]) << 8) | buff[2];
  z = (((int)buff[5]) << 8) | buff[4];
  
  float roll = 0.0;
  RP_calculate(roll);

  pitch_msg.data = roll;
  pub_pitch.publish(&pitch_msg);
  
  nh.spinOnce();
  delay(10);
}

void writeTo(int device, byte address, byte val) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(val);
  Wire.endTransmission();
}

void readFrom(int device, byte address, int num, byte buff[]) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.endTransmission();

  Wire.beginTransmission(device);
  Wire.requestFrom(device, num);

  int i = 0;
  while(Wire.available())
  { 
    buff[i] = Wire.read();
    i++;
  }
  Wire.endTransmission();
}

void RP_calculate(float &roll){
  float y_Buff = float(y);
  float z_Buff = float(z);
  roll = atan2(y_Buff , z_Buff) * 57.3;
}
