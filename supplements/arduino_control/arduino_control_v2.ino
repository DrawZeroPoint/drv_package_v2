/* Author: Dong Zhipeng 2017/9/1
 *  This code is for NVG's Arduino Mega 2560 to connect and communicate with ROS 
 *  as well as report angle data, control the servos and display status with leds.
 *  
 *  Connection map
 *  Componts         Mega 2560
 *  led_front   ->   digital 6
 *  led_back    ->   digital 7
 *  JY61(TX)    ->   digital 10
 *  servo_pitch ->   digital 12
 *  servo_yaw   ->   digital 13
 */
 
#include <ros.h>

#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <ros/time.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/UInt16MultiArray.h>

ros::NodeHandle  nh;

#include <Servo.h>

#include <Wire.h>
#include <SoftwareSerial.h>
#include <JY901.h>
SoftwareSerial mySerial(10, 11); // RX, TX
// JY901   Arduino
//  TX <---> 10
float x_a = 0.0;
float y_a = 0.0;
float z_a = 0.0;

#include "FastLED.h"
#define NUM_LEDS 2
CRGB leds[NUM_LEDS][1]; // use 2 leds, each has 1 light

int error_flag = 0;

Servo p_sv; // for camera up and down
Servo y_sv; // for head turn left and right

std_msgs::Float32 pitch_msg;
std_msgs::Float32 yaw_msg;
ros::Publisher pub_pitch("camera_pitch", &pitch_msg);
ros::Publisher pub_yaw("camera_yaw", &yaw_msg);

void error_cb(const std_msgs::UInt16& msg) {
  error_flag = msg.data;
}

void servo_cb(const std_msgs::UInt16MultiArray& msg) {
  p_sv.write(msg.data[0]); // 0-180
  y_sv.write(msg.data[1]); // 0-180
}

ros::Subscriber<std_msgs::UInt16> sub_error("error", error_cb);
ros::Subscriber<std_msgs::UInt16MultiArray> sub_servo("servo", servo_cb);

void setup() {
  Wire.begin();
  Serial.begin(57600); // JY61 only support this baud
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  mySerial.begin(115200);

  FastLED.addLeds<NEOPIXEL, 6>(leds[0], 1); // front
  FastLED.addLeds<NEOPIXEL, 7>(leds[1], 1); // back

  nh.getHardware()->setBaud(57600); // refer https://answers.ros.org/question/206972/baud-rate-parameter-in-rosserial_python-arduino/
  nh.initNode();
  nh.advertise(pub_pitch);
  nh.subscribe(sub_error);
  nh.subscribe(sub_servo);
  
  p_sv.attach(12);
  y_sv.attach(13);
}

void loop() {
  while (mySerial.available()) 
  {
    JY901.CopeSerialData(mySerial.read()); //Call JY901 data cope function
  }
  x_a = (float)JY901.stcAngle.Angle[0]/32768*180;
  y_a = (float)JY901.stcAngle.Angle[1]/32768*180;
  z_a = (float)JY901.stcAngle.Angle[2]/32768*180;
  
  pitch_msg.data = x_a;
  yaw_msg.data = y_a;
  pub_pitch.publish(&pitch_msg);
  pub_yaw.publish(&yaw_msg);
  
  nh.spinOnce();
  delay(10);

  checkError(); // if error occur, show it using led
}

void checkError() {
  // warning
  if (error_flag == 1) {
    for(int i = 0; i < NUM_LEDS; i++) {
      leds[i][0] = CRGB::Yellow;
      FastLED.show();
    }
  }
  // error
  else if (error_flag == 2) {
    for(int i = 0; i < NUM_LEDS; i++) {
      leds[i][0] = CRGB::Red;
      FastLED.show();
    }
  }
  // fatal error
  else if (error_flag == 3) {
    for(int i = 0; i < NUM_LEDS; i++) {
      leds[i][0] = CRGB::Red;
      FastLED.show();
    }
    delay(300);
    for(int i = 0; i < NUM_LEDS; i++) {
      leds[i][0] = CRGB::Black;
      FastLED.show();
    }
    delay(290);
  }
  // normal
  else {
    for(int i = 0; i < NUM_LEDS; i++) {
      leds[i][0] = CRGB::Green;
      FastLED.show();
    }
  }
}


