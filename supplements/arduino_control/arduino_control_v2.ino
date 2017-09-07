/* Author: Dong Zhipeng 2017/9/1
 *  This code is for NVG's Arduino Mega 2560 to connect and communicate with ROS 
 *  as well as report angle data, control the servos and display status with leds.
 *  
 *  Connection map
 *  Componts         Mega 2560
 *  led_front   ->   digital 6
 *  led_back    ->   digital 7
 *  JY61(TX)    ->   digital 19
 *  servo_pitch ->   digital 32
 *  servo_yaw   ->   digital 33
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
#include <std_msgs/Float32MultiArray.h>

ros::NodeHandle  nh;

/* Servo control */
#include <VarSpeedServo.h>
const int servo_speed = 70;
VarSpeedServo p_sv;
VarSpeedServo y_sv;

void servo_cb(const std_msgs::Float32MultiArray& msg) {
  if (sizeof(msg.data)/sizeof(*(msg.data)) == 2) {
    p_sv.write(msg.data[0], servo_speed, false);
    y_sv.write(msg.data[1], servo_speed, true);
  }
  else if (sizeof(msg.data)/sizeof(*(msg.data)) == 3) {
    if (msg.data[2] >= 1 && msg.data[2] <=255) {
      p_sv.write(msg.data[0], msg.data[2], false);
      y_sv.write(msg.data[1], msg.data[2], true);
    }
    else {
      p_sv.write(msg.data[0], servo_speed, false);
      y_sv.write(msg.data[1], servo_speed, true);
    }
  }
  else if (sizeof(msg.data)/sizeof(*(msg.data)) == 4) {
    p_sv.write(msg.data[0], msg.data[2], false);
    y_sv.write(msg.data[1], msg.data[3], true);
  }
}

ros::Subscriber<std_msgs::Float32MultiArray> sub_servo("motor", servo_cb);

/* IMU */
#include <JY901.h>
float x_ang = 0.0; // roll angle
float y_ang = 0.0; // ptich angle
float z_ang = 0.0; // yaw angle
std_msgs::Float32 pitch_msg;
std_msgs::Float32 yaw_msg;
ros::Publisher pub_pitch("camera_pitch", &pitch_msg);
ros::Publisher pub_yaw("camera_yaw", &yaw_msg);

/* LED control */
#include <FastLED.h>
#define NUM_LEDS 2
CRGB leds[NUM_LEDS][1]; // use 2 leds, each has 1 light
int fade_val = 5;
int fade_step = 5;

int error_flag = 0;

void error_cb(const std_msgs::UInt16& msg) {
  error_flag = msg.data;
}

ros::Subscriber<std_msgs::UInt16> sub_error("error", error_cb);

void setup() {
  Serial.begin(57600); 
  Serial1.begin(115200); // 18 TX, 19 RX

  FastLED.addLeds<NEOPIXEL, 6>(leds[0], 1); // front
  FastLED.addLeds<NEOPIXEL, 7>(leds[1], 1); // back
  LEDS.setBrightness(255); // 0-255

  nh.getHardware()->setBaud(57600);
  nh.advertise(pub_pitch);
  nh.advertise(pub_yaw);
  nh.subscribe(sub_error);
  nh.subscribe(sub_servo);
  
  p_sv.attach(32, 500, 2500);
  y_sv.attach(33, 500, 2500);
  y_sv.write(90, servo_speed, false); // set the intial position of the servo, run in background
  p_sv.write(60, servo_speed, true);
}

void loop() {
  while (Serial1.available()) 
  {
    JY901.CopeSerialData(Serial1.read()); //Call JY901 data cope function
  }
  x_ang = (float)JY901.stcAngle.Angle[0]/32768*180;
  y_ang = (float)JY901.stcAngle.Angle[1]/32768*180;
  z_ang = (float)JY901.stcAngle.Angle[2]/32768*180;
  
  pitch_msg.data = x_ang; // We make x axis as yaw
  yaw_msg.data = z_ang;
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
    delay(500);
    for(int i = 0; i < NUM_LEDS; i++) {
      leds[i][0] = CRGB::Black;
      FastLED.show();
    }
    delay(490);
  }
  // Illumination
  else if (error_flag == 10) {
    leds[0][0] = CRGB::White;
    FastLED.show();
  }
  // Night mode
  else if (error_flag == 11) {
    for(int i = 0; i < NUM_LEDS; i++) {
      leds[i][0] = CRGB::Black;
      FastLED.show();
    }
  }
  // Sleep mode
  else if (error_flag == 12) {
    for(int i = 0; i < NUM_LEDS; i++) {
      leds[i][0] = CRGB::Amethyst;
      leds[i][0].fadeLightBy(fade_val); // 0:no fade, 255 full fade
      FastLED.show();
    }
    delay(30);
    fade_val += fade_step;
    if (fade_val >= 255 || fade_val <= 0) {
      fade_step = -fade_step;
    }
  }
  // normal
  else {
    for(int i = 0; i < NUM_LEDS; i++) {
      leds[i][0] = CRGB::Green;
      FastLED.show();
    }
  }
}


