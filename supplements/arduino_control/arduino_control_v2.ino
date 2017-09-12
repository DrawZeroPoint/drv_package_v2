/* Author: Dong Zhipeng 2017/9/1
 *  This code is for NVG's Arduino Mega 2560 to connect and communicate with ROS 
 *  as well as report angle data, control the servos and display status with leds.
 *  
 *  Connection map
 *  Componts         Mega 2560
 *  led_front   ->   digital 13
 *  led_back    ->   digital 11
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

/* Suspend control */
bool active = false;
unsigned long previousActive = 0;
const long interval = 60000; 

/* Servo control */
#include <VarSpeedServo.h>
const int servo_speed = 50; // 0-255
VarSpeedServo p_sv;
VarSpeedServo y_sv;

void servo_cb(const std_msgs::Float32MultiArray& msg) {
  if (msg.data_length == 2) {
    p_sv.write(msg.data[0], servo_speed, false);
    y_sv.write(msg.data[1], servo_speed, true);
  }
  else if (msg.data_length == 3) {
    if (msg.data[2] > 0 && msg.data[2] < 255) {
      p_sv.write(msg.data[0], msg.data[2], false);
      y_sv.write(msg.data[1], msg.data[2], true);
    }
    else if (msg.data[2] >= 255) {
      p_sv.write(msg.data[0], 255, false);
      y_sv.write(msg.data[1], 255, true);
    }
  }
  else if (msg.data_length == 4) {
    if (msg.data[2] > 0 && msg.data[2] < 255) {
      p_sv.write(msg.data[0], msg.data[2], false);
    }
    else if (msg.data[2] >= 255) {
      p_sv.write(msg.data[0], 255, false);
    }
    if (msg.data[3] > 0 && msg.data[3] < 255) {
      y_sv.write(msg.data[1], msg.data[3], true);
    }
    else if (msg.data[3] >= 255) {
      y_sv.write(msg.data[1], 255, true);
    }
  }
  active = true;
}

ros::Subscriber<std_msgs::Float32MultiArray> sub_servo("motor", servo_cb);

/* IMU */
#include <JY901.h>
float x_ang = 0.0; // roll angle
float y_ang = 0.0; // ptich angle
float z_ang = 0.0; // yaw angle
float offset_yaw = 0.0; // When y_sv = 90, record this value, suppose servo is accurate

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
float acc_th = 1.04; // g

int state_flag = 0;

void error_cb(const std_msgs::UInt16& msg) {
  state_flag = msg.data;
  active = true;
}

ros::Subscriber<std_msgs::UInt16> sub_error("error", error_cb);

void setup() {
  Serial.begin(57600); 
  Serial1.begin(115200); // 18 TX, 19 RX

  FastLED.addLeds<NEOPIXEL, 13>(leds[0], 1); // front
  FastLED.addLeds<NEOPIXEL, 11>(leds[1], 1); // back
  LEDS.setBrightness(255); // 0-255

  nh.getHardware()->setBaud(57600);
  nh.advertise(pub_pitch);
  nh.advertise(pub_yaw);
  nh.subscribe(sub_error);
  nh.subscribe(sub_servo);
  
  p_sv.attach(32, 500, 2500);
  y_sv.attach(33, 500, 2500);
  p_sv.write(70, servo_speed, true); // set the intial position of the servo
  y_sv.write(90, servo_speed, true); 

  while (Serial1.available()) 
  {
    JY901.CopeSerialData(Serial1.read()); //Call JY901 data cope function
  }
  z_ang = (float)JY901.stcAngle.Angle[2]/32768*180;
  offset_yaw = -z_ang;
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
  yaw_msg.data = 90.0 - z_ang - offset_yaw;
  pub_pitch.publish(&pitch_msg);
  pub_yaw.publish(&yaw_msg);

  float x_acc = (float)JY901.stcAcc.a[0]/32768*16;
  float y_acc = (float)JY901.stcAcc.a[1]/32768*16;
  float z_acc = (float)JY901.stcAcc.a[2]/32768*16;
  checkVibration(x_acc, y_acc, z_acc);

  nh.spinOnce();
  delay(10);

  checkSleep();

  checkError(); // if error occur, show it using led
}

void checkVibration(float x_a, float y_a, float z_a) {
  if (fabs(x_a) > acc_th || fabs(y_a) > acc_th || fabs(z_a) > acc_th) {
    state_flag = 20;
    active = true;
  }
}

void checkSleep() {
  // Check whether the hardware shall suspend
  unsigned long currentMillis = millis();
  if (!active) {
    if (currentMillis - previousActive >= interval) {
      state_flag = 12;
    }
  }
  else {
    if (state_flag == 12)
      state_flag = 0;
    previousActive = currentMillis;
    active = false;
  }
}

void checkError() {
  // warning
  if (state_flag == 1) {
    for(int i = 0; i < NUM_LEDS; i++) {
      leds[i][0] = CRGB::Yellow;
      FastLED.show();
    }
  }
  // error
  else if (state_flag == 2) {
    for(int i = 0; i < NUM_LEDS; i++) {
      leds[i][0] = CRGB::Red;
      FastLED.show();
    }
  }
  // fatal error
  else if (state_flag == 3) {
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
  else if (state_flag == 10) {
    leds[0][0] = CRGB::White;
    FastLED.show();
  }
  // Night mode
  else if (state_flag == 11) {
    for(int i = 0; i < NUM_LEDS; i++) {
      leds[i][0] = CRGB::Black;
      FastLED.show();
    }
  }
  // Sleep mode
  else if (state_flag == 12) {
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
  // Vibration
  else if (state_flag == 20) {
    for(int i = 0; i < NUM_LEDS; i++) {
      leds[i][0] = CRGB::Orange;
      FastLED.show();
    }
    delay(30);
    state_flag = 0;
  }
  // Normal
  else {
    for(int i = 0; i < NUM_LEDS; i++) {
      leds[i][0] = CRGB::Green;
      FastLED.show();
    }
  }
}

