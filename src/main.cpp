#include <Arduino.h>
#include <math.h>
//#include <String.h>
#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <HardwareSerial.h>
#include <ODriveArduino.h>
#include "diff_drive.hpp"
#include "LEDtape.hpp"


//hardware parameter
const int motor0 = 0;
const int motor1 = 1;
const unsigned long calibration_interval = 2000;
const double wheel_tread = 0.575;
const double wheel_radius = 0.1;
// Set the number of LEDs to control.
const uint16_t ledCount = 60;
// Set the brightness to use (the maximum is 31).
const uint8_t brightness = 20;


//pin number

////////////////////////////////
// Set up serial pins to the ODrive
////////////////////////////////
// pin 17: RX - connect to ODrive TX GPIO 1
// pin 16: TX - connect to ODrive RX GPIO 2
const uint8_t stop_button_pin = 11;
const uint8_t dataPin = 8;
const uint8_t clockPin = 7;

HardwareSerial& odrive_serial = Serial1;

//variable
unsigned long pre_time;


geometry_msgs::Twist cmd_vel;
void vel_callback(const geometry_msgs::Twist& vel_message)
{
    cmd_vel = vel_message;
}

String mode;
void mode_callback(const std_msgs::String& mode_message)
{
    mode = mode_message.data;
}

//ros
ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> cmd_sub("plat/cmd_vel", &vel_callback);
ros::Subscriber<std_msgs::String> mode_sub("plat/mode_in", &mode_callback);
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("plat/imu", &imu_msg);

// ODrive object
ODriveArduino odrive(odrive_serial);

LEDtape led(dataPin, clockPin, ledCount, brightness);

void odrive_calibration()
{

  int requested_state0;
  int requested_state1;

  requested_state0 = AXIS_STATE_CLOSED_LOOP_CONTROL;
  if(!odrive.run_state(motor0, requested_state0, false )) return;

  requested_state1 = AXIS_STATE_CLOSED_LOOP_CONTROL;
  if(!odrive.run_state(motor1, requested_state1, false )) return;
}

void setup()
{
    // ODrive uses 115200 baud
    odrive_serial.begin(115200);
    // Serial to PC
    //Serial.begin(115200);
    //while (!Serial) ; // wait for Arduino Serial Monitor to open

    //pin mode
    pinMode(stop_button_pin, INPUT_PULLUP);

    //ros
    nh.initNode();
    nh.subscribe(cmd_sub);
    nh.subscribe(mode_sub);
    nh.advertise(imu_pub);

    odrive_calibration();

    //variable initialize
    mode = "stop";
    pre_time = millis();
}

void loop()
{
    //emergency stop
    if(digitalRead(stop_button_pin) == 1){
        mode = "safety_stop";
    }
    //mode
    if(mode == "run"){
        led.lit(LEDtape::Color::GREEN, 500);
    }else if(mode == "stop"){
        cmd_vel.linear.x = 0.0f;
        cmd_vel.angular.z = 0.0f;
        led.lit(LEDtape::Color::BLUE, 200);
    }else if(mode == "safety_stop"){
        cmd_vel.linear.x = 0.0f;
        cmd_vel.angular.z = 0.0f;
        led.lit(LEDtape::Color::RED, 50);
    }

    diffDrive diff_drive(wheel_tread, wheel_radius);
    diff_drive.steering(cmd_vel.linear.x, cmd_vel.angular.z);

    //[m/s] -> [turn/s]
    double v_right = diff_drive.rightVelocity()/(2*M_PI*wheel_radius);
    double v_left = -diff_drive.leftVelocity()/(2*M_PI*wheel_radius);

    if(millis() - pre_time > calibration_interval){
        pre_time = millis();
        odrive_calibration();
    }
    odrive.SetVelocity(motor0, v_left);
    odrive.SetVelocity(motor1, v_right);

    imu_pub.publish(&imu_msg);
    
    nh.spinOnce();
    delay(1);
}