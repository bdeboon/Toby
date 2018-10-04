
//Includes

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include <ros.h>


#include <ros/time.h>
#include <geometry_msgs/Vector3.h>


//Arduino Motor Control Pins
#define left_motor 4
#define right_motor 5

////////////// Input Voltage initialization///////////////////
float left_motor_voltage;
float right_motor_voltage;
float in_left = 0;
float in_right = 0;


//ROS Node Setup//////////////////////////////////////
ros::NodeHandle  nh;

// Input Voltage Callback/////////////////////////////
void voltageCb(const geometry_msgs::Vector3 &voltage_msg){
//  left_motor_voltage = voltage_msg.x;
//  right_motor_voltage = voltage_msg.y;
}


//ROS msg setup////////////////////////////////////////

ros::Subscriber<geometry_msgs::Vector3> input_sub("motor_voltages", &voltageCb );

/////////////////////////////////////////////////////////

void setup() {
  Serial.begin(57600); // initialize serial communication
 
  pinMode(left_motor, OUTPUT);
  pinMode(right_motor, OUTPUT);

  analogWrite(left_motor, 135); //Turn motors 'off'
  analogWrite(right_motor, 135);

  nh.initNode();
  nh.subscribe(input_sub);

}

void loop() {
  nh.spinOnce();
  delay(1);
  
  //in_left = round(135 + 120*(left_motor_voltage/22));
  //in_right = round(135 + 120*(right_motor_voltage/22));
  if(in_left > 255){
    in_left = 255;
  }
  if(in_left < 0) {
    in_left = 0;
  }
  if(in_right > 255){
    in_right = 255;
  }
  if(in_right < 0) {
    in_right = 0;
  }
  //analogWrite(left_motor, in_left); //Turn motors 'off'
  //analogWrite(right_motor, in_right);
}
