// Motor Speed Controller Test
// Ensure DEBUG_MOTOR Flag is high in Arduino Sensor Script
#include <stdlib.h>

#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16.h>
#include <std_msgs/ColorRGBA.h>
#include <sstream>
//#include <signal.h>

using namespace std_msgs;

sensor_msgs::Joy joy_msg;
std_msgs::ColorRGBA LED;
std_msgs::Int16 left_motor;
std_msgs::Int16 right_motor;

float u_left = 1;
float u_right = 1;
int start = 0;
int start_flag = 0;
int left_trig = 0;
int right_trig = 0;
int red, green, blue;

float ul = 0;
float ur = 0;

void joy_callback(const sensor_msgs::Joy &joyy)
{
	u_left = joyy.axes[2];
	u_right = joyy.axes[5];
	start = joyy.buttons[7];
	red = joyy.buttons[1];
	green = joyy.buttons[0];
	blue = joyy.buttons[2];
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "joy_control_node");
	//ros::init(argc, argv, "joy_control_node", ros::init_options::NoSigintHandler);

  ros::NodeHandle nh;


  ros::Publisher left_motor_pub = nh.advertise<std_msgs::Int16>("left_motor", 100);
  ros::Publisher right_motor_pub = nh.advertise<std_msgs::Int16>("right_motor", 100);
	ros::Publisher color_pub = nh.advertise<std_msgs::ColorRGBA>("LED", 100);

	ros::Subscriber joy_sub = nh.subscribe("joy", 100, joy_callback);

  ros::Rate loop_rate(100);

	ROS_INFO("JOY NODE");
	ROS_INFO("Pull the left trigger");
	while(u_left > -0.95) {
		ros::spinOnce();
		loop_rate.sleep();}

	ROS_INFO("Pull the right trigger");
	while(u_right > -0.95) {
		ros::spinOnce();
		loop_rate.sleep();}

	ROS_INFO("Press START");
	while(start < 0.5) {
		ros::spinOnce();
		loop_rate.sleep();}

	ROS_INFO("************Node Initialized**************");
	start_flag = 1;
	
  while (ros::ok()) {

	ul = (1 - u_left)*2047;
	ur = (1 - u_right)*2047;
	//u_right = abs(u_right - 1)*2048;
	//ROS_INFO("%f",u_left);

  left_motor.data = (int) ul;
  right_motor.data = (int) ur;
	LED.r = red;
	LED.g = green;
	LED.b = blue;
	color_pub.publish(LED);
	left_motor_pub.publish(left_motor);
  right_motor_pub.publish(right_motor);


  ros::spinOnce();

  loop_rate.sleep();

  }


  return 0;
}
