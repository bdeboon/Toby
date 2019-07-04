
#include <stdlib.h>
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16.h>
#include <std_msgs/ColorRGBA.h>
#include <sstream>

using namespace std_msgs;

sensor_msgs::Joy joy_msg;
std_msgs::ColorRGBA LED;
std_msgs::Int16 left_motor;
std_msgs::Int16 right_motor;


///////////// Arguments ////////////////////////

float max_speed = 2200;
float turn_speed = 1400;

////////////////////////////////////////////////


float u_forward = 1;
float u_direc = 1;
int start = 0;
int start_flag = 0;
int left_trig = 0;
int right_trig = 0;
int red, green, blue;

float ul = 0;
float ur = 0;

void joy_callback(const sensor_msgs::Joy &joyy){
	u_forward = joyy.axes[1];
	u_direc = joyy.axes[0];
	start = joyy.buttons[9];
	//red = joyy.buttons[1];
	//green = joyy.buttons[0];
	//blue = joyy.buttons[2];
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "joy_control_node");
  ros::NodeHandle nh;

  ros::Publisher left_motor_pub = nh.advertise<std_msgs::Int16>("left_motor", 10);
  ros::Publisher right_motor_pub = nh.advertise<std_msgs::Int16>("right_motor", 10);
  ros::Publisher color_pub = nh.advertise<std_msgs::ColorRGBA>("LED", 100);
  ros::Subscriber joy_sub = nh.subscribe("joy", 100, joy_callback);
  ros::Rate loop_rate(100);

  ROS_INFO("JOY NODE");
  ROS_INFO("Press START");
  while(start < 0.5) {
	ros::spinOnce();
	loop_rate.sleep();
  }
  ROS_INFO("************Node Initialized**************");
  start_flag = 1;
	
  while (ros::ok()) {
  // u_forward is amount in forward direction
 //if u direc is 0, right and left motors should be the same

  ul = u_forward*max_speed - (u_direc*turn_speed);
  ur = u_forward*max_speed + (u_direc*turn_speed);
	//u_right = abs(u_right - 1)*2048;
	//ROS_INFO("%f",u_left);

  left_motor.data = (int) ul;
  right_motor.data = (int) ur;
  //LED.r = red;
  //LED.g = green;
  //LED.b = blue;
  //color_pub.publish(LED);

  left_motor_pub.publish(left_motor);
  right_motor_pub.publish(right_motor);


  ros::spinOnce();

  loop_rate.sleep();

  }


  return 0;
}
