
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros/time.h"


#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <sstream>

using namespace sensor_msgs;
using namespace message_filters;

float voltage;

float tilt = 0;
float Kp = 8000;
float Ki = 0;
float Kd = 1500;
float rate = 100;
float calibrate = 0;
float ref = 0;

float old_error = 0;
float total_error = 0;

sensor_msgs::Joy joyy;
geometry_msgs::Vector3 input_voltage;
geometry_msgs::Vector3 kalman_states;
std_msgs::Int16 left_m;
std_msgs::Int16 right_m;

void tilt_callback(const std_msgs::Float64 &tilts)
{
	tilt = tilts.data;
}
void joy_callback(const sensor_msgs::Joy &joyy)
{
	calibrate = joyy.axes[0];
	ref += calibrate*0.001;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "PID_control_node");

  ros::NodeHandle nh;

  ros::Publisher left_motor_pub = nh.advertise<std_msgs::Int16>("left_motor", 1);
  ros::Publisher right_motor_pub = nh.advertise<std_msgs::Int16>("right_motor", 1);
  //ros::Subscriber kalman_sub = nh.subscribe("filtered_states", 1, kalman_callback);
	ros::Subscriber kalman_sub = nh.subscribe("tilt", 1, tilt_callback);
	ros::Subscriber joy_sub = nh.subscribe("joy", 100, joy_callback);

  ros::Rate loop_rate(200);

  while (ros::ok())
  {

		total_error += (ref - tilt);
		voltage = Kp*(ref - tilt) + Ki*(total_error) + Kd*(((ref - tilt) - old_error)/rate);
		old_error = ref - tilt;

  //Save voltages as rosmsg
  left_m.data = (int) -1*voltage; //calm down desired voltage a little (driver expects 4095 as max)
  right_m.data = (int) -1*voltage;

  //Publish motor voltage data
	left_motor_pub.publish(left_m);
  right_motor_pub.publish(right_m);

  ros::spinOnce();
  loop_rate.sleep();

  }


  return 0;
}
