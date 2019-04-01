// Motor Speed Controller Test
// Ensure DEBUG_MOTOR Flag is high in Arduino Sensor Script

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros/time.h"

#include <sensor_msgs/Imu.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <sstream>

using namespace geometry_msgs;

int motor[2][1] = {0,0};

sensor_msgs::Imu imu_measured;
std_msgs::Int16 left_motor;
std_msgs::Int16 right_motor;
geometry_msgs::Twist velocities;

float reference = 0.3192; //m/s (1 rotation every 2 seconds)
float error_left;
float old_error_left = 0;
float total_error_left = 0;

float error_right;
float old_error_right = 0;
float total_error_right = 0;

float u_left;
float u_right;

int Kp_l = 3200, Ki_l = 0, Kd_l = 0;
int Kp_r = 3200, Ki_r = 0, Kd_r = 0;

void imu_callback(const sensor_msgs::Imu &imu)
{
	imu_measured = imu;
}
void velocities_callback(const geometry_msgs::Twist &twist)
{
	velocities = twist;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "speed_control_node");

  ros::NodeHandle nh;

  ros::Publisher left_motor_pub = nh.advertise<std_msgs::Int16>("left_motor", 1000);
  ros::Publisher right_motor_pub = nh.advertise<std_msgs::Int16>("right_motor", 1000);

  ros::Subscriber imu_sub = nh.subscribe("imu", 1000, imu_callback);
  ros::Subscriber velocities_sub = nh.subscribe("toby_velocity", 1000, velocities_callback);

  ros::Rate loop_rate(1000);

  ROS_INFO("Node Initialized, waiting for step input");
  sleep(2);
  ROS_INFO("6");
  sleep(1);
  ROS_INFO("5");
  sleep(1);
  ROS_INFO("4");
  sleep(1);
  ROS_INFO("3");
  sleep(1);
  ROS_INFO("2");
  sleep(1);
  ROS_INFO("1");
  sleep(1);

  while (ros::ok())
  {
  error_left = velocities.linear.y - reference;
  error_right = velocities.linear.z - reference;
  total_error_left = total_error_left + error_left;
  total_error_right = total_error_right + error_right;
  error_right = velocities.linear.z - reference;

  u_left = Kp_l*(error_left)+Ki_l*(total_error_left)+Kd_l*(error_left - old_error_left);
  u_right = Kp_r*(error_right)+Ki_r*(total_error_right)+Kd_r*(error_right - old_error_right);
  old_error_left = error_left;
  old_error_right = error_right;

  left_motor.data = (int) u_left;
  right_motor.data = (int) u_right;

	left_motor_pub.publish(left_motor);
  right_motor_pub.publish(right_motor);

  ros::spinOnce();

  loop_rate.sleep();
  }


  return 0;
}
