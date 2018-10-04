// LQR Controller

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros/time.h"


#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <sstream>

using namespace sensor_msgs;
using namespace message_filters;
using namespace nav_msgs;
using namespace geometry_msgs;

sensor_msgs::Imu imu_measured;
std_msgs::Float64 x_vel;
geometry_msgs::Vector3 input_voltage;

void imu_callback(const sensor_msgs::Imu &imu) 
{
	imu_measured = imu;
}
void x_vel_callback(const std_msgs::Float64 &x_velocity) 
{
	x_vel = x_velocity;
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "control_node");

  ros::NodeHandle nh;

  ros::Publisher motor_pub = nh.advertise<geometry_msgs::Vector3>("motor_voltages", 1000);
  ros::Subscriber imu_sub = nh.subscribe("imu", 1000, imu_callback);
  ros::Subscriber x_vel_sub = nh.subscribe("linear_velocity", 1000, x_vel_callback);

  ros::Rate loop_rate(100);
  float K_LQR[2][5] = {
-11.4214, -5.9419, 170.0483, 7.0711, 123.6416,
-11.4214, -5.9419, 170.0483, -7.0711, -123.6416
};
  float Current_States[5][1] = {0,0,0,0,0};
  float voltages[2][1] = {0,0};
  int count = 0;
  int i;
  int j;

  while (ros::ok())
  {
       
	Current_States[1][1] = imu_measured.orientation.x;
	Current_States[2][1] = imu_measured.angular_velocity.x;
        Current_States[3][1] = x_vel.data;
	Current_States[4][1] = imu_measured.orientation.z;
	Current_States[5][1] = imu_measured.angular_velocity.z;

	//Calculate values for input voltage using K_LQR

	voltages[1][1] = (K_LQR[1][1]*Current_States[1][1]) + (K_LQR[1][2]*Current_States[2][1]) + (K_LQR[1][3]*Current_States[3][1]) + (K_LQR[1][4]*Current_States[4][1]) + (K_LQR[1][5]*Current_States[5][1]);
	
	voltages[2][1] = (K_LQR[2][1]*Current_States[1][1]) + (K_LQR[2][2]*Current_States[2][1]) + (K_LQR[2][3]*Current_States[3][1]) + (K_LQR[2][4]*Current_States[4][1]) + (K_LQR[2][5]*Current_States[5][1]);
         

	input_voltage.x = voltages[1][1];
	input_voltage.y = voltages[2][1];

	motor_pub.publish(input_voltage);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
