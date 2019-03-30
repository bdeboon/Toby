// LQR Controller

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros/time.h"


#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <sstream>

using namespace sensor_msgs;
using namespace message_filters;


//Initial Gains
/*
float K_LQR[2][5] = {
-11.4214, -5.9419, 170.0483, 7.0711, 123.6416,
-11.4214, -5.9419, 170.0483, -7.0711, -123.6416
};
*/
//A Little more agressive

float K_LQR[2][5] = {
-12.3562, -6.4751, 169.7837, 22.3607, 125.2383,
-12.3562, -6.4751, 169.7837, -22.3607, -125.2383,
};



//Most Aggressive
/*
float K_LQR[2][5] = {
-25.4305, -11.4675, 169.7837, 70.7107, 129.9393,
-25.4305, -11.4675, 169.7837, -70.7107, -129.9393,
};
*/


float Current_States[5][1] = {0,0,0,0,0};
float Desired_States[5][1] = {0,0,0,0,0};
float voltages[2][1] = {0,0};

float x_vel = 0;
float ang_vel_z = 0;
float ang_vel_x = 0;
float tilt = 0;
float heading = 0;


geometry_msgs::Vector3 input_voltage;
geometry_msgs::Vector3 kalman_states;
geometry_msgs::Twist twist_msg;
std_msgs::Int16 left_m;
std_msgs::Int16 right_m;


void twist_callback(const geometry_msgs::Twist &twist)
{
	x_vel = twist.linear.x;
  ang_vel_x = twist.angular.x;
  ang_vel_z = twist.angular.z;
}
void kalman_callback(const geometry_msgs::Vector3 &kf)
{
	tilt = kf.x;
  heading = kf.y;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "control_node");

  ros::NodeHandle nh;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
   ros::Publisher left_motor_pub = nh.advertise<std_msgs::Int16>("left_motor", 1);
   ros::Publisher right_motor_pub = nh.advertise<std_msgs::Int16>("right_motor", 1);
   ros::Subscriber kalman_sub = nh.subscribe("filtered_states", 1, kalman_callback);
   ros::Subscriber twist_sub = nh.subscribe("toby_velocity", 1, twist_callback);

	/*
  message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "imu", 1);
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "odom", 1);

  typedef sync_policies::ApproximateTime<sensor_msgs::Imu, nav_msgs::Odometry> MySyncPolicy;
  typedef Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync;
  sync.reset(new Sync(MySyncPolicy(10), imu_sub, odom_sub));

  //Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), imu_sub, odom_sub);
  //sync.registerCallback(boost::bind(&callback, _1, _2));
  */

  ros::Rate loop_rate(100);

  int count = 0;

  while (ros::ok())
  {
  //Fill up Current_States Matrix with Measurements
	Current_States[0][0] = tilt;
	Current_States[1][0] = ang_vel_x;
  Current_States[2][0] = x_vel;
	Current_States[3][0] = heading;
	Current_States[4][0] = ang_vel_z;

	//Calculate values for input voltage using K_LQR
	//voltages[0][0] = (K_LQR[0][0]*(Desired_States[0][0] - Current_States[0][0])) + (K_LQR[0][1]*Current_States[1][0]) + (K_LQR[0][2]*Current_States[2][0]) + (K_LQR[0][3]*(Desired_States[3][0] - Current_States[3][0])) + (K_LQR[0][4]*Current_States[4][0]);
	//voltages[1][0] = (K_LQR[1][0]*(Desired_States[0][0] - Current_States[0][0])) + (K_LQR[1][1]*Current_States[1][0]) + (K_LQR[1][2]*Current_States[2][0]) + (K_LQR[1][3]*(Desired_States[3][0] - Current_States[3][0])) + (K_LQR[1][4]*Current_States[4][0]);

  //Heading Dont Care
  voltages[0][0] = (4095/2)*(K_LQR[0][0]*(Desired_States[0][0] - Current_States[0][0])) + (K_LQR[0][1]*Current_States[1][0]) + (K_LQR[0][2]*Current_States[2][0]) + (K_LQR[0][3]*Current_States[3][0]) + (K_LQR[0][4]*Current_States[4][0]);
	voltages[1][0] = (4095/2)*(K_LQR[1][0]*(Desired_States[0][0] - Current_States[0][0])) + (K_LQR[1][1]*Current_States[1][0]) + (K_LQR[1][2]*Current_States[2][0]) + (K_LQR[1][3]*Current_States[3][0]) + (K_LQR[1][4]*Current_States[4][0]);


  //Save voltages as rosmsg
  left_m.data = (int) (voltages[0][0]); //calm down desired voltage a little (driver expects 4095 as max)
  right_m.data = (int) (voltages[1][0]);

  //Publish motor voltage data
	left_motor_pub.publish(left_m);
  right_motor_pub.publish(right_m);

  ros::spinOnce();
  loop_rate.sleep();

  }


  return 0;
}
