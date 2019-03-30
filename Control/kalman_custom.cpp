// Kalman Filter

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros/time.h"
#include <eigen3/Eigen/Dense>
#include <math.h>

#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>
#include <sstream>
#include <iostream>
#include <stdexcept>

float f_uson;
float r_uson;

sensor_msgs::Imu imu_measured;
std_msgs::Float32 f;
std_msgs::Float32 r;
geometry_msgs::Vector3 kalman_states;

float l = 350; //350 mm between sensors

float mean_ru = 120.184;
float var_ru = 10.1906;

float mean_fu = 119.30;
float var_fu = 4.68782;

float var_imu_tilt = 0.0003125;
float var_imu_heading = 0.0008;

float us_var_fr = atan2(sqrt(pow(var_fu,2) + pow(var_ru,2)),l);
float L1_tilt, L2_tilt;

float tilt_measurement = 0;
float heading_measurement = 0;

float tilt_imu;
float tilt_uson = 0 ;

void imu_callback(const sensor_msgs::Imu &imu)
{
	imu_measured = imu;
}
void fuson_callback(const std_msgs::Float32 &f)
{
	r_uson = f.data*23.29346;
}
void ruson_callback(const std_msgs::Float32 &r)
{
	f_uson = r.data*22.9344;
}


int main(int argc, char **argv)
{
  int n = 5; // Number of states [tilt,tilt_vel,linear_vel,heading,heading_vel]
  int m = 2; // Number of measurements [tilt,heading]

  Eigen::MatrixXf A(n, n); // System dynamics matrix
  Eigen::MatrixXf C(m, n); // Output matrix
  Eigen::MatrixXf Q(n, n); // Process noise covariance
  Eigen::MatrixXf R(m, m); // Measurement noise covariance
  Eigen::MatrixXf P(n, n); // Estimate error covariance
  Eigen::MatrixXf K(n, m);
  Eigen::MatrixXf I(n,n); //Identity matrix

  // Estimated states
  Eigen::VectorXf x_hat_new;

  // Discrete LTI projectile motion, measuring position only
  A << 0, 1, 0, 0, 0, -61.1768, 0, -9.8557, 0, 0, -1.7387, 0, 1.5766, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 9.5562;
  C << 1, 0, 0, 0, 0, 0, 0, 0, 1, 0;

  // Reasonable covariance matrices
  Q << 10, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 1;
  R << 1, 0, 0, 1;
  P << 10, 10, 1, 0.1, 0.1, 10, 10, 1, 0.1, 0.1, 1, 1, 10, 1, 1, 0.1, 0.1, 1, 10, 10, 0.1, 0.1, 1, 10, 10;

  I << 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1;
  //initial states
  Eigen::VectorXf x_hat(5);
  x_hat << 0, 0, 0, 0, 0;

  //Measurements
  Eigen::VectorXf y(m);

  float L1 = (pow(us_var_fr,2))/(pow(us_var_fr,2) + pow(var_imu_tilt,2) + pow(var_imu_tilt,2)*pow(us_var_fr,2));
  float L2 = (pow(var_imu_tilt,2))/(pow(us_var_fr,2) + pow(var_imu_tilt,2) + pow(var_imu_tilt,2)*pow(us_var_fr,2));


  ros::init(argc, argv, "filter_node");

  ros::NodeHandle nh;
  ros::Publisher state_pub = nh.advertise<geometry_msgs::Vector3>("filtered_states", 10);
  ros::Subscriber imu_sub = nh.subscribe("imu", 10, imu_callback);
  ros::Subscriber ruson_sub = nh.subscribe("rear_uson", 10, ruson_callback);
  ros::Subscriber fuson_sub = nh.subscribe("front_uson", 10, fuson_callback);
  ros::Rate loop_rate(100);

  double dt = 1.0/30; // Time step
  double t0 = ros::Time::now().toSec();
  double run_time;


  while (ros::ok())
  {
    tilt_imu = imu_measured.orientation.x; //To put in rads

    //tilt_uson = atan2((f_uson - r_uson),l)/2;
    //tilt_measurement = L1*tilt_imu + L2*tilt_uson;
		tilt_measurement = tilt_imu;

    heading_measurement = imu_measured.orientation.z;

    if(run_time > 10) { //Account for drift
      heading_measurement = heading_measurement - 0.00003868*(run_time-10);
    }

    y << tilt_measurement, heading_measurement;

    x_hat_new = A * x_hat;
    P = A*P*A.transpose() + Q;
    K = P*C.transpose()*(C*P*C.transpose() + R).inverse();
    x_hat_new += K * (y - C*x_hat_new);

    kalman_states.x = x_hat_new(1)*4/110;
    kalman_states.y = x_hat_new(2)*4/110;

		P = (I - K*C)*P;
    x_hat = x_hat_new;

    dt = ros::Time::now().toSec();
    run_time = dt - t0;

	  state_pub.publish(kalman_states);
    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}
