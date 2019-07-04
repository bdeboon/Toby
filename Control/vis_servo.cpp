// Motor Speed Controller
// Ensure DEBUG_MOTOR Flag is high in Arduino Sensor Script

#include "ros/ros.h"
#include "ros/time.h"

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <sstream>

using namespace geometry_msgs;

///////////// Params //////////////////////////////////

int max_forward_speed = 2000; // Max forward speed
int max_turn_speed = 1200; //Max turn speed
int xref = 350; // Reference delta x in pixels
int Kpf = 8; // Proportianl forward contribution
int Kdf = 0.2; //Derivative forward contribution
int Kpr = 1.5; //Proporational rotation contribution
int Kdr = 0.01; //Derivative rotation contribution
 
///////////////////////////////////////////////////////

darknet_ros_msgs::BoundingBoxes box_found;
std_msgs::Int16 left_motor;
std_msgs::Int16 right_motor;

int xmin, xmax, ymin, ymax, lm, rm;
int errf = 0; //Last sample error for forward deviations
int errr = 0; //Last sample error for rotational deviations
bool person = false;

void box_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& box)
{
	xmin = box->bounding_boxes[0].xmin;
	xmax = box->bounding_boxes[0].xmax;
	ymin = box->bounding_boxes[0].ymin;
	ymax = box->bounding_boxes[0].ymax;
	person = true;	
	
}
int main(int argc, char **argv)
{

  ros::init(argc, argv, "speed_control_node");

  ros::NodeHandle nh;

  ros::Publisher left_motor_pub = nh.advertise<std_msgs::Int16>("left_motor", 100);
  ros::Publisher right_motor_pub = nh.advertise<std_msgs::Int16>("right_motor", 100);

  ros::Subscriber box_sub = nh.subscribe("darknet_ros/bounding_boxes", 10, box_callback);

  ros::Rate loop_rate(10);

  ROS_INFO("Visual Servoing Node Initialized");

  while (ros::ok())
  {
	/*

	Controller Breakdown 	

	// For forward error
  	// Calculate deltax
	delta_x = xmax - xmin;
	//reference delta x approx 340 pixels
	errorx = xref - delta_x;
	// PD controller
	forward_contribution = Kpf*errorx + Kdf*delx;
	errf = xref - (xmax - xmin);
	//For rotational error
	//Try to center box, resolution = 720
	//(720 - (delta_x))/2; // Where x min should be
	center_error = xmin - ((720 - (delta_x))/2); // Where x min should be
	//PD Rotational contribution
	rotational_contribution = Kpr*center_error + Kdr*delr;
	errr = xmin - ((720 - (xmax - xmin))/2);

	left_motor.data = forward_contribution + rotational_contribution;
	right_motor.data = forward_contribution - rotational_contribution;

	*/
	if(person) {
		if((xmax - xmin) < 650) { //if person close
			if((xref - (xmax - xmin)) < 0) { //Ensure straight reverse direction
				left_motor.data = 0.8*(Kpf*(xref - (xmax - xmin)) + Kdf*((xref - (xmax - xmin)) - errf));
				right_motor.data = 0.8*(Kpf*(xref - (xmax - xmin)) + Kdf*((xref - (xmax - xmin)) - errf));
			} 
			else { //Include rotation
  				left_motor.data = (Kpf*(xref - (xmax - xmin)) + Kdf*((xref - (xmax - xmin)) - errf)) + (Kpr*(xmin - ((720 - (xmax - xmin))/2)) + Kdr*(((720 - (xmax - xmin))/2) - errr));
  				right_motor.data = (Kpf*(xref - (xmax - xmin)) + Kdf*((xref - (xmax - xmin)) - errf)) - (Kpr*(xmin - ((720 - (xmax - xmin))/2)) + Kdr*(((720 - (xmax - xmin))/2) - errr));
			}
			errf = xref - (xmax - xmin);
			errr = xmin - ((720 - (xmax - xmin))/2);
		}
		else {
			right_motor.data = 0;
			left_motor.data = 0;
		}

		left_motor_pub.publish(left_motor);
  		right_motor_pub.publish(right_motor);
		person = false;
  		ros::spinOnce();
		loop_rate.sleep();
	}
	else {
		right_motor.data = 0;
		left_motor.data = 0;
		left_motor_pub.publish(left_motor);
  		right_motor_pub.publish(right_motor);
		ros::spinOnce();
		loop_rate.sleep();
	}
}
	return 0;
}
