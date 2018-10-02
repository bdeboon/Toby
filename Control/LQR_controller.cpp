// LQR Controller

#include "ros/ros.h"
#include "std_msgs/String.h"


#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <sstream>

using namespace sensor_msgs;
using namespace message_filters;
using namespace nav_msgs;
using namespace geometry_msgs;


void callback(const sensor_msgs::Imu::ConstPtr& imu, const nav_msgs::Odometry::ConstPtr& odom)
{
  



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
  ros::Publisher motor = nh.advertise<geometry_msgs::Vector3>("motor_voltages", 1000);
  
  message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "imu", 1);
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "odom", 1);
  
  typedef sync_policies::ApproximateTime<sensor_msgs::Imu, nav_msgs::Odometry> MySyncPolicy;
  typedef Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync;
  sync.reset(new Sync(MySyncPolicy(10), imu_sub, odom_sub));

  //Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), imu_sub, odom_sub);
  //sync.registerCallback(boost::bind(&callback, _1, _2));
  
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    //std_msgs::String msg;

    //std::stringstream ss;
    //ss << "hello world " << count;
    //msg.data = ss.str();

    //ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    //chatter_pub.publish(msg);
//
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
