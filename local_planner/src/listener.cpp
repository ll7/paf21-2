#include "ros/ros.h"
#include "std_msgs/String.h"
#include "local_planner/coordinates.h"
#include <geometry_msgs/Twist.h>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void coordinatesCallback(const geometry_msgs::Twist& msg)
{
  ROS_INFO("I received coordinates(%lf | %lf) ",  msg.linear.x , msg.linear.y);
}

int main(int argc, char **argv)
{

  
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("coordinates_publisher", 1000, coordinatesCallback);

  ros::spin();

  return 0;
}