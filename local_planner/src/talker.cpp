#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include "local_planner/coordinates.h"
#include <geometry_msgs/PoseStamped.h>

#include <sstream>

using namespace std;

vector<geometry_msgs::PoseStamped> path;

geometry_msgs::PoseStamped nearestPositon(const geometry_msgs::PoseStamped &current_pose)
{
  double min_distance, x_diff, y_diff, distance;
  geometry_msgs::PoseStamped closesPoint;

  // search for closest path point to the current position
  for (auto waypoint : path)
  {
    x_diff = current_pose.pose.position.x - waypoint.pose.position.x;
    y_diff = current_pose.pose.position.y - waypoint.pose.position.y;
    distance = x_diff * x_diff + y_diff * y_diff;

    if(distance < min_distance) 
    {
      min_distance = distance;
      closesPoint = waypoint;
    }
  }

  return closesPoint;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "talker");

  // main access point to communication
  ros::NodeHandle n;

  ros::Publisher coordinates_pub = n.advertise<geometry_msgs::Twist>("coordinates_publisher", 1000);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */

  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */

    vector<tuple<int, int>> coordinates;

    geometry_msgs::Twist twist_msg;
  std:

    geometry_msgs::PoseStamped posStamped1;
    posStamped1.pose.position.x = 1;
    posStamped1.pose.position.y = 1;

    geometry_msgs::PoseStamped posStamped2;
    posStamped2.pose.position.x = 2;
    posStamped2.pose.position.y = 2;

    geometry_msgs::PoseStamped posStamped3;
    posStamped3.pose.position.x = 2;
    posStamped3.pose.position.y = 2;

    geometry_msgs::PoseStamped posStamped4;
    posStamped4.pose.position.x = 2;
    posStamped4.pose.position.y = 2;

    geometry_msgs::PoseStamped posStamped5;
    posStamped5.pose.position.x = 2;
    posStamped5.pose.position.y = 2;

    path.emplace_back(posStamped1);
    path.emplace_back(posStamped2);
    path.emplace_back(posStamped3);
    path.emplace_back(posStamped4);
    path.emplace_back(posStamped5);

    twist_msg.linear.x = 1;
    twist_msg.linear.y = 1;

    ROS_INFO("publishing coordinates %ld | %ld", (long int)twist_msg.linear.x, (long int)twist_msg.linear.y);
    coordinates_pub.publish(twist_msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}