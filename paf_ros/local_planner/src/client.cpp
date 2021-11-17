#include "ros/ros.h"
#include "local_planner/AddTwoInts.h"
#include <cstdlib>
#include "local_planner/GetNextCoordinates.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_next_coordinates_client");

  ros::NodeHandle n;
  // ros::ServiceClient client = n.serviceClient<local_planner::AddTwoInts>("add_two_ints");
   ros::ServiceClient client = n.serviceClient<local_planner::GetNextCoordinates>("get_next_coordinates");
  //local_planner::AddTwoInts srv;
  local_planner::GetNextCoordinates srv;

  

  int i = 0;

  for (int i = 0; i < 10; i++)
  {
 
    ROS_INFO("in for loop");
    srv.request.x = i;
    srv.request.y = i + 1;

    if (client.call(srv))
    {
      ROS_INFO("Next coordinates: nextX = %ld, nextY = %ld", (long int) srv.response.nextX, (long int) srv.response.nextY);
    }
    else
    {
      ROS_ERROR("Failed to call service get_next_coordinates");
    }
  }

  return 0;
}
