#include "ros/ros.h"
#include "local_planner/AddTwoInts.h"
#include "local_planner/GetNextCoordinates.h"

int call_counter;

bool next_coordinates(//local_planner::AddTwoInts::Request  &req,
         //local_planner::AddTwoInts::Response &res,
         local_planner::GetNextCoordinates::Request &req,
         local_planner::GetNextCoordinates::Response &res)
{
  // setting return value of the response 
  res.nextX = 1;
  res.nextY = 2;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.x, (long int)req.y);

  int nextX = 1;
  int nextY = 2;

  // counts how often this function is called
  call_counter++;

  ROS_INFO("sending back response: next x=%ld, next y =%ld", (long int) nextX, (long int) nextY);
  return true;
}

int main(int argc, char **argv)
{
  
  call_counter = 0;
   
  ros::init(argc, argv, "get_next_coordinates_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("get_next_coordinates", next_coordinates);
 
    ROS_INFO("Ready to get next coordinates");
  
  ros::spin();

  return 0;
}