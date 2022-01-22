cmake_minimum_required(VERSION 2.8.3)
project(paf_messages)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  FloatArray.msg
  PafLocalPath.msg
  PafObstacle.msg
  PafObstacleFollowInfo.msg
  PafObstacleList.msg
  Point2D.msg
  PafLaneletRoute.msg
  PafRoutingRequest.msg
  PafRoutingGraphNode.msg
  PafTrafficSignal.msg
  PafLogImage.msg
  PafLogScalar.msg
  PafLogText.msg
  PafTopDownViewPointSet.msg
  PafSpeedMsg.msg
)

#add_service_files(
#  FILES
#)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
  sensor_msgs
)
