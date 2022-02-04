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
  PafLocalPath.msg
  PafObstacle.msg
  PafObstacleFollowInfo.msg
  PafObstacleList.msg
  Point2D.msg
  PafRoutingRequest.msg
  PafRoutingGraphNode.msg
  PafTrafficSignal.msg
  PafLogImage.msg
  PafLogScalar.msg
  PafLogText.msg
  PafTopDownViewPointSet.msg
  PafSpeedMsg.msg
  PafDetectedTrafficLights.msg
  PafLaneletRoute.msg
  PafRouteSection.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  PafRoutingService.srv
)

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
