cmake_minimum_required(VERSION 2.8.3)
project(local_planner)



## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg message_generation)

include_directories(${catkin_INCLUDE_DIRS})

## Declare ROS messages and services
add_message_files(DIRECTORY msg FILES Num.msg coordinates.msg)

## Declare ROS messages and services
add_service_files(
  FILES
  AddTwoInts.srv
  GetNextCoordinates.srv
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## -------------------------- publisher subscriber
add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})

## -------------------------- server client
add_executable(server src/server.cpp)
target_link_libraries(server ${catkin_LIBRARIES})


add_executable(client src/client.cpp)
target_link_libraries(client ${catkin_LIBRARIES})

