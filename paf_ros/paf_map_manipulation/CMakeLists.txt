cmake_minimum_required(VERSION 3.0.2)
project(paf_map_manipulation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
