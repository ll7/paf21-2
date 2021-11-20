cmake_minimum_required(VERSION 2.8.3)
project(helper_functions)

find_package(catkin REQUIRED COMPONENTS rospy roslaunch)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS rospy)

catkin_install_python(PROGRAMS src/helper_functions.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
