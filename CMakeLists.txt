cmake_minimum_required(VERSION 2.8.3)
project(dvs_simulator_py)

find_package(catkin REQUIRED COMPONENTS
  rospy
  dvs_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

