cmake_minimum_required(VERSION 2.8.3)
project(dvs_simulator_py)

find_package(catkin REQUIRED COMPONENTS
  rospy
  dvs_msgs
)

catkin_package(
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

