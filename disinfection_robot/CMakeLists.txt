cmake_minimum_required(VERSION 3.0.2)
project(disinfection_robot)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES disinfection_robot
 CATKIN_DEPENDS gazebo_ros roscpp rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/move.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
