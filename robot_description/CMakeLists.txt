cmake_minimum_required(VERSION 3.0.2)
project(robot_description)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  urdf
  xacro
  gazebo_ros
  controller_manager
  joint_state_controller
  robot_state_publisher
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
