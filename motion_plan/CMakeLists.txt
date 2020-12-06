cmake_minimum_required(VERSION 3.0.2)
project(motion_plan)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES motion_plan
#  CATKIN_DEPENDS geometry_msgs rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   sript
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
