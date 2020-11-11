# Disinfection Robot
Simulation of UV disinfection process by a mobile robot in ROS

# To install:
```
$ cd <catkin_ws>/src
$ git clone https://github.com/Cassie051/DisinfectionRobot.git
$ cd ..
$ catkin_make
```

# To use:
```
To make sure your workspace is properly overlayed by the setup script.
$ source <catkin_ws>/devel/setup.sh
Launching Gazebo
$ roslaunch disinfection_robot dis_robot.launch

Move robot
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py key_vel=cmd_vel
```
