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
$ roslaunch robot_description world.launch

Move robot
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py key_vel=cmd_vel

Test robot module
$ rosrun motion_plan robot.py

Test map module
$ rosrun motion_plan mapy.py

Test disinfection module
$ rosrun motion_plan disinfection.py
```
