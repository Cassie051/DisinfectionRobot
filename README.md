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
In [make_disinfection.py](motion_plan/scripts/make_disinfection.py) you can select the way of robot move.
```
To make sure your workspace is properly overlayed by the setup script.
$ source <catkin_ws>/devel/setup.sh

Launching Gazebo&Rviz
$ roslaunch robot_description world.launch

Run program
$ rosrun motion_plan make_disinfection.py
```
# To test the particular module
```
Keyborad move robot
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py key_vel=cmd_vel

Test move module
$ rosrun motion_plan move.py

Test robot module
$ rosrun motion_plan robot.py

Test map module
$ rosrun motion_plan mapy.py

Test disinfection module
$ rosrun motion_plan disinfection.py
```
