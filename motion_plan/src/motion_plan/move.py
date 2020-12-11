#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import os, math, time

# parameters
x_vel = 1.5
t = 3.333

ang_z = -math.pi/32
vel_round = 0.1


def Round(msg):
    msg.linear.x = vel_round
    msg.angular.z = ang_z

def GoStright(msg):
    msg.linear.x = x_vel

def TurnLeft(msg):
    msg.linear.x = 0.01
    msg.angular.z = -math.pi/2

def TurnRight(msg):
    msg.linear.x = 0.01
    msg.angular.z = math.pi/2

def TurnRound(msg):
    msg.linear.x = 0.001
    msg.angular.z = -math.pi

def Stop(msg):
    msg.angular.x, msg.angular.y, msg.angular.z = 0, 0, 0
    msg.linear.x, msg.linear.y, msg.linear.z = 0, 0, 0

def ChoseMove(pick, msg, pub):
    # go round
    if(pick == 'a'):
        while True:
            Round(msg)
            pub.publish(msg)
    # go stright 
    elif(pick == 'b'):
        while True:   
            Stop(msg)
            pub.publish(msg)
            time.sleep(2)
            
            GoStright(msg)
            pub.publish(msg)
            time.sleep(t)
            
            Stop(msg)
            pub.publish(msg)
            time.sleep(2)
            
            TurnRound(msg)
            pub.publish(msg)
            time.sleep(1.05)
    # wall follow
    elif(pick == 'c'):
        os.system("rosrun two-wheeled-robot-motion-planning follow_wall.py")
    else:
        print("Unknow option")

if __name__ == '__main__':
    try:
        rospy.init_node('robot_move')
        msg = Twist()
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        chose = 'c'
        ChoseMove(chose, msg, pub)
    except rospy.ROSInterruptException:
        pass