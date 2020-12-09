#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import os, math, time

# parameters
x_vel = 1
ang_z = 0.4


def Round(msg):
    msg.linear.x = x_vel
    msg.angular.z = ang_z

def GoStright(msg):
    msg.linear.x = x_vel

def TurnLeft(msg):
    msg.angular.z = math.pi/2

def TurnRight(msg):
    msg.angular.z = -math.pi/2

def TurnRound(msg):
    msg.angular.z = -math.pi

def Stop(msg):
    msg.angular.x, msg.angular.y, msg.angular.z = 0, 0, 0
    msg.linear.x, msg.linear.y, msg.linear.z = 0, 0, 0

def choseMove(pick, msg, pub):
    # go round
    if(pick == 'a'):
        GoStright(msg)
        Round(msg)
        pub.publish(msg)
        time.sleep(10)
    # go stright 
    elif(pick == 'b'):
        TurnLeft(msg)
        pub.publish(msg)
        GoStright(msg)
        pub.publish(msg)
        Stop(msg)
        pub.publish(msg)
        TurnRight(msg)
        while True:    
            GoStright(msg)
            Stop(msg)
            TurnRound(msg)
    # wall follow
    elif(pick == 'c'):
        os.system("rosrun two-wheeled-robot-motion-planning follow_wall.py")
    else:
        print("Unknow option")

if __name__ == '__main__':
    rospy.init_node('robot_move')
    msg = Twist()
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    chose = 'a'
    choseMove(chose, msg, pub)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass