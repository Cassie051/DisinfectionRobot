#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import os, math, time
import motion_plan.disinfection

# parameters
x_vel = 1
t = 5

ang_z = -1
vel_round = 1


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

def ChoseMove(pick, msg, pub, go, disinfection):
    # go round
    if(pick == 'a'):
        while go:
            Round(msg)
            pub.publish(msg)
        Stop(msg)
        pub.publish(msg)   
    # go stright 
    elif(pick == 'b'):
        while go[0]:   
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
        Stop(msg)
        pub.publish(msg) 
    # pure pursuit
    elif(pick == 'c'):
        disinfection.DoPurepursuite()
    else:
        print("Unknow option")

if __name__ == '__main__':
    try:
        rospy.init_node('robot_move')
        msg = Twist()
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        chose = 'c'
        ChoseMove(chose, msg, pub, True, None)
    except rospy.ROSInterruptException:
        pass