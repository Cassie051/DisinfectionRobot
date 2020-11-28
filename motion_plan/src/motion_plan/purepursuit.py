#! /usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.srv import GetMap
from robot import Robot
from mapy import Map


class PurePursuit:
    def __init__(self, robot, loaded_map):
        self.robot = robot
        self.loaded_map = loaded_map
        self.r = 0
        self.L2 = 25
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.msg = Twist()
        self.CountCord()

    def __del__(self):
        del self.robot
        del self.pub
        del self.r
        del self.L2
        del self.msg

    def CountCord(self):
        self.robotCordX = int((self.robot.position.x - self.loaded_map.origin.position.x)/self.loaded_map.resolution)
        self.robotCordY = int((self.robot.position.y - self.loaded_map.origin.position.y)/self.loaded_map.resolution)

    def FindCurrentWaypoint(self):
        ACCURACY = 0
        for goal in self.robot.goalPointsonMap:
            xDistance = (self.robotCordX - goal[0])*self.loaded_map.resolution
            yDistance = (self.robotCordY- goal[1])*self.loaded_map.resolution
            if(self.L2 - xDistance*xDistance - yDistance*yDistance < ACCURACY):
                self.CalculateR(goal)
                while(not (self.robotCordX == goal[0] and self.robotCordY == goal[1])):
                    self.RobotMove()
                    self.CountCord()
                ACCURACY = 0
                self.robot.goalPointsonMap.pop(self.robot.goalPointsonMap.index(goal))
            else:
                ACCURACY+=1

    def CalculateR(self, goal):
        # xDistance = abs(self.robot.position.x - goal[0])
        yDistance = abs(self.robot.position.y - goal[1])*self.loaded_map.resolution
        self.r = self.L2/(2*yDistance)
        
    def RobotMove(self):
        self.robot.linear_vel_x = 0.1
        self.robot.angular_z = self.robot.linear_vel_x/self.r
        self.msg.linear.x = self.robot.linear_vel_x
        self.msg.angular.z = self.robot.angular_z
        self.pub.publish(self.msg)


        # TODO: find the current waypoint to track using methods mentioned in lecture

        # TODO: transform goal point to vehicle frame of reference

        # TODO: calculate curvature/steering angle

        # TODO: publish drive message, don't forget to limit the steering angle between -0.4189 and 0.4189 radians    
    

if __name__ == '__main__':
    dis_robot = Robot()
    rospy.init_node('robot_position', anonymous=True)
    rospy.wait_for_service('static_map')
    mapsrv = rospy.ServiceProxy('static_map', GetMap)
    result = mapsrv()
    loaded_map = Map(result.map.info.resolution, result.map.info.height, result.map.info.width, result.map.info.origin, result.map.data)
    algorythm = PurePursuit(dis_robot, loaded_map)
    algorythm.FindCurrentWaypoint()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass