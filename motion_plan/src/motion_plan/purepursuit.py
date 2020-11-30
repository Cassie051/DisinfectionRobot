#! /usr/bin/env python

import rospy
import math
import sys
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.srv import GetMap
from robot import Robot
from mapy import Map
from copy import deepcopy


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
        ACCURACY = 1
        index = len(self.robot.goalPointsonMap)-1
        while(len(self.robot.goalPointsonMap) != 0):
            xDistance = (self.robotCordX - self.robot.goalPointsonMap[index][0])*self.loaded_map.resolution
            yDistance = (self.robotCordY - self.robot.goalPointsonMap[index][1])*self.loaded_map.resolution
            if(abs(self.L2 - xDistance*xDistance - yDistance*yDistance) <= ACCURACY):
                self.CalculateR(index)
                # while(not (self.robotCordX == goal[0] and self.robotCordY == goal[1])):
                self.RobotMove()
                self.CountCord()
                self.robot.goalPointsonMap.pop(index)
            index -= 1
            if(index < 0):
                self.AddNewPoint()
                index = len(self.robot.goalPointsonMap)-1

    def AddNewPoint(self):
        minGoal = self.FindNearestPoints()
        goal = self.CalculateNewPoint(minGoal)
        self.robot.goalPointsonMap.append([int(goal[0]), int(goal[1])])

    def FindNearestPoints(self):
        MIN1, MIN2 = int(sys.maxsize), int(sys.maxsize)
        wayPoints = deepcopy(self.robot.goalPointsonMap)
        minGoals =[]
        index = len(self.robot.goalPointsonMap)-1
        while(index >= 0):
            xDistance = (self.robotCordX - wayPoints[index][0])*self.loaded_map.resolution
            yDistance = (self.robotCordY - wayPoints[index][1])*self.loaded_map.resolution
            if(abs(self.L2 - xDistance*xDistance - yDistance*yDistance) < MIN1):
                MIN1 = abs(self.L2 - xDistance*xDistance - yDistance*yDistance)
                minindex = index
            index -= 1
        minGoals.append(wayPoints[minindex])
        wayPoints.pop(minindex)
        index = len(wayPoints)-1
        while(index >= 0):
            xDistance = (self.robotCordX - wayPoints[index][0])*self.loaded_map.resolution
            yDistance = (self.robotCordY - wayPoints[index][1])*self.loaded_map.resolution
            if(abs(self.L2 - xDistance*xDistance - yDistance*yDistance) < MIN2):
                MIN2 = abs(self.L2 - xDistance*xDistance - yDistance*yDistance)
                minindex = index
            index -= 1
        minGoals.append(wayPoints[minindex])
        return minGoals
        
    def CalculateNewPoint(self, foundGoal):
        goal =[]
        cal = (foundGoal[0][1]-foundGoal[1][1])/(foundGoal[0][0]-foundGoal[1][0])
        # steps = abs(foundGoal[1][0] - foundGoal[0][0]) * 10
        accuracy = 0.001
        for x in np.arange(foundGoal[0][0], foundGoal[1][0], accuracy):
           if(round(self.L2-x**2, 2) == round(cal**2*(x**2-foundGoal[0][0]**2)+foundGoal[0][1]**2-2*cal*foundGoal[0][0], 2)):
                goal.append(x)
                goal.append(math.sqrt(self.L2-x**2))
        return goal

    def CalculateR(self, index):
        # xDistance = abs(self.robot.position.x - goal[0])
        yDistance = abs(self.robot.position.y - self.robot.goalPointsonMap[index][1])*self.loaded_map.resolution
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