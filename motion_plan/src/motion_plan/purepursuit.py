#! /usr/bin/env python

import rospy
import math
import sys, time
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.srv import GetMap
from motion_plan.robot import Robot
from motion_plan.mapy import Map
from copy import deepcopy



class PurePursuit:
    def __init__(self, robot, loaded_map):
        self.robot = robot
        self.loaded_map = loaded_map
        # self.Lf = 2
        self.Lfc = 2
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.msg = Twist()
        self.CountCord()
        self.old_nearest_point_index = None
        self.MoveTime = 0

    def __del__(self):
        del self.robot
        del self.loaded_map
        del self.pub
        del self.Lfc
        del self.msg
        del self.old_nearest_point_index

    def CountCord(self):
        self.robotCordX = int((self.robot.position.x - self.loaded_map.origin.position.x)/self.loaded_map.resolution)
        self.robotCordY = int((self.robot.position.y - self.loaded_map.origin.position.y)/self.loaded_map.resolution)

    def FindCurrentWaypoint(self):
        if self.old_nearest_point_index is None:
            xDistance = [abs(self.robotCordX - goal[0])/self.loaded_map.resolution for goal in self.robot.goalPointsonMap]
            yDistance = [abs(self.robotCordY - goal[1])/self.loaded_map.resolution for goal in self.robot.goalPointsonMap]
            d = np.hypot(xDistance, yDistance)
            # newd = []
            # for onedist in d:
            #     if (onedist != 0):
            #         newd.append(d)
            # index = np.argmin(newd)
            index = np.argmin(d)
            self.old_nearest_point_index = index
        else:
            index = self.old_nearest_point_index
            distanceToIndex = math.hypot((self.robotCordX - self.robot.goalPointsonMap[index][0])/self.loaded_map.resolution, (self.robotCordY - self.robot.goalPointsonMap[index][1])/self.loaded_map.resolution)
            while True:
                distanceNextIndex = math.hypot((self.robotCordX - self.robot.goalPointsonMap[index+1][0])/self.loaded_map.resolution, (self.robotCordY - self.robot.goalPointsonMap[index+1][1])/self.loaded_map.resolution)
                if distanceToIndex < distanceNextIndex:
                    break
                index = index+1 if (index +1) < len(self.robot.goalPointsonMap) else index
                distanceToIndex = distanceNextIndex
            self.old_nearest_point_index = index

        execTime = time.time() - self.MoveTime
        Lf = execTime * self.robot.linear_vel_x + self.Lfc

        while Lf > math.hypot((self.robotCordX - self.robot.goalPointsonMap[index+1][0])/self.loaded_map.resolution, (self.robotCordY - self.robot.goalPointsonMap[index+1][1])/self.loaded_map.resolution):
            if(index + 1) >= len(self.robot.goalPointsonMap):
                break
            index += 1
        return index, Lf

    def RobotMove(self, delta):
        self.robot.linear_vel_x = 0.1
        target = delta - self.robot.orientation[2]
        while(target > math.pi or target < -1*math.pi):
            target = target - np.sign(target)*math.pi
        self.robot.angular_vel_z = target
        self.msg.linear.x = self.robot.linear_vel_x
        self.msg.angular.z =  self.robot.angular_vel_z

    def Algorythm(self, pIndex):
        self.CountCord()
        index, Lf = self.FindCurrentWaypoint()
        trajectory = []
        if pIndex >= index:
            index = pIndex

        if index < len(self.robot.goalPointsonMap):
            trajectory = self.robot.goalPointsonMap[index]
        else:  # toward goal
            trajectory = self.robot.goalPointsonMap[0]
            self.old_nearest_point_index = None

        print(trajectory)

        alpha = math.atan2((trajectory[1] - self.robotCordY)/self.loaded_map.resolution, (trajectory[0] - self.robotCordX)/self.loaded_map.resolution) - self.robot.orientation[2] ## YAW
        delta = math.atan2(2.0 * self.Lfc *math.sin(alpha) / Lf, 1.0)
        return delta, index
