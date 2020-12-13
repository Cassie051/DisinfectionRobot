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
        self.Lfc = 0.5
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.msg = Twist()
        self.old_nearest_point = []
        self.nearest_point = None

    def __del__(self):
        del self.robot
        del self.loaded_map
        del self.pub
        del self.Lfc
        del self.msg
        del self.old_nearest_point
        del self.nearest_point

    def SortGoals(self):
        self.robot.goalPointsonMap.sort(key=lambda x:math.hypot(self.robot.onMapPosition[0] - x[0], self.robot.onMapPosition[1] - x[1]))
    
    def FindCurrentWaypoint(self):
        Lf =  self.robot.linear_vel_x
        if(Lf == 0):
            Lf = 1

        if self.nearest_point is None:
            self.SortGoals()
            for goal in self.robot.goalPointsonMap:
                    d = math.hypot((self.robot.onMapPosition[0] - goal[0])*self.loaded_map.resolution, 
                                    (self.robot.onMapPosition[1] - goal[1])*self.loaded_map.resolution)
                    if(self.Lfc < d):
                        self.old_nearest_point = [[-1, -1], [-1, -1], [-1, -1], [-1, -1], [-1, -1]]
                        self.nearest_point = goal
                        break
        else:
            distanceToIndex = math.hypot((self.robot.onMapPosition[0] - self.nearest_point[0])*self.loaded_map.resolution, 
                                        (self.robot.onMapPosition[1] - self.nearest_point[1])*self.loaded_map.resolution)
            while distanceToIndex < self.Lfc:
                self.SortGoals()
                i = 0
                while (self.robot.goalPointsonMap[i] in self.old_nearest_point or self.robot.goalPointsonMap[i] == self.nearest_point ):
                    i += 1
                self.old_nearest_point.append(self.nearest_point)
                self.old_nearest_point.pop(0)
                self.nearest_point = self.robot.goalPointsonMap[i] 
                distanceToIndex = math.hypot((self.robot.onMapPosition[0] - self.nearest_point[0])*self.loaded_map.resolution, 
                                            (self.robot.onMapPosition[1] - self.nearest_point[1])*self.loaded_map.resolution)
        return Lf

    def RobotMove(self, delta):
        self.robot.linear_vel_x = 0.2
        target = delta
        while(target > math.pi or target < -1*math.pi):
            target = target - np.sign(target)*math.pi
        self.robot.angular_vel_z = target
        self.msg.linear.x = self.robot.linear_vel_x
        self.msg.angular.z =  self.robot.angular_vel_z

    def Algorythm(self):
        Lf = self.FindCurrentWaypoint()

        alpha = math.atan2((self.robot.onMapPosition[1] - self.nearest_point[1] )*self.loaded_map.resolution, 
                            (self.robot.onMapPosition[0] - self.nearest_point[0])*self.loaded_map.resolution) - self.robot.orientation[2] ## YAW
        delta = math.atan2(2.0 * self.Lfc *math.sin(alpha) / Lf, 1.0)
        return delta
