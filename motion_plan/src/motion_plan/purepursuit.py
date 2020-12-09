#! /usr/bin/env python

import rospy
import math
import sys, time
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.srv import GetMap
from robot import Robot
from mapy import Map
from copy import deepcopy

Kp = 1.0
k = 0.1


class PurePursuit:
    def __init__(self, robot, loaded_map):
        self.robot = robot
        self.loaded_map = loaded_map
        self.Lf = 2
        self.Lfc = 1.5
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.msg = Twist()
        self.CountCord()
        self.old_nearest_point_index = None

    def __del__(self):
        del self.robot
        del self.pub
        del self.Lfc
        del self.msg

    def CountCord(self):
        self.robotCordX = int((self.robot.position.x - self.loaded_map.origin.position.x)/self.loaded_map.resolution)
        self.robotCordY = int((self.robot.position.y - self.loaded_map.origin.position.y)/self.loaded_map.resolution)

    def FindCurrentWaypoint(self):
        if self.old_nearest_point_index is None:
            xDistance = [abs(self.robotCordX - goal[0]) for goal in self.robot.goalPointsonMap]
            yDistance = [abs(self.robotCordY - goal[1]) for goal in self.robot.goalPointsonMap]
            d = np.hypot(xDistance, yDistance)
            index = np.argmin(d)
            self.old_nearest_point_index = index
        else:
            index = self.old_nearest_point_index
            distanceToIndex = math.hypot(self.robot.goalPointsonMap[index][0], self.robot.goalPointsonMap[index][1])
            while True:
                distanceNextIndex = math.hypot(self.robot.goalPointsonMap[index+1][0], self.robot.goalPointsonMap[index+1][1])
                if distanceToIndex < distanceNextIndex:
                    break
                index = index+1 if (index +1) < len(self.robot.goalPointsonMap) else index
                distanceToIndex = distanceNextIndex
            self.old_nearest_point_index = index

        Lf = k * self.robot.linear_vel_x + self.Lfc

        while Lf > math.hypot(self.robot.goalPointsonMap[index][0], self.robot.goalPointsonMap[index][1]):
            if(index + 1) >= len(self.robot.goalPointsonMap):
                break
            index += 1
        return index, Lf

    def ProportionalControl(self, target, current):
        a = Kp * (target - current)
        return a

    def RobotMove(self, a, delta):
        self.robot.linear_vel_x = a
        self.robot.angular_z = self.robot.linear_vel_x / self.Lfc * math.tan(delta)
        self.msg.linear.x = self.robot.linear_vel_x
        self.msg.angular.z = self.robot.angular_z
        time.sleep(50)

    def Algorythm(self, pIndex):
        self.CountCord()
        index, Lf = self.FindCurrentWaypoint()
        trajectory = []
        if pIndex >= index:
            index = pIndex

        if index < len(self.robot.goalPointsonMap):
            trajectory = self.robot.goalPointsonMap[index]
        else:  # toward goal
            trajectory = self.robot.goalPointsonMap[-1]
            index = len(self.robot.goalPointsonMap) - 1

        alpha = math.atan2(trajectory[1] - self.robotCordY, trajectory[0] - self.robotCordX) - self.robot.angular_z ## YAW
        delta = math.atan2(2.0 * self.Lfc * math.sin(alpha) / Lf, 1.0)
        return delta, index

def main():
    dis_robot = Robot()
    rospy.init_node('robot_position', anonymous=True)

    rospy.wait_for_service('static_map')
    mapsrv = rospy.ServiceProxy('static_map', GetMap)
    result = mapsrv()
    loaded_map = Map(result.map.info.resolution, result.map.info.height, result.map.info.width, result.map.info.origin, result.map.data)

    algorythm = PurePursuit(dis_robot, loaded_map)
    targetIndex, _ = algorythm.FindCurrentWaypoint()
    targetSpeed = 10.0 / 8
    lastIndex = len(algorythm.robot.goalPointsonMap) - 1

    while lastIndex > targetIndex:
        ai = algorythm.ProportionalControl(targetSpeed, algorythm.robot.linear_vel_x)
        di, targetIndex = algorythm.Algorythm(targetIndex)

        algorythm.RobotMove(ai, di)
        algorythm.pub.publish(algorythm.msg)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

        
    # def CalculateNewPoint(self, foundGoal):
    #     goal =[]
    #     cal = (foundGoal[0][1]-foundGoal[1][1])/(foundGoal[0][0]-foundGoal[1][0])
    #     # steps = abs(foundGoal[1][0] - foundGoal[0][0]) * 10
    #     accuracy = 0.001
    #     for x in np.arange(foundGoal[0][0], foundGoal[1][0], accuracy):
    #        if(round(self.L2-x**2, 2) == round(cal**2*(x**2-foundGoal[0][0]**2)+foundGoal[0][1]**2-2*cal*foundGoal[0][0], 2)):
    #             goal.append(x)
    #             goal.append(math.sqrt(self.L2-x**2))
    #     return goal

    # def CalculateR(self, index):
    #     # xDistance = abs(self.robot.position.x - goal[0])
    #     yDistance = abs(self.robot.position.y - self.robot.goalPointsonMap[index][1])*self.loaded_map.resolution
    #     self.r = self.L2/(2*yDistance)
        

if __name__ == '__main__':
    main()