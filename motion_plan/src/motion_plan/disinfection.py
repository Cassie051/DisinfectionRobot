#! /usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from nav_msgs.srv import GetMap
from geometry_msgs.msg import Point, Quaternion, Pose
from motion_plan.mapy import Map
from motion_plan.robot import Robot
import motion_plan.purepursuit
from copy import deepcopy
import math
import time
import threading

class Disinfection:
    def __init__(self, mapresult):
        self.loaded_map = Map(mapresult.map.info.resolution, mapresult.map.info.height, mapresult.map.info.width, mapresult.map.info.origin, mapresult.map.data)
        self.dis_map = deepcopy(self.loaded_map)
        self.loaded_map.ReadMap()
        self.loaded_map.FindWalls()
        self.dis_robot = Robot()
        self.dis_start_time = time.time()
        rospy.init_node('disinfection')
        self.map_pub = rospy.Publisher('/dis_map', OccupancyGrid)
        self.CalculateWayPoints()       # to update way points for pure pursuit in frirst init

    def __del__(self):
        del self.dis_map
        del self.dis_robot
        del self.loaded_map

    def UpdateRobotPosition(self):
        xcord = (self.dis_robot.position.x - self.loaded_map.origin.position.x)/self.loaded_map.resolution
        ycord = (self.dis_robot.position.y - self.loaded_map.origin.position.y)/self.loaded_map.resolution
        # self.dis_map.grid[int(xcord)][int(ycord)] = "R"
        self.dis_robot.onMapPosition[0] = int(xcord)
        self.dis_robot.onMapPosition[1] = int(ycord)
    
    
    def CalculateDis(self):
        E = 15373.44 * 100
        # E = 30746.88
        self.UpdateRobotPosition()
        self.dis_map.SaveMap()
        self.dis_map.PublishMap(self.map_pub)
        for wallCord in self.loaded_map.walls:
            obstycle = False
            line = self.AlgorithmBres(wallCord, self.dis_robot.onMapPosition) 
            xobs = 0
            yobs = 0
            for i in range(0, len(line)-2):
                mapPoint = self.loaded_map.grid[line[i][0]][line[i][1]]
                if(mapPoint == 1):
                    obstycle = True
                    xobs=line[i][0]
                    yobs = line[i][1]
                if(obstycle):
                    if(line[i][0] == wallCord[0] and line[i][1] == wallCord[1] and (line[i][0] == xobs or line[i][1]== yobs)):
                        obstycle = False
                        break
                    elif (not(line[i][0] == xobs or line[i][1]== yobs)):
                        break
                
            if(obstycle == False):
                x = (self.dis_robot.onMapPosition[0] - wallCord[0])*self.dis_map.resolution
                y = (self.dis_robot.onMapPosition[1] - wallCord[1])*self.dis_map.resolution
                r = math.sqrt(x**2 + y**2)*100
                dis_pass_time = time.time() - self.dis_start_time   # SARS killing dose 10-20 COVID sure killing dose 1000 - 3000 mJ/cm2  | uses 30 mJ/cm2
                dose = E*dis_pass_time/r                            # lamp 1.7W/cm lenght = 90 cm weight = 12.56 cm and 8 lamps-> 15373.44 W/cm2
                self.dis_map.grid[wallCord[0]][wallCord[1]] += dose

    def Process(self):
        i = 0
        while(True):
            th = threading.Thread(target = self.CalculateDis)
            th.start()
            time.sleep(1)
            i += 1
            if(i == 20):
                self.dis_map.PrintMap()
                i = 0

    def AlgorithmBres(self, wallCord, point):
        line = []
        x0, y0 = point[0], point[1]
        x1, y1 = wallCord[0], wallCord[1]
        dx = abs(wallCord[0] - point[0])
        sx = 1 if point[0] < wallCord[0] else -1
        dy = -1*abs(wallCord[1] - point[1])
        sy = 1 if point[1] < wallCord[1] else -1
        err = dx+dy
        toEndCOunter = 101
        while(toEndCOunter>0):
            line.append([x0, y0])
            if(x0 == x1 and y0 == y1):
                toEndCOunter=40
            if(not toEndCOunter <100):
                toEndCOunter =101
            toEndCOunter -= 1
            e2 = 2*err
            if(e2 >= dy):
                err += dy
                x0 += sx
            if(e2 <= dx):
                err += dx
                y0 += sy
        return line

 
    def CalculateWayPoints(self):
        check = 0
        point = [323, 80]
        for wallCord in self.loaded_map.walls:
            if(check % 5 == 0):
                line = self.AlgorithmBres(wallCord, point) 
                for i in range(0, len(line)-2):
                    if(line[i][0] == wallCord[0] and line[i][1] == wallCord[1]):
                        if(line[i-50][0] !=  self.dis_robot.onMapPosition[0] and line[i-50][1] !=  self.dis_robot.onMapPosition[1]):
                            self.dis_robot.goalPointsonMap.append([line[i-50][0], line[i-50][1]])
            check += 1

    def DoPurepursuite(self):
        algorythm = motion_plan.purepursuit.PurePursuit(self.dis_robot, self.loaded_map)
        targetIndex, _ = algorythm.FindCurrentWaypoint()

        while True:
            di, targetIndex = algorythm.Algorythm(targetIndex)

            algorythm.RobotMove(di)
            algorythm.pub.publish(algorythm.msg)
            algorythm.MoveTime = time.time()

            # d = math.hypot(abs(algorythm.robot.goalPointsonMap[targetIndex][0]-algorythm.robotCordX)/self.loaded_map.resolution, abs(algorythm.robot.goalPointsonMap[targetIndex][1]-algorythm.robotCordY)/self.loaded_map.resolution)
            # while(d > 1.6):
            #     algorythm.CountCord()
            #     d = math.hypot(abs(algorythm.robot.goalPointsonMap[targetIndex][0]-algorythm.robotCordX)/self.loaded_map.resolution, abs(algorythm.robot.goalPointsonMap[targetIndex][1]-algorythm.robotCordY)/self.loaded_map.resolution)
    

if __name__ == '__main__':
    try:
        rospy.wait_for_service('static_map')
        mapsrv = rospy.ServiceProxy('static_map', GetMap)
        result = mapsrv()

        dis_process = Disinfection(result)
        # dis_process.DoPurepursuite()
        dis_process.Process()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
