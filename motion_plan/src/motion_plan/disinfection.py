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

        self.testMap = deepcopy(self.loaded_map) # ONLY FOR
        self.loaded_map.ReadMap()
        self.loaded_map.FindWalls()
        self.dis_robot = Robot()
        self.dis_start_time = time.time()
        rospy.init_node('disinfection')
        self.map_pub = rospy.Publisher('/dis_map', OccupancyGrid, queue_size=1)
        # self.CalculateWayPoints()       # to update way points for pure pursuit in frirst init
        self.ShowWayPointsOnMap([-1, -1])


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
        E = 15373.44 * 10
        # E = 30746.88
        self.UpdateRobotPosition()
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
        self.dis_map.SaveMap()
        self.dis_map.PublishMap(self.map_pub)

    def Process(self):
        b30, b60, b90, b100 = False, False, False, False
        while( not b100):
            th = threading.Thread(target = self.CalculateDis)
            th.start()
            time.sleep(1)

            diswall = 0    
            for wallCord in self.loaded_map.walls:
                if(self.dis_map.grid[wallCord[0]][wallCord[1]] >= 30000000):
                    diswall +=1
            # 30 60 90 100
            precent = diswall/len(self.loaded_map.walls) * 100
            if(precent >= 30 and not b30):
                passTime = time.time() - self.dis_start_time
                print("30 %% dezynfekcji, czas %s s" % str(passTime))
                b30 = True
            elif(precent >= 60 and not b60):
                passTime = time.time() - self.dis_start_time
                print("60 %% dezynfekcji, czas %s s" % str(passTime))
                b60 = True
            elif(precent >= 90 and not b90):
                passTime = time.time() - self.dis_start_time
                print("90 %% dezynfekcji, czas %s s" % str(passTime))
                b90 = True
            elif(precent >= 100 and not b100):
                passTime = time.time() - self.dis_start_time
                print("100 %% dezynfekcji, czas %s s" % str(passTime))
                b100 = True
            # else:
            #     passTime = time.time() - self.dis_start_time
            #     if(int(passTime)%20 == 0):
            #         print("Czas dezynfekcji %s s" % str(passTime))
        return True
            

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
            # if(wallCord[0] % 10 == 0 or wallCord[1] % 10 ==0):
                line = self.AlgorithmBres(wallCord, point) 
                for i in range(0, len(line)-2):
                    if(line[i][0] == wallCord[0] and line[i][1] == wallCord[1]):
                        if(line[i-27][0] !=  self.dis_robot.onMapPosition[0] and line[i-27][1] !=  self.dis_robot.onMapPosition[1]):
                            self.dis_robot.goalPointsonMap.append([line[i-27][0], line[i-27][1]])
            check += 1
        

    def ShowWayPointsOnMap(self, point):
        i = 0
        for goal in self.dis_robot.goalPointsonMap:
            if(goal[0] == point[0] and goal[1] == point[1]):
                self.testMap.grid[goal[0]][goal[1]] = 5000000
            else:
                self.testMap.grid[goal[0]][goal[1]] = 3000435453
            i += 1
        self.testMap.SaveMap()
        self.testMap.PublishMap(self.map_pub)

    def DoPurepursuite(self):
        algorythm = motion_plan.purepursuit.PurePursuit(self.dis_robot, self.loaded_map)
        self.UpdateRobotPosition()
        algorythm.FindCurrentWaypoint()
        self.ShowWayPointsOnMap(algorythm.nearest_point)
        i = 0
        while True:
            i += 1
            self.UpdateRobotPosition()
            di = algorythm.Algorythm()
            if(i > 1000):
                thmap = threading.Thread(target = self.ShowWayPointsOnMap, args=[algorythm.nearest_point])
                thmap.start()
                thmap.join()
                i=0
            algorythm.RobotMove(di)
            algorythm.MoveTime = time.time()
            algorythm.pub.publish(algorythm.msg)
            oldtarget = algorythm.nearest_point


if __name__ == '__main__':
    try:
        rospy.wait_for_service('static_map')
        mapsrv = rospy.ServiceProxy('static_map', GetMap)
        result = mapsrv()

        dis_process = Disinfection(result)
        dis_process.DoPurepursuite()

    except rospy.ROSInterruptException:
        pass
