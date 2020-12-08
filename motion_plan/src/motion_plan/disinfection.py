#! /usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from nav_msgs.srv import GetMap
from geometry_msgs.msg import Point, Quaternion, Pose
from mapy import Map
from robot import Robot
from copy import deepcopy
import math
import time
import threading

class Disinfection:
    def __init__(self, mapresult):
        self.loaded_map = Map(mapresult.map.info.resolution, mapresult.map.info.height, mapresult.map.info.width, mapresult.map.info.origin, mapresult.map.data)
        self.dis_map = deepcopy(self.loaded_map)
        # rospy.init_node('disinfection_map', anonymous=True)
        self.loaded_map.ReadMap()
        self.loaded_map.FindWalls()
        self.dis_robot = Robot()
        self.dis_start_time = time.time()
        rospy.init_node('disinfection')
        self.map_pub = rospy.Publisher('/dis_map', OccupancyGrid)

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
    
    def AlgorithmBres(self, wallCord):
        line = []
        x0, y0 = self.dis_robot.onMapPosition[0], self.dis_robot.onMapPosition[1]
        x1, y1 = wallCord[0], wallCord[1]
        dx = abs(wallCord[0] - self.dis_robot.onMapPosition[0])
        sx = 1 if self.dis_robot.onMapPosition[0] < wallCord[0] else -1
        dy = -1*abs(wallCord[1] - self.dis_robot.onMapPosition[1])
        sy = 1 if self.dis_robot.onMapPosition[1] < wallCord[1] else -1
        err = dx+dy
        line.append([x0, y0])
        # 4
        if(sx == 1 and sy == -1 and err > 0):
            x1, y1 = wallCord[0]-1, wallCord[1]+1
        elif(sx == 1 and sy == -1 and err < 0):
            x1, y1 = wallCord[0]-1, wallCord[1]+1
        # 1
        elif(sx == 1 and sy == 1 and err > 0):
            x1, y1 = wallCord[0]-1, wallCord[1]-1
        elif(sy == 1 and sx ==1 and err < 0):
            x1, y1 = wallCord[0]-1, wallCord[1]-1
        # 2
        elif(sy == 1 and sx ==-1 and err < 0):
            x1, y1 = wallCord[0]+1, wallCord[1]-1
        elif(sx == -1 and sy == 1 and err > 0):
            x1, y1 = wallCord[0]+1, wallCord[1]-1
        # 3
        elif(sx == -1 and sy == -1 and err > 0):
            x1, y1 = wallCord[0]+1, wallCord[1]+1
        elif(sx == -1 and sy == -1 and err < 0):
            x1, y1 = wallCord[0]+1, wallCord[1]+1
        dx = abs(x1 - self.dis_robot.onMapPosition[0])
        dy = -1*abs(y1 - self.dis_robot.onMapPosition[1])
        err = dx+dy
        while(True):
            if(x0 == x1 and y0 == y1):
                break
            e2 = 2*err
            if(e2 >= dy):
                err += dy
                x0 += sx
            if(e2 <= dx):
                err += dx
                y0 += sy
            line.append([x0, y0])
        dx = abs(wallCord[0] - self.dis_robot.onMapPosition[0])
        dy = -1*abs(wallCord[1] - self.dis_robot.onMapPosition[1])
        err = dx+dy
        if(sx == 1 and sy == -1 and err > 0):
            x0, y0 = x0+1, y0-1
        elif(sx == 1 and sy == -1 and err < 0):
            x0, y0 = x0+1, y0-1
        # 1
        elif(sx == 1 and sy == 1 and err > 0):
            x0, y0 = x0+1, y0+1
        elif(sy == 1 and sx ==1 and err < 0):
            x0, y0 = x0+1, y0+1
        # 2
        elif(sy == 1 and sx ==-1 and err < 0):
            x0, y0 = x0-1, y0+1
        elif(sx == -1 and sy == 1 and err > 0):
            x0, y0 = x0-1, y0+1
        # 3
        elif(sx == -1 and sy == -1 and err > 0):
            x0, y0 = x0-1, y0-1
        elif(sx == -1 and sy == -1 and err < 0):
            x0, y0 = x0-1, y0-1
        line.append([x0, y0])
        return line
    
    def CalculateDis(self):
        # E = 15373.44
        E = 30746.88 *100
        self.UpdateRobotPosition()
        self.dis_map.SaveMap()
        self.dis_map.PublishMap(self.map_pub)
        for wallCord in self.loaded_map.walls:
            obstycle = False
            line = self.AlgorithmBres(wallCord)           
            for i in range(0, len(line)-2):
                if(self.loaded_map.grid[line[i][0]][line[i][1]] == 1):
                    obstycle = True
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
            time.sleep(2)
            i += 1
            if(i == 20):
                self.dis_map.PrintMap()
                i = 0

if __name__ == '__main__':
    rospy.wait_for_service('static_map')
    mapsrv = rospy.ServiceProxy('static_map', GetMap)
    result = mapsrv()
    dis_process = Disinfection(result)
    dis_process.Process()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
