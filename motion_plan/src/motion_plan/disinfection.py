#! /usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from nav_msgs.srv import GetMap
from geometry_msgs.msg import Point, Quaternion, Pose
from mapy import Map
from robot import Robot
from copy import deepcopy
import math

class Disinfection:
    def __init__(self, mapresult):
        self.loaded_map = Map(mapresult.map.info.resolution, mapresult.map.info.height, mapresult.map.info.width, mapresult.map.info.origin, mapresult.map.data)
        self.dis_map = deepcopy(self.loaded_map)
        self.loaded_map.ReadMap()
        self.loaded_map.FindWalls()
        self.dis_robot = Robot()
        rospy.init_node('robot_position', anonymous=True)

    def __del__(self):
        del self.dis_map
        del self.dis_robot
        del self.loaded_map

    def UpdateRobotPosition(self):
        xcord = (self.dis_robot.position.x - self.loaded_map.origin.position.x)/self.loaded_map.resolution
        ycord = (self.dis_robot.position.y - self.loaded_map.origin.position.y)/self.loaded_map.resolution
        self.dis_map.grid[int(xcord)][int(ycord)] = "R"
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
        while(True):
            line.append([x0, y0])
            if(x0 == x1 and y0 == y1):
                break
            e2 = 2*err
            if(e2 >= dy):
                err += dy
                x0 += sx
            if(e2 <= dx):
                err += dx
                y0 += sy
        return line

    def Process(self):
        self.UpdateRobotPosition()
        E = 4.88
        time = 1
        for wallCord in self.loaded_map.walls:
            x = abs(self.dis_robot.onMapPosition[0] - wallCord[0])*self.dis_map.resolution
            y = abs(self.dis_robot.onMapPosition[1] - wallCord[1])*self.dis_map.resolution
            r = math.sqrt(x**2 + y**2)
            dose = E*time/r # *x/r
            line = self.AlgorithmBres(wallCord)
            for point in line:
                mapPoint = self.loaded_map.grid[point[0]][point[1]]
                if(mapPoint == 1):
                    self.dis_map.grid[point[0]][point[1]] += dose
                    break
        self.dis_map.PrintMap()

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
