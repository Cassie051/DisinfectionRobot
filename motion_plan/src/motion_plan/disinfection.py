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
    def __init__(self):
        rospy.wait_for_service('static_map')
        mapsrv = rospy.ServiceProxy('static_map', GetMap)
        result = mapsrv()
        self.loaded_map = Map(result.map.info.resolution, result.map.info.height, result.map.info.width, result.map.info.origin, result.map.data)
        self.loaded_map.ReadMap()
        self.loaded_map.FindWalls()
        self.dis_map = deepcopy(self.loaded_map)
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
        self.dis_robot.onMapPosition.append([int(xcord), int(ycord)])
    
    def Process(self):
        self.UpdateRobotPosition()
        E = 4.88
        time = 1
        for wallCord in self.loaded_map.walls:
            obstacle = False
            A = self.dis_robot.onMapPosition[0] - wallCord[0]     # line parametrs Ay + Bx + C = 0
            B = wallCord[1] - self.dis_robot.onMapPosition[1]
            C = (self.dis_robot.onMapPosition[1] - wallCord[1])*self.dis_robot.onMapPosition[0] + (wallCord[0] - self.dis_robot.onMapPosition[0])*self.dis_robot.onMapPosition[1]
            iterPoint = [0, 0]
            if(B != 0):
                if(wallCord[0] < self.dis_robot.onMapPosition[0]):
                    iterPoint[0] = wallCord[0]+1
                    while(iterPoint[0] < self.dis_robot.onMapPosition[0]):
                        iterPoint[1] = int((B*iterPoint[0]+C)/A)
                        if(self.loaded_map.grid[iterPoint[0]][iterPoint[1]] != 0):
                            obstacle = True
                            break
                        else:
                            iterPoint[0] += 1
                            
                if(wallCord[0] > self.dis_robot.onMapPosition[0]):
                    iterPoint[0] = wallCord[0] -1
                    while(iterPoint[0] > self.dis_robot.onMapPosition[0]):
                        iterPoint[1] = int((B*iterPoint[0]+C)/A)
                        if(self.loaded_map.grid[iterPoint[0]][iterPoint[1]] != 0):
                            obstacle = True
                            break
                        else:
                            iterPoint[0] -= 1
            else:
                iterPoint[0] = wallCord[0]
                if(wallCord[1] < self.dis_robot.onMapPosition[1]):
                    iterPoint[1] = wallCord[1]
                    while(iterPoint[1] < self.dis_robot.onMapPosition[1]):
                        if(self.loaded_map.grid[iterPoint[0]][iterPoint[1]] == 1):
                            obstacle = True
                            break
                        else:
                            iterPoint[1] += 1
                if(wallCord[1] > self.dis_robot.onMapPosition[1]):
                    iterPoint[1] = wallCord[1]
                    while(iterPoint[1] > self.dis_robot.onMapPosition[1]):
                        if(self.loaded_map.grid[iterPoint[0]][iterPoint[1]] == 1):
                            obstacle = True
                            break
                        else:
                            iterPoint[1] -= 1
            if(A != 0):
                if(wallCord[1] < self.dis_robot.onMapPosition[1]):
                    iterPoint[1] = wallCord[1] + 1
                    while(iterPoint[1] < self.dis_robot.onMapPosition[1]):
                        iterPoint[0] = int((A*iterPoint[1]+C)/-B)
                        if(self.loaded_map.grid[iterPoint[0]][iterPoint[1]] != 0):
                            obstacle = True
                            break
                        else:
                            iterPoint[1] += 1
                if(wallCord[1] > self.dis_robot.onMapPosition[1]):
                    iterPoint[1] = wallCord[1] - 1
                    while(iterPoint[1] > self.dis_robot.onMapPosition[1]):
                        iterPoint[0] = int((A*iterPoint[1]+C)/-B)
                        if(self.loaded_map.grid[iterPoint[0]][iterPoint[1]] != 0):
                            obstacle = True
                            break
                        else:
                            iterPoint[1] -= 1
            else:
                iterPoint[1] = wallCord[1]
                if(wallCord[0] < self.dis_robot.onMapPosition[0]):
                    iterPoint[0] = wallCord[0]
                    while(iterPoint[0] < self.dis_robot.onMapPosition[0]):
                        if(self.loaded_map.grid[iterPoint[0]][iterPoint[1]] == 1):
                            obstacle = True
                            break
                        else:
                            iterPoint[0] += 1
                if(wallCord[0] > self.dis_robot.onMapPosition[0]):
                    iterPoint[0] = wallCord[0]
                    while(iterPoint[0] > self.dis_robot.onMapPosition[0]):
                        if(self.loaded_map.grid[iterPoint[0]][iterPoint[1]] == 1):
                            obstacle = True
                            break
                        else:
                            iterPoint[0] -= 1
            if(obstacle == False):
                x = abs(self.dis_robot.onMapPosition[0] - wallCord[0])*self.dis_map.resolution
                y = abs(self.dis_robot.onMapPosition[1] - wallCord[1])*self.dis_map.resolution
                r = math.sqrt(x**2 + y**2)
                dose = E*time/r # *x/r
                self.dis_map.grid[wallCord[0]][wallCord[1]] = dose
        self.dis_map.PrintMap()



if __name__ == '__main__':
    dis_process = Disinfection()
    dis_process.Process()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
