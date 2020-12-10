#! /usr/bin/env python

import rospy
import numpy as np
import sys
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from nav_msgs.srv import GetMap

class Map:
    def __init__(self, resolution, height, width, origin, data):
        self.resolution = np.float32(resolution)
        self.height = np.uint32(height)
        self.width = np.uint32(width)
        self.origin = origin
        self.data = np.array(data, dtype=np.int8)
        self.grid = [[0 for i in range(0, self.height)] for j in range(0, self.width)]
        self.walls = []


    def __del__(self):
        del self.resolution
        del self.height
        del self.width
        del self.origin
        del self.data
        del self.grid
        del self.walls

    def ReadMap(self):
        self.grid = [[-1 for i in range(0, self.height)] for j in range(0, self.width)]
        currentCell = 0
        for y in range(self.height):
            for x in range(self.width):
                if(self.data[currentCell] == -1 or self.data[currentCell] == 0):
                    self.grid[x][y] = 0
                # elif (self.data[currentCell] ):
                #     self.grid[x][y] = 0
                elif(self.data[currentCell] == 100):
                    self.grid[x][y] = 1
                currentCell+=1
        return self.grid

    def PrintMap(self):
        f=open("room.txt", "w")
        f.write("Grid Map\n")
        for y in range(self.height):
            for x in range(self.width):
                if(self.grid[x][y] != "R"):
                    tmp = float(self.grid[x][y])
                    gridFloat = round(tmp, 1)
                    f.write("%s " % str(gridFloat))
                else:
                    f.write("%s " % str(self.grid[x][y]))
            f.write("\n")
        f.close()
        
    def FindWalls(self):
        for y in range(self.height):
            for x in range(self.width):
                if(self.grid[x][y] == 1):
                    self.walls.append([x, y])
    
    def max_value(self):
        max_val = -1
        for y in range(self.height):
                for x in range(self.width):
                    if(self.grid[x][y] > max_val and self.grid[x][y] != 0):
                        max_val = self.grid[x][y]
        return max_val

    def min_value(self):
        min_val = sys.maxsize
        for y in range(self.height):
                for x in range(self.width):
                    if(self.grid[x][y] < min_val and self.grid[x][y] != 0):
                        min_val = self.grid[x][y]
        return min_val

    def SaveMap(self):
        max_grid = 30000000
        min_grid = 2
        data_range = 99/(max_grid  - min_grid)
        currentCell = 0
        while(currentCell < self.width * self.height):
            for y in range(self.height):
                for x in range(self.width):
                    if(self.grid[x][y] == 0):
                        self.data[currentCell] = 0
                    else:
                        if(int(self.grid[x][y]) >= max_grid):
                            self.data[currentCell] = 99
                        else:
                            self.data[currentCell] = int(self.grid[x][y]*data_range)
                    currentCell+=1

    def PublishMap(self, map_pub):
        make_map = OccupancyGrid()
        make_map.info.resolution = self.resolution
        make_map.info.width = self.width
        make_map.info.height = self.height
        make_map.info.origin = self.origin
        make_map.data = self.data
        map_pub.publish(make_map)
            
if __name__ == '__main__':
    # rospy.init_node('map_try', anonymous=True)
    rospy.wait_for_service('static_map')
    try:
        mapsrv = rospy.ServiceProxy('static_map', GetMap)
        result = mapsrv()
        loaded_map = Map(result.map.info.resolution, result.map.info.height, result.map.info.width, result.map.info.origin, result.map.data)
        loaded_map.ReadMap()
        loaded_map.PrintMap()
        loaded_map.FindWalls()
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

