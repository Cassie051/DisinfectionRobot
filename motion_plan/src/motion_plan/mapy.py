#! /usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from nav_msgs.srv import GetMap

class Map:
    def __init__(self, resolution, height, width, origin, data):
        self.resolution = np.float32(resolution)
        self.height = np.uint32(height)
        self.width = np.uint32(width)
        self.origin = origin
        self.data = np.array(data, dtype=np.int8)
        self.grid = [[-1 for i in range(0, self.height)] for j in range(0, self.width)]
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
        # print("GridMap")
        f.write("Grid Map\n")
        for y in range(self.height):
            for x in range(self.width):
                # print(str(self.grid[x][y]), end=' ')
                if(self.grid[x][y] != "R"):
                    tmp = float(self.grid[x][y])
                    gridFloat = round(tmp, 1)
                    f.write("%s " % str(gridFloat))
                else:
                    f.write("%s " % str(self.grid[x][y]))
            # print()
            f.write("\n")
        f.close()
        
    def FindWalls(self):
        for y in range(self.height):
            for x in range(self.width):
                if(self.grid[x][y] == 1):
                    self.walls.append([x, y])

            
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

