#! /usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from nav_msgs.srv import GetMap

class Map:
    def __init__(self, resolution, height, width, data):
        self.resolution = np.float32(resolution)
        self.height = np.uint32(height)
        self.width = np.uint32(width)
        self.data = np.array(data, dtype=np.int8)
        self.grid = [[-1 for i in range(0, self.height)] for j in range(0, self.width)]

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
        f.write("Grid Map")
        for y in range(self.height):
            for x in range(self.width):
                # print(str(self.grid[x][y]), end=' ')
                f.write("%s " % str(self.grid[x][y]))
            # print()
            f.write("\n")
        f.close()
        
    def OutPoints(self):
        isWallx, isWally = 0, 0
        x, y = 0, 0
        outPoints = []
        wallX, wallY = [], []
        for y in range(self.width):
            if(self.grid[x][y]):
                isWally+=1
                wallY.append(y)
            else:
                isWally = 0
            if(isWally>=3 and not self.grid[x][y+1]):
                    XoutPoint = y
                    isWally = 0
            for x in range(self.height):
                if(self.grid[x][y]):
                    isWallx +=1
                    wallX.append(x)
                else:
                    isWallx = 0
                if((isWallx>=3 and not self.grid[x+1][y])):
                    min(wallX)
                    isWallx = 0
                if(isWally == 1 and isWally == 1): outPoints.append([x, y])
            
if __name__ == '__main__':
    # rospy.init_node('map_try', anonymous=True)
    rospy.wait_for_service('static_map')
    try:
        mapsrv = rospy.ServiceProxy('static_map', GetMap)
        result = mapsrv()
        loaded_map = Map(result.map.info.resolution, result.map.info.height, result.map.info.width, result.map.data)
        loaded_map.ReadMap()
        loaded_map.PrintMap()
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

