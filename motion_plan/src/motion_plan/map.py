#! /usr/bin/env python

import rospy
import numpy as np
from nav_msgs import *

class Map():
    def __init__(self, resolution, width, height, data):
        self.resolution = resolution
        self.height = height
        self.width = width
        self.data = data
        self.grid = np.array([]) 

    def ReadMap(self):
        currentCell = 0
        for y in range(self.width):
            for x in range(self.height):
                if(self.data[currentCell] != 0):
                    self.grid[x][y] = False 
                else:
                    self.grid[x][y] = True
                currentCell+=1
        return self.grid
        
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
            


                      
