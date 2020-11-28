#! /usr/bin/env python

import rospy
from mapy import Map
from robot import Robot
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from nav_msgs.srv import GetMap
from geometry_msgs.msg import Pose
from copy import deepcopy

class Disinfection:
    def __init__(self):
        rospy.wait_for_service('static_map')
        mapsrv = rospy.ServiceProxy('static_map', GetMap)
        result = mapsrv()
        self.dis_robot = Robot()
        rospy.init_node('robot_position', anonymous=True)
        self.loaded_map = Map(result.map.info.resolution, result.map.info.height, result.map.info.width, result.map.info.origin, result.map.data)
        self.loaded_map.ReadMap()
        self.dis_map = deepcopy(self.loaded_map)

    def UpdateRobotPosition(self):
        xcord = (self.dis_robot.position.x - self.loaded_map.origin.position.x)/self.loaded_map.resolution
        ycord = (self.dis_robot.position.y - self.loaded_map.origin.position.y)/self.loaded_map.resolution
        self.dis_map.grid[int(xcord)][int(ycord)] = "R"
        self.dis_map.PrintMap()

if __name__ == '__main__':
    try:
        dis_process = Disinfection()
        dis_process.UpdateRobotPosition()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
