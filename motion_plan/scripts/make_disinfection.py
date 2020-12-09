#! /usr/bin/env python

import rospy
from ..src.motion_plan.robot import Robot
from ..src.motion_plan.mapy import Map
from ..src.motion_plan.disinfection import Disinfection
from ..src.motion_plan.move import ChoseMove
from geometry_msgs.msg import Twist
from nav_msgs.srv import GetMap

def main():
    try:
        rospy.wait_for_service('static_map')
        mapsrv = rospy.ServiceProxy('static_map', GetMap)
        result = mapsrv()
        dis_process = Disinfection(result)

        msg = Twist()
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        chose = 'c'
        ChoseMove(chose, msg, pub)
        
        dis_process.Process()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()