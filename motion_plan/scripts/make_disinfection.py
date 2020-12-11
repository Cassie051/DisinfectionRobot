#! /usr/bin/env python

import rospy
import motion_plan.disinfection, motion_plan.move
from geometry_msgs.msg import Twist
from nav_msgs.srv import GetMap

def main():
    try:
        rospy.wait_for_service('static_map')
        mapsrv = rospy.ServiceProxy('static_map', GetMap)
        result = mapsrv()
        dis_process = motion_plan.disinfection.Disinfection(result)

        msg = Twist()
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        chose = 'a'
        motion_plan.move.ChoseMove(chose, msg, pub)
        
        dis_process.Process()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()