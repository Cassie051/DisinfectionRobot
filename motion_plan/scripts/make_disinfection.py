#! /usr/bin/env python

import rospy
import motion_plan.disinfection, motion_plan.move
from geometry_msgs.msg import Twist
from nav_msgs.srv import GetMap
import threading

def main():
    try:
        rospy.wait_for_service('static_map')
        mapsrv = rospy.ServiceProxy('static_map', GetMap)
        result = mapsrv()
        dis_process = motion_plan.disinfection.Disinfection(result)

        msg = Twist()
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        chose = 'c'
        thd = threading.Thread(target = dis_process.Process)
        go = True
        tab = [go]
        thm = threading.Thread(target = motion_plan.move.ChoseMove, args=(chose, msg, pub, tab, dis_process))
        
        thd.start()
        thm.start()

        thd.join()
        tab[0] = False
        motion_plan.move.Stop(msg)
        pub.publish(msg)

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()