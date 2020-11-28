#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import Odometry


class Robot:
    def __init__(self):
        self.position = 0
        self.orientation = 0
        rospy.Subscriber('/odom', Odometry, self.UpdatePosition)

    def PrinPosition(self):
        print("Position XYZ: %f, %f, %f" %(self.position.x, self.position.y, self.position.y), end ="  ")
        print("Oreint XYZw: %f, %f, %f, %f" %(self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w))

    def UpdatePosition(self, msg):
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
        self.PrinPosition()



if __name__ == '__main__':
    dis_robot = Robot()
    rospy.init_node('robot_position', anonymous=True)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass