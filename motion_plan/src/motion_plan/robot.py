#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry

class Robot:
    def __init__(self):
        self.orientation = 0
        self.linear_vel_x = 0
        self.angular_vel_z = 0
        self.onMapPosition = [0, 0]
        rospy.Subscriber('/odom', Odometry, self.UpdatePosition)
        self.goalPointsonMap = [[284, 33], [284, 80,], [310, 80],[360, 60], [310, 20], [270, 60], [350, 30]]
        # self.goalPointsonMap = [[804, 119], [710, 87], [473, 103]]
        # self.goalPointsonMap = []

    def __del__(self):
        del self.orientation
        del self.linear_vel_x
        del self.angular_vel_z 
        del self.onMapPosition
        del self.goalPointsonMap

    def PrinPosition(self):
        print("Position XYZ: %f, %f, %f" %(self.position.x, self.position.y, self.position.y), end ="  ")
        print("Oreint RPY: %f, %f, %f" %(self.orientation[0], self.orientation[1], self.orientation[2]))

    def UpdatePosition(self, msg):
        self.position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        self.orientation = euler_from_quaternion(orientation_list)
        # self.PrinPosition()


if __name__ == '__main__':
    dis_robot = Robot()
    rospy.init_node('robot_position', anonymous=True)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass