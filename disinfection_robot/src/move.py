#! /usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def move(speed, distance):
    rospy.init_node("speed_controller")
    velocity_publisher = rospy.Publisher("dis_robot/cmd_vel", Twist, queue_size = 1)

    vel_msg = Twist()

    if(speed>0):
        vel_msg.linear.x = abs(speed)
    else:
        vel_msg.linear.x = -abs(speed)

    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    while not rospy.is_shutdown():

        t0 = rospy.Time.now().to_sec()
        current_distance = 0

        while(current_distance < distance):
 
            velocity_publisher.publish(vel_msg)
            t1=rospy.Time.now().to_sec()
            current_distance= speed*(t1-t0)

        vel_msg.linear.x = 0

        velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        #Testing our function
        move(2, 10)
    except rospy.ROSInterruptException: pass