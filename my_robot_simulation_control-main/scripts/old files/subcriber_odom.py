#!/usr/bin/env python
import rospy

from nav_msgs.msg import Odometry

def odometryCb(msg):
    rospy.loginfo(msg.pose.pose)

if __name__ == "__main__":
    rospy.init_node('oodometry', anonymous=True) 
    rospy.Subscriber('odom', Odometry, odometryCb)
    rospy.spin()