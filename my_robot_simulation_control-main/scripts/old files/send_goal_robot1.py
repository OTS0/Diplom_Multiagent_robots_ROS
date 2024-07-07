#!/usr/bin/env python3  
from geometry_msgs.msg import Point 
import rospy  


if __name__ == '__main__':
    x=-26
    y=40
    z=0.115
    point_msg = Point()  
    rospy.init_node("send_goal_to_robot1")  
    rospy.loginfo("send_goal node started...")  
    rate = rospy.Rate(2) 
    point_msg.x=x
    point_msg.y=y
    point_msg.z=z
    publisher = rospy.Publisher("/robot1/goal/point", Point, queue_size=10)   

    while not rospy.is_shutdown():  
        publisher.publish(point_msg)  
        rospy.loginfo("ppublish oint xyz: {0} {1} {2}".format(point_msg.x, point_msg.y, point_msg.z)) 
        rate.sleep()


