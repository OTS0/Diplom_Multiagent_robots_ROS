#!/usr/bin/env python3  
import math
import time
from nav_msgs.msg import Odometry
import rospy  
from geometry_msgs.msg import Twist 
from geometry_msgs.msg import Point 
import tf  
import numpy 

import tf.transformations  
  
  
class SimpleController():  
  
    def __init__(self):  
        rospy.init_node('robot2_controller', anonymous=True)  
        rospy.on_shutdown(self.shutdown)  
        rospy.loginfo("Node started")  
        self.x=0  
        self.y=0
        self.z=0
        self.target_x = 0  
        self.target_y = 0
        self.target_z = 0    
        self.target_index = 0  
        self.pitch = 0
        self.yaw = 0
        self.roll = 0
        self.quaternion = [0.0,0.0,0.0,0.0]
        self.cmd_vel_pub = rospy.Publisher('robot2/cmd_vel', Twist, queue_size=1)  
        self.subscriber =rospy.Subscriber('robot2/odom', Odometry, self.pose_callback)
        self.subscriber_target =rospy.Subscriber('robot2/goal/point', Point, self.target_callback)
        while (self.target_z!=0.115 and self.target_z!=9.7):
            time.sleep(0.5)
            rospy.loginfo("wait")
        self.rate = rospy.Rate(20)  
        # rospy.loginfo("Coordinates: {0} {1}".format(self.target_x , self.target_y))  
        # rospy.loginfo("Coordinates: {0} {1}".format(self.x, self.y))  


    def pose_callback(self, msg):  
        # rospy.loginfo("cur pose: {0} {1}".format(msg.x, msg.y))  
        self.quaternion = [
            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        ]
        self.update_control(msg.pose.pose.position.x, msg.pose.pose.position.y, self.quaternion)  
  
    def update_control(self, x, y, quaternion):  
        self.x, self.y = x, y
        (self.roll,self.pitch,self.yaw) = tf.transformations.euler_from_quaternion(quaternion)
        rospy.loginfo("yaw in radian: {0}".format(self.yaw))
        rospy.loginfo("yaw in radian: {0}".format(self.yaw))
        rospy.loginfo("selfes: {0} {1}".format(self.x, self.y))

    def target_callback(self, msg):  
        # rospy.loginfo("cur pose: {0} {1}".format(msg.x, msg.y))  
        self.update_target(msg.x, msg.y, msg.z)  
  
    def update_target(self, x, y, z):  
        self.target_x, self.target_y, self.target_z = x, y, z
        rospy.loginfo("selfes: {0} {1} {2}".format(self.target_x, self.target_y, self.target_z)) 
  
    def spin(self):
        twist_msg = Twist()   
        speed=0.0 
        # self.cmd_vel_pub.publish(twist_msg)        
        # rospy.loginfo("yaw: {0}".format(self.yaw)) 
        # rospy.loginfo("arctg: {0}".format(arctg))  
        self.rate.sleep()
        diff_x = self.target_x - self.x  
        diff_y = self.target_y - self.y
        arctg = math.atan2(diff_y, diff_x)-self.yaw
        rospy.loginfo("selfes after sleeping: {0} {1}".format(self.target_x, self.target_y))         
        rospy.loginfo("selfes after sleeping: {0} {1}".format(self.target_x, self.target_y)) 



        while (abs(arctg)>0.02):
            twist_msg.angular.z = arctg*0.5
            self.cmd_vel_pub.publish(twist_msg)
            rospy.loginfo("arctg: {0}".format(arctg))
            rospy.loginfo("arctg in degrees: {0}".format(arctg*180/math.pi))
            # rospy.loginfo("diff_xy: {0} {1}".format(diff_x, diff_y))
            # rospy.loginfo("yaw: {0}".format(self.yaw)) 
            # rospy.loginfo("arctg: {0}".format(arctg)) 
            arctg = math.atan2(diff_y, diff_x)-self.yaw
            self.rate.sleep()

        while (abs(diff_x) > 0.2 or abs(diff_y) > 0.2):
            arctg = math.atan2(diff_y, diff_x)-self.yaw
            diff_x = self.target_x - self.x  
            diff_y = self.target_y - self.y
            speed = math.sqrt(diff_x**2+diff_y**2)
            if speed > 8:
                speed = 8
            if speed < 2:
                speed=0.5*speed
            # rospy.loginfo("Coordinates: {0} {1}".format(self.x, self.y)) 
            # rospy.loginfo("Speeds: {0}".format(twist_msg.linear.x)) 
            twist_msg.angular.z = 0                  
            twist_msg.linear.x = speed*0.5  
            self.cmd_vel_pub.publish(twist_msg)  
            self.rate.sleep()       
            while (abs(arctg)>0.157):
                twist_msg.angular.z = arctg*0.5
                twist_msg.linear.x = speed*0.25
                self.cmd_vel_pub.publish(twist_msg)  
                # rospy.loginfo("yaw: {0}".format(self.yaw)) 
                rospy.loginfo("arctg: {0}".format(arctg))
                rospy.loginfo("arctg in degrees: {0}".format(arctg*180/math.pi))
                rospy.loginfo("arctg in degrees: {0} {1}".format(diff_x, diff_y)) 
                arctg = math.atan2(diff_y, diff_x)-self.yaw            
                self.rate.sleep()
            rospy.loginfo("selfes in WHILE: {0} {1}".format(self.x, self.y))  
            #rospy.loginfo("Coordinates: {0} {1}".format(self.x, self.y))  
            #rospy.loginfo("Speeds: {0} {1}".format(twist_msg.linear.x, twist_msg.linear.y))

        rospy.loginfo("Got goal")
        twist_msg.linear.x = 0  
        twist_msg.angular.z = 0 
        self.cmd_vel_pub.publish(twist_msg)
        self.rate.sleep()  
  
  
    def shutdown(self):  
        self.cmd_vel_pub.publish(Twist())  
        rospy.sleep(1)   
  
simple_mover = SimpleController()              
simple_mover.spin()