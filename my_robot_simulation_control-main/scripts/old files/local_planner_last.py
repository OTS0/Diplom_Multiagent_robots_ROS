#!/usr/bin/env python3  
import math
import time
from nav_msgs.msg import Odometry
import rospy  
from geometry_msgs.msg import Twist 
from geometry_msgs.msg import Point 
import tf  
from sensor_msgs.msg import LaserScan
import tf.transformations  
import numpy as np
  
class SimpleController():  
  
    def __init__(self):  
        rospy.init_node('robot1_controller_local', anonymous=True)  
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
        self.lidar_range = [0]*720
        self.quaternion = [0.0,0.0,0.0,0.0]

        self.forward_right_free=True
        self.forward_left_free=True

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)  
        self.subscriber =rospy.Subscriber('/odom', Odometry, self.pose_callback)
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        self.subscriber_target =rospy.Subscriber('/goal/point', Point, self.target_callback)
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
        # rospy.loginfo("yaw in radian: {0}".format(self.yaw))
        # rospy.loginfo("yaw in radian: {0}".format(self.yaw))
        # rospy.loginfo("selfes: {0} {1}".format(self.x, self.y))

    def target_callback(self, msg):  
        # rospy.loginfo("cur pose: {0} {1}".format(msg.x, msg.y))  
        self.update_target(msg.x, msg.y, msg.z)  
  
    def update_target(self, x, y, z):  
        self.target_x, self.target_y, self.target_z = x, y, z
        # rospy.loginfo("selfes: {0} {1} {2}".format(self.target_x, self.target_y, self.target_z)) 

    def lidar_callback(self, msg):  
        self.lidar_range = np.array(msg.ranges)

#-----------------scan and obsctacled--------------------

    def lidar_scan_forward(self):
        for i in range (256,464):
            if self.lidar_range[i]<1.2:
                return False
        return True
    
    def lidar_scan_back(self):
        back_left_free=True
        back_right_free=True
        for i in range (0,66):
            if self.lidar_range[i]<2:
                back_left_free=False
                break
        for i in range (657,719):
            if self.lidar_range[i]<2:
                back_right_free=False
                break
        if (back_right_free and back_left_free):
            rospy.loginfo("Back free")
            return True
        else: return False

    
    def lidar_scan_side(self, angle_1, angle_2):
        for i in range (angle_1,angle_2):
            if self.lidar_range[i]<1.5:
                return False
        return True

    def find_min_range_obs(self):
        min_range_obs=math.inf
        for i in range (260,460):
            if self.lidar_range[i]<min_range_obs:
                min_range_obs = self.lidar_range[i]
        return min_range_obs

    def lidar_check_forward(self):
        for i in range (260,360):
            if self.lidar_range[i]<1.5:
                self.forward_right_free = False
                rospy.loginfo("range and i {0} {1}".format(self.lidar_range[i], i))
                break
        for i in range (360,460):
            if self.lidar_range[i]<1.5:
                self.forward_left_free = False
                break

    def rotate_for_free_side(self,angular, twist_msg):
        while (self.lidar_scan_forward()==False):
            twist_msg.angular.z = angular
            self.cmd_vel_pub.publish(twist_msg)
            self.rate.sleep()
        twist_msg.angular.z = 0
        self.cmd_vel_pub.publish(twist_msg)
        self.rate.sleep()     

    def stuck_wait(self,twist_msg):
        rospy.loginfo("Все пути заблокированы")
        while (self.lidar_scan_forward()==False):
            twist_msg.angular.z = 0.25
            self.cmd_vel_pub.publish(twist_msg)
            self.rate.sleep()
 
    def move_around_obstacles(self, twist_msg):
        self.lidar_check_forward()
# если препятсиве только впереди слева 

        if (self.forward_right_free==True and self.forward_left_free==False and self.lidar_scan_side(180,270)):
            self.rotate_for_free_side(-0.5,twist_msg)
            rospy.loginfo("Спереди справа свободно")

        elif (self.forward_right_free==False and self.forward_left_free==True and self.lidar_scan_side(450,540)):
            self.rotate_for_free_side(0.5,twist_msg)
            rospy.loginfo("Сперели слева свободно")

        elif (self.lidar_scan_side(120,210)):
            self.rotate_for_free_side(-0.5,twist_msg)
            rospy.loginfo("Справа свободно")

        elif (self.lidar_scan_side(500,620)):
            self.rotate_for_free_side(0.5,twist_msg)
            rospy.loginfo("Слева свободно")

        elif (self.lidar_scan_back()):
            twist_msg.linear.x=-0.3
            self.cmd_vel_pub.publish(twist_msg)  
            self.rate.sleep()
            rospy.loginfo("Двигаюсь назад")


        else:
            self.stuck_wait(twist_msg)


    def work_with_obs(self,twist_msg):
        while (self.lidar_scan_forward()==False):
            if twist_msg.linear.x!=0:
                twist_msg.linear.x = 0.05*self.find_min_range_obs() 
                self.cmd_vel_pub.publish(twist_msg)  
                self.rate.sleep()
            # rospy.loginfo("twist_msg.linear.x: {0}".format(twist_msg.linear.x))
            self.move_around_obstacles(twist_msg)



    def spin(self):
        twist_msg = Twist()   
        speed=0.0 
        # self.cmd_vel_pub.publish(twist_msg)        

        self.rate.sleep()
        diff_x = self.target_x - self.x  
        diff_y = self.target_y - self.y
        arctg = math.atan2(diff_y, diff_x)-self.yaw

        distance = math.sqrt(diff_x**2+diff_y**2)
        rospy.loginfo("distance {0}".format(distance))
        while (distance>1):
            diff_x = self.target_x - self.x  
            diff_y = self.target_y - self.y
            distance = math.sqrt(diff_x**2+diff_y**2)
            arctg = math.atan2(diff_y, diff_x)-self.yaw

            rospy.loginfo("distance in while {0}".format(distance))
            # rospy.loginfo("Coordinates: {0} {1}".format(self.x, self.y)) 
            # rospy.loginfo("Speeds: {0}".format(twist_msg.linear.x))
            if (self.lidar_scan_forward()==False):
                self.work_with_obs(twist_msg)
                if (abs(distance)<1):
                    break

            distance = math.sqrt(diff_x**2+diff_y**2)
            speed = distance
            if speed > 6:
                speed = 6
            if speed < -6:
                speed = -6
            twist_msg.angular.z = 0                  
            twist_msg.linear.x = speed*0.5  
            self.cmd_vel_pub.publish(twist_msg)           

            while (abs(arctg)>0.1):
                rospy.loginfo("I got while")
                if (self.lidar_scan_forward()==False):
                    rospy.loginfo("I'm in while if")
                    self.work_with_obs(twist_msg)  
                if (distance<1):
                    break 
                if twist_msg.linear.x!=0:
                    twist_msg.linear.x = speed*0.1
                    self.cmd_vel_pub.publish(twist_msg) 
                    self.rate.sleep()

                while (self.lidar_scan_side(500,620)==True and arctg>0 and self.lidar_scan_forward()==True): 
                    twist_msg.angular.z = 0.4
                    self.cmd_vel_pub.publish(twist_msg)
                    self.rate.sleep()
                    arctg = math.atan2(diff_y, diff_x)-self.yaw
                    distance = math.sqrt(diff_x**2+diff_y**2)
                    rospy.loginfo("arctg 500 620(нужен поворот влево) {0}{1}".format(arctg,twist_msg.angular.z))     
                    rospy.loginfo("target_arctg, yaw {0}{1}".format(math.atan2(diff_y, diff_x),self.yaw))

                while (arctg<0 and self.lidar_scan_side(120,210)==True and self.lidar_scan_forward()==True): 
                    twist_msg.angular.z = -0.4
                    self.cmd_vel_pub.publish(twist_msg)
                    self.rate.sleep()
                    arctg = math.atan2(diff_y, diff_x)-self.yaw
                    distance = math.sqrt(diff_x**2+diff_y**2)
                    rospy.loginfo("arctg 120 210(нужен поворот вправо) {0} {1}".format(arctg,twist_msg.angular.z))
                    rospy.loginfo("target_arctg, yaw {0}{1}".format(math.atan2(diff_y, diff_x),self.yaw))

                twist_msg.angular.z = 0
                self.cmd_vel_pub.publish(twist_msg) 
                self.rate.sleep()
 
                # rospy.loginfo("yaw: {0}".format(self.yaw)) 
                # rospy.loginfo("arctg: {0}".format(arctg))
                # rospy.loginfo("arctg in degrees: {0}".format(arctg*180/math.pi))
                # rospy.loginfo("arctg in degrees: {0} {1}".format(diff_x, diff_y))
                diff_x = self.target_x - self.x  
                diff_y = self.target_y - self.y
                arctg = math.atan2(diff_y, diff_x)-self.yaw
                distance = math.sqrt(diff_x**2+diff_y**2)            
                self.rate.sleep()
            # rospy.loginfo("selfes in WHILE: {0} {1}".format(self.x, self.y))  
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