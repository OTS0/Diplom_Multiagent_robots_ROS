#!/usr/bin/env python3  
import math
import time
from nav_msgs.msg import Odometry
import rospy  
from geometry_msgs.msg import Twist 
from std_msgs.msg import Bool
import tf  
from sensor_msgs.msg import LaserScan
import tf.transformations  
import numpy as np
from a_star.msg import Coords
  
class SimpleController():  
  
    def __init__(self):  
        self.number_robot_str='8'
        self.number_robot=8
        rospy.init_node('MoveLift'+self.number_robot_str, anonymous=True)  
        rospy.on_shutdown(self.shutdown)  
        rospy.loginfo("Node started")  
        self.x=0  
        self.y=0
        self.z=0

        self.get_path = False  
        self.x_path = []  
        self.y_path = []

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
        self.rate = rospy.Rate(20)  

        self.cmd_vel_pub = rospy.Publisher('/robot'+self.number_robot_str+'/cmd_vel', Twist, queue_size=1)
        self.answer_to_gplanner = rospy.Publisher('/robot'+self.number_robot_str+'/answer/getpath', Bool, queue_size=1)    
        self.subscriber =rospy.Subscriber('/robot'+self.number_robot_str+'/odom', Odometry, self.pose_callback)
        self.lidar_sub = rospy.Subscriber('/robot'+self.number_robot_str+'/scan', LaserScan, self.lidar_callback)
        self.path_sub = rospy.Subscriber('/robot'+self.number_robot_str+'/path', Coords, self.path_callback)

        while (self.get_path==False):
            time.sleep(0.5)
            rospy.loginfo("Waiting path")
        # rospy.loginfo("Coordinates: {0} {1}".format(self.target_x , self.target_y))  
        # rospy.loginfo("Coordinates: {0} {1}".format(self.x, self.y))  

    def path_callback(self, msg):  
        self.x_path = msg.x
        self.y_path = msg.y
        if (len(self.x_path)>0 and len(self.y_path)>0):
            self.get_path=True
            answer=Bool()
            answer.data=True
            self.answer_to_gplanner.publish(answer)
        self.rate.sleep()


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
        # for i in range (270,450):
        for i in range (308,415):
            if self.lidar_range[i]<1:
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
        # for i in range (260,360):
        for i in range (308,360):
            if self.lidar_range[i]<1.5:
                self.forward_right_free = False
                rospy.loginfo("range and i {0} {1}".format(self.lidar_range[i], i))
                break
        # for i in range (360,460):
        for i in range (360,415):
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
        while (self.lidar_scan_forward()==False and self.round_move()==True):
            twist_msg.angular.z = 0.25
            self.cmd_vel_pub.publish(twist_msg)
            self.rate.sleep()
 
    def move_around_obstacles(self, twist_msg):
        self.lidar_check_forward()
# если препятсиве только впереди слева 

        if (self.forward_right_free==True and self.forward_left_free==False and self.lidar_scan_side(180,270)):
            self.rotate_for_free_side(-0.5,twist_msg)
            rospy.loginfo("Спереди справа свободно")
            twist_msg.linear.x = 0.5
            self.cmd_vel_pub.publish(twist_msg)
            self.rate.sleep()

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
            twist_msg.linear.x=-0.5
            self.cmd_vel_pub.publish(twist_msg)  
            self.rate.sleep()
            rospy.loginfo("Двигаюсь назад")
        else:
            self.stuck_wait(twist_msg)


    def work_with_obs(self,twist_msg):
        while (self.lidar_scan_forward()==False):
            if twist_msg.linear.x!=0:
                twist_msg.linear.x = 0.1*self.find_min_range_obs() 
                self.cmd_vel_pub.publish(twist_msg)  
                self.rate.sleep()
            self.move_around_obstacles(twist_msg)

    def round_move(self):
        if self.lidar_range[223]<0.6 or self.lidar_range[497]<0.6 or self.lidar_range[653]<0.9 or self.lidar_range[65]<0.9:
            return False
        else: 
            return True
        
        
    def check_distance(self, i):
 
        self.target_x = self.x_path[i] 
        self.target_y = self.y_path[i] 
        self.rate.sleep()
        rospy.loginfo("goal_point (x,y): {0} {1}".format(self.target_x, self.target_y))
        rospy.loginfo("odom x and y:{0} {1}".format(self.x, self.y))
        diff_x = self.target_x - self.x  
        diff_y = self.target_y - self.y
        self.rate.sleep()

        distance = math.sqrt(diff_x**2+diff_y**2)
        arctg = math.atan2(diff_y, diff_x)-self.yaw

        if i+1<len(self.x_path):
            self.target_x = self.x_path[i+1] 
            self.target_y = self.y_path[i+1] 
            rospy.loginfo("goal_point (x,y): {0} {1}".format(self.target_x, self.target_y))
            rospy.loginfo("odom x and y:{0} {1}".format(self.x, self.y))
            diff_x = self.target_x - self.x  
            diff_y = self.target_y - self.y
            distance_next = math.sqrt(diff_x**2+diff_y**2)
            arctg_next = math.atan2(diff_y, diff_x)-self.yaw

        if  i+1<len(self.x_path) and distance>distance_next:
            return i+1, distance_next, arctg_next
        else:
            return i, distance, arctg



    def move(self):
        for i in range (len(self.x_path)):
            twist_msg = Twist()   
            speed=0.0 
            self.rate.sleep()

            # self.cmd_vel_pub.publish(twist_msg)        
            self.target_x = self.x_path[i] 
            self.target_y = self.y_path[i] 
            rospy.loginfo("goal_point (x,y): {0} {1}".format(self.target_x, self.target_y))
            rospy.loginfo("odom x and y:{0} {1}".format(self.x, self.y))

            diff_x = self.target_x - self.x  
            diff_y = self.target_y - self.y
            arctg = math.atan2(diff_y, diff_x)-self.yaw
            rospy.loginfo("arctg first: {0}".format(arctg))
            distance = math.sqrt(diff_x**2+diff_y**2)
            self.rate.sleep()

            while (distance>0.4):
                diff_x = self.target_x - self.x  
                diff_y = self.target_y - self.y
                distance = math.sqrt(diff_x**2+diff_y**2)
                arctg = math.atan2(diff_y, diff_x)-self.yaw
                self.rate.sleep()

                while (abs(arctg)>0.05 and self.round_move()==True):
                # while abs(arctg)>0.05:
                    if (distance<0.4):
                        break 
                    twist_msg.linear.x = 0
                    twist_msg.angular.z = arctg*0.5
                    rospy.loginfo("-------")
                    rospy.loginfo("arctg in while {0}".format(arctg))
                    rospy.loginfo("arctg_goal in while {0}".format(math.atan2(diff_y, diff_x)))
                    rospy.loginfo("yaw {0}".format(self.yaw))

                    rospy.loginfo("-------")

                    self.cmd_vel_pub.publish(twist_msg)
                    self.rate.sleep()
                    diff_x = self.target_x - self.x  
                    diff_y = self.target_y - self.y
                    arctg = math.atan2(diff_y, diff_x)-self.yaw
                    distance = math.sqrt(diff_x**2+diff_y**2)            
                    self.rate.sleep()

                if (self.lidar_scan_forward()==False):
                    self.work_with_obs(twist_msg)
                    if (abs(distance)<0.4):
                        break

                speed = distance
                if speed > 6:
                    speed = 6
                if speed < -6:
                    speed = -6
                twist_msg.angular.z = 0                  
                twist_msg.linear.x = speed*0.5  
                self.cmd_vel_pub.publish(twist_msg)

                (i, distance, arctg)=self.check_distance(i)
                self.rate.sleep()
                
            rospy.loginfo("got point, move next")

        rospy.loginfo("Got goal")
        twist_msg.linear.x = 0 
        twist_msg.angular.z = 0 
        self.cmd_vel_pub.publish(twist_msg)
        self.rate.sleep()    
        self.answer_to_gplanner.publish(True)

    
    def shutdown(self):  
        self.cmd_vel_pub.publish(Twist())  
        rospy.sleep(1) 
  
simple_mover = SimpleController()              
simple_mover.move()