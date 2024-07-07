#!/usr/bin/env python3  
import math
from nav_msgs.msg import Odometry
import rospy  
from geometry_msgs.msg import Twist 
from std_msgs.msg import Bool
import tf  
from sensor_msgs.msg import LaserScan
import tf.transformations  
import numpy as np
from a_star.msg import Coords
from distributor.msg import Connect

  
class SimpleController():  
  
    def __init__(self):  
        self.number_robot_str='10'
        self.number_robot=10
        rospy.init_node('robot'+self.number_robot_str+'_controller_local', anonymous=True)  
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

        self.info_robots=Connect()
        self.info_robots_get_first=False

        self.cmd_vel_pub = rospy.Publisher('/robot'+self.number_robot_str+'/cmd_vel', Twist, queue_size=1)
        self.answer_to_gplanner = rospy.Publisher('/robot'+self.number_robot_str+'/answer/getpath', Bool, queue_size=1)   
        self.subscriber_info =rospy.Subscriber('/robots/info', Connect, self.robots_callback)
        self.subscriber =rospy.Subscriber('/robot'+self.number_robot_str+'/odom', Odometry, self.pose_callback)
        self.lidar_sub = rospy.Subscriber('/robot'+self.number_robot_str+'/scan', LaserScan, self.lidar_callback)
        self.path_sub = rospy.Subscriber('/robot'+self.number_robot_str+'/path', Coords, self.path_callback)

        while (self.get_path==False):
            self.rate.sleep()
            rospy.loginfo("Waiting path")
        # rospy.loginfo("Coordinates: {0} {1}".format(self.target_x , self.target_y))  
        # rospy.loginfo("Coordinates: {0} {1}".format(self.x, self.y))  


    def robots_callback(self,msg):

        if self.info_robots_get_first==False:
            count_robots=len(msg.y_cur)
            self.info_robots.x_cur =[0]*count_robots
            self.info_robots.y_cur =[0]*count_robots
            self.info_robots.tasks_important =[0]*count_robots
            self.info_robots_get_first=True

        self.info_robots.x_cur=msg.x_cur
        self.info_robots.y_cur=msg.y_cur
        self.info_robots.tasks_important=msg.tasks_important


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
        for i in range (315,405):
            if self.lidar_range[i]<1.5:
                return False
            
        for i in range (280,315):
            if self.lidar_range[i]<1.2:
                return False
            
        for i in range (405,440):
            if self.lidar_range[i]<1.5:
                return False 
        return True
    
    def lidar_scan_back(self):
        back_left_free=True
        back_right_free=True
        for i in range (0,65):
            if self.lidar_range[i]<2:
                back_left_free=False
                break
        for i in range (653,719):
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

    def rotate_for_free_side(self,angular, twist_msg):
        while (self.lidar_scan_forward()==False and self.round_move()==True) :
            twist_msg.angular.z = angular
            self.cmd_vel_pub.publish(twist_msg)
            self.rate.sleep()
            rospy.loginfo("I'm in while rotate for free side")

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
# если препятсиве только впереди слева 

        # if (self.forward_right_free==True and self.forward_left_free==False and self.lidar_scan_side(180,270)):
        #     self.rotate_for_free_side(-0.25,twist_msg)
        #     rospy.loginfo("Спереди справа свободно")

        # elif (self.forward_right_free==False and self.forward_left_free==True and self.lidar_scan_side(450,540)):
        #     self.rotate_for_free_side(0.25,twist_msg)
        #     rospy.loginfo("Сперели слева свободно")

        if (self.lidar_scan_side(120,210)):
            angular=-0.25
            while (self.lidar_scan_forward()==False and self.lidar_range[223]>0.6 and self.lidar_range[653]>0.9 and self.lidar_range[65]>0.9) :
                twist_msg.angular.z = angular
                self.cmd_vel_pub.publish(twist_msg)
                self.rate.sleep()

            twist_msg.angular.z = 0
            self.cmd_vel_pub.publish(twist_msg)
            self.rate.sleep() 

            if self.lidar_scan_forward()==True:
                twist_msg.linear.x = 0.25
                self.cmd_vel_pub.publish(twist_msg)
                self.rate.sleep()
            rospy.loginfo("Справа свободно")

        elif (self.lidar_scan_side(500,620)):
            angular=0.25
            while (self.lidar_scan_forward()==False and self.lidar_range[497]>0.6 and self.lidar_range[653]>0.9 and self.lidar_range[65]>0.9) :
                twist_msg.angular.z = angular
                self.cmd_vel_pub.publish(twist_msg)
                self.rate.sleep()

            twist_msg.angular.z = 0
            self.cmd_vel_pub.publish(twist_msg)
            self.rate.sleep() 

            if self.lidar_scan_forward()==True:
                twist_msg.linear.x = 0.25
                self.cmd_vel_pub.publish(twist_msg)
                self.rate.sleep()
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
                twist_msg.linear.x = 0
                self.cmd_vel_pub.publish(twist_msg)  
                self.rate.sleep()
            self.move_around_obstacles(twist_msg)

    def work_with_robots(self,twist_msg, index_obs):

        if twist_msg.linear.x!=0:
                twist_msg.linear.x = 0
                self.cmd_vel_pub.publish(twist_msg)  
                self.rate.sleep()

        if self.info_robots.tasks_important[index_obs]==True or self.info_robots.tasks_important[self.number_robot-1]==False:

            if (self.lidar_scan_side(120,210)):
                rospy.loginfo("Справа свободно")
                angular=-0.25
                self.rate.sleep()
                err_angle=-3.14-self.yaw
                while (abs(err_angle)>0.05 and self.lidar_range[223]>0.6 and self.lidar_range[653]>0.9 and self.lidar_range[65]>0.9) :
                    twist_msg.angular.z = angular
                    self.cmd_vel_pub.publish(twist_msg)
                    self.rate.sleep()
                    err_angle=-3.14-self.yaw

                twist_msg.angular.z = 0
                self.cmd_vel_pub.publish(twist_msg)
                self.rate.sleep() 

                if self.lidar_scan_forward()==True:
                    twist_msg.linear.x = 0.5
                    self.cmd_vel_pub.publish(twist_msg)
                    self.rate.sleep()

            elif (self.lidar_scan_side(500,620)):
                rospy.loginfo("Слева свободно")
                angular=0.25
                self.rate.sleep()
                err_angle=0-self.yaw
                while (abs(err_angle)>0.05 and self.lidar_range[497]>0.6 and self.lidar_range[653]>0.9 and self.lidar_range[65]>0.9) :
                    twist_msg.angular.z = angular
                    self.cmd_vel_pub.publish(twist_msg)
                    self.rate.sleep()
                    err_angle=0-self.yaw

                twist_msg.angular.z = 0
                self.cmd_vel_pub.publish(twist_msg)
                self.rate.sleep() 

                if self.lidar_scan_forward()==True:
                    twist_msg.linear.x = 0.5
                    self.cmd_vel_pub.publish(twist_msg)
                    self.rate.sleep()

            elif (self.lidar_scan_back()):
                twist_msg.linear.x=-0.5
                self.cmd_vel_pub.publish(twist_msg)  
                self.rate.sleep()
                rospy.loginfo("Двигаюсь назад")
            else:
                self.stuck_wait(twist_msg)
        else:
            if (self.lidar_scan_forward()==False):
                self.rate.sleep()


    def round_move(self):
        if self.lidar_range[223]<0.6 or self.lidar_range[497]<0.6 or self.lidar_range[653]<0.9 or self.lidar_range[65]<0.9:
            return False
        else: 
            return True
        
    def check_robots(self):
        for i in range (len(self.info_robots.y_cur)):
            if i==self.number_robot-1:
                continue
            self.robot_x = self.info_robots.x_cur[i] 
            self.robot_y = self.info_robots.y_cur[i] 
            diff_x = self.robot_x - self.x  
            diff_y = self.robot_y - self.y
            distance = math.sqrt(diff_x**2+diff_y**2)
            rospy.loginfo("robot_obs (x,y): {0} {1}".format(self.robot_x, self.robot_y))
            rospy.loginfo("odom x and y:{0} {1}".format(self.x, self.y))
            rospy.loginfo("distance:{0}".format(distance))
            if distance<3:
                return True, i 
        return False, -1

    def check_distance(self, i):
        if i<len(self.x_path):
            self.target_x = self.x_path[i] 
            self.target_y = self.y_path[i] 
            rospy.loginfo("goal_point (x,y): {0} {1}".format(self.target_x, self.target_y))
            rospy.loginfo("odom x and y:{0} {1}".format(self.x, self.y))
            diff_x = self.target_x - self.x  
            diff_y = self.target_y - self.y
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
        if  i<len(self.x_path):
            if  i+1<len(self.x_path) and distance>distance_next:
                return i+1, distance_next, arctg_next
            else:
                return i, distance, arctg
        else:
            return i, 3, 0


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
                    #if obstacle==robot
                    obs_robot=False
                    number_obs=-1
                    (obs_robot, number_obs)=self.check_robots()
                    if obs_robot==True:
                        self.work_with_robots(twist_msg, number_obs)
                    else:
                        self.work_with_obs(twist_msg)

                    if (abs(distance)<0.4):
                        break
                else:
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