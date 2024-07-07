#!/usr/bin/env python3  
import rospy 
import math
import roslaunch
import random
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from distributor.msg import Task
from distributor.msg import Odom
from distributor.msg import Connect
from elevator.msg import Lift


class Disributor():  
  
    def __init__(self):  
        rospy.init_node('main_distibutor', anonymous=True)  
        rospy.loginfo("Node started")
        self.rate = rospy.Rate(20)
        self.tasks_active = list()
        self.task_finish = list()
        self.robots= list()
        count_robots=10

        self.robots_connection = Connect()
        self.robots_connection.x_cur=[0]*count_robots
        self.robots_connection.y_cur=[0]*count_robots
        self.robots_connection.x_goal=[0]*count_robots
        self.robots_connection.y_goal=[0]*count_robots
        self.robots_connection.level=[0]*count_robots
        self.robots_connection.tasks_important=[False]*count_robots

        # self.load_task(10)
        self.load_my_task()

        self.load_robots(count_robots)

        self.answer_from_planner_1=False 
        self.answer_from_planner_2=False 
        self.answer_from_planner_3=False 
        self.answer_from_planner_4=False 
        self.answer_from_planner_5=False 
        self.answer_from_planner_6=False 
        self.answer_from_planner_7=False 
        self.answer_from_planner_8=False 
        self.answer_from_planner_9=False 
        self.answer_from_planner_10=False 

        self.got_odom=[False]*count_robots
        self.block_odom=True
        self.elevator_free=False
        self.elevator_level=1
 
        self.script_1 = None  
        self.script_2 = None    
        self.script_3 = None       
        self.script_4 = None      
        self.script_5 = None       
        self.script_6 = None  
        self.script_7 = None    
        self.script_8 = None       
        self.script_9 = None      
        self.script_10 = None  

        self.pub_robots =rospy.Publisher('/robots/info', Connect, queue_size=1)

        self.pub_goal_1 =rospy.Publisher('/robot1/goal/', Task, queue_size=1)
        self.pub_goal_2 =rospy.Publisher('/robot2/goal/', Task, queue_size=1)
        self.pub_goal_3 =rospy.Publisher('/robot3/goal/', Task, queue_size=1)
        self.pub_goal_4 =rospy.Publisher('/robot4/goal/', Task, queue_size=1)
        self.pub_goal_5 =rospy.Publisher('/robot5/goal/', Task, queue_size=1)
        self.pub_goal_6 =rospy.Publisher('/robot6/goal/', Task, queue_size=1)
        self.pub_goal_7 =rospy.Publisher('/robot7/goal/', Task, queue_size=1)
        self.pub_goal_8 =rospy.Publisher('/robot8/goal/', Task, queue_size=1)
        self.pub_goal_9 =rospy.Publisher('/robot9/goal/', Task, queue_size=1)
        self.pub_goal_10 =rospy.Publisher('/robot10/goal/', Task, queue_size=1)

        self.subscriber_lift =rospy.Subscriber('/elevator/info', Lift, self.lift_callback, queue_size=1)

        self.subscriber1 =rospy.Subscriber('/robot1/odom', Odometry, self.pose_callback, queue_size=1)
        self.subscriber2 =rospy.Subscriber('/robot2/odom', Odometry, self.pose_callback_2, queue_size=1)
        self.subscriber3 =rospy.Subscriber('/robot3/odom', Odometry, self.pose_callback_3, queue_size=1)
        self.subscriber4 =rospy.Subscriber('/robot4/odom', Odometry, self.pose_callback_4, queue_size=1)
        self.subscriber5 =rospy.Subscriber('/robot5/odom', Odometry, self.pose_callback_5, queue_size=1)
        self.subscriber6 =rospy.Subscriber('/robot6/odom', Odometry, self.pose_callback_6, queue_size=1)
        self.subscriber7 =rospy.Subscriber('/robot7/odom', Odometry, self.pose_callback_7, queue_size=1)
        self.subscriber8 =rospy.Subscriber('/robot8/odom', Odometry, self.pose_callback_8, queue_size=1)
        self.subscriber9 =rospy.Subscriber('/robot9/odom', Odometry, self.pose_callback_9, queue_size=1)
        self.subscriber10 =rospy.Subscriber('/robot10/odom', Odometry, self.pose_callback_10, queue_size=1)

        self.subscriber_planner1 =rospy.Subscriber('/planner1/answer', Bool, self.planner1_callback, queue_size=1)
        self.subscriber_planner2 =rospy.Subscriber('/planner2/answer', Bool, self.planner2_callback, queue_size=1)
        self.subscriber_planner3 =rospy.Subscriber('/planner3/answer', Bool, self.planner3_callback, queue_size=1)
        self.subscriber_planner4 =rospy.Subscriber('/planner4/answer', Bool, self.planner4_callback, queue_size=1)
        self.subscriber_planner5 =rospy.Subscriber('/planner5/answer', Bool, self.planner5_callback, queue_size=1)
        self.subscriber_planner6 =rospy.Subscriber('/planner6/answer', Bool, self.planner6_callback, queue_size=1)
        self.subscriber_planner7 =rospy.Subscriber('/planner7/answer', Bool, self.planner7_callback, queue_size=1)
        self.subscriber_planner8 =rospy.Subscriber('/planner8/answer', Bool, self.planner8_callback, queue_size=1)
        self.subscriber_planner9 =rospy.Subscriber('/planner9/answer', Bool, self.planner9_callback, queue_size=1)
        self.subscriber_planner10 =rospy.Subscriber('/planner10/answer', Bool, self.planner10_callback, queue_size=1)

        self.subscriber_status_1 =rospy.Subscriber('/robot1/status', Int8, self.status_callback, queue_size=1)
        self.subscriber_status_2 =rospy.Subscriber('/robot2/status', Int8, self.status_callback_2, queue_size=1)
        self.subscriber_status_3 =rospy.Subscriber('/robot3/status', Int8, self.status_callback_3, queue_size=1)
        self.subscriber_status_4 =rospy.Subscriber('/robot4/status', Int8, self.status_callback_4, queue_size=1)
        self.subscriber_status_5 =rospy.Subscriber('/robot5/status', Int8, self.status_callback_5, queue_size=1)
        self.subscriber_status_6 =rospy.Subscriber('/robot6/status', Int8, self.status_callback_6, queue_size=1)
        self.subscriber_status_7 =rospy.Subscriber('/robot7/status', Int8, self.status_callback_7, queue_size=1)
        self.subscriber_status_8 =rospy.Subscriber('/robot8/status', Int8, self.status_callback_8, queue_size=1)
        self.subscriber_status_9 =rospy.Subscriber('/robot9/status', Int8, self.status_callback_9, queue_size=1)
        self.subscriber_status_10 =rospy.Subscriber('/robot10/status', Int8, self.status_callback_10, queue_size=1)
        # all odom from robots 
        # self.subscriber =rospy.Subscriber('/odom', Odometry, self.pose_callback)

        self.rate.sleep()
        
        print("GOALS:",self.tasks_active)
        print("---")
        print("ROBOTS",self.robots)
        print("---")


    def lift_callback(self, msg):  
        self.elevator_free = msg.free_elevator
        self.elevator_level= msg.level_elevator
        
    def planner1_callback(self, msg):  
        self.answer_from_planner_1 = msg.data
        rospy.loginfo(" ")   
        rospy.loginfo("Planner 1 get goal")   
        rospy.loginfo(" ")   

    def planner2_callback(self, msg):  
        self.answer_from_planner_2 = msg.data
        rospy.loginfo(" ")   
        rospy.loginfo("Planner 2 get goal")   
        rospy.loginfo(" ")   

    def planner3_callback(self, msg):  
        self.answer_from_planner_3 = msg.data
        rospy.loginfo(" ")   
        rospy.loginfo("Planner 3 get goal")   
        rospy.loginfo(" ")   

    def planner4_callback(self, msg):  
        self.answer_from_planner_4 = msg.data
        rospy.loginfo(" ")   
        rospy.loginfo("Planner 4 get goal")   
        rospy.loginfo(" ")   

    def planner5_callback(self, msg):  
        self.answer_from_planner_5 = msg.data
        rospy.loginfo(" ")   
        rospy.loginfo("Planner 5 get goal")   
        rospy.loginfo(" ")  

    def planner6_callback(self, msg):  
        self.answer_from_planner_6 = msg.data
        rospy.loginfo(" ")   
        rospy.loginfo("Planner 6 get goal")   
        rospy.loginfo(" ")   

    def planner7_callback(self, msg):  
        self.answer_from_planner_7 = msg.data
        rospy.loginfo(" ")   
        rospy.loginfo("Planner 7 get goal")   
        rospy.loginfo(" ")   

    def planner8_callback(self, msg):  
        self.answer_from_planner_8 = msg.data
        rospy.loginfo(" ")   
        rospy.loginfo("Planner 8 get goal")   
        rospy.loginfo(" ")   

    def planner9_callback(self, msg):  
        self.answer_from_planner_9 = msg.data
        rospy.loginfo(" ")   
        rospy.loginfo("Planner 9 get goal")   
        rospy.loginfo(" ")   

    def planner10_callback(self, msg):  
        self.answer_from_planner_10 = msg.data
        rospy.loginfo(" ")   
        rospy.loginfo("Planner 10 get goal")   
        rospy.loginfo(" ") 

    def pose_callback(self, msg):  
        level=1
        if (msg.pose.pose.position.z<9.0):
            level=1
        else:
            level=2 
        self.got_odom[0]=True
        self.update_pose(msg.pose.pose.position.x, msg.pose.pose.position.y, level, 1, '/robot1/odom') 

    def pose_callback_2(self, msg):  
        level=1
        if (msg.pose.pose.position.z<9):
            level=1
        else:
            level=2 
        self.got_odom[1]=True
        self.update_pose(msg.pose.pose.position.x, msg.pose.pose.position.y, level, 2, '/robot2/odom')  

    def pose_callback_3(self, msg):  
        level=1
        if (msg.pose.pose.position.z<9.0):
            level=1
        else:
            level=2 
        self.got_odom[2]=True
        self.update_pose(msg.pose.pose.position.x, msg.pose.pose.position.y, level, 3, '/robot3/odom') 

    def pose_callback_4(self, msg):  
        level=1
        if (msg.pose.pose.position.z<9):
            level=1
        else:
            level=2 
        self.got_odom[3]=True
        self.update_pose(msg.pose.pose.position.x, msg.pose.pose.position.y, level, 4, '/robot4/odom') 

    def pose_callback_5(self, msg):  
        level=1
        if (msg.pose.pose.position.z<9):
            level=1
        else:
            level=2 
        self.got_odom[4]=True
        self.update_pose(msg.pose.pose.position.x, msg.pose.pose.position.y, level, 5, '/robot5/odom') 

    def pose_callback_6(self, msg):  
        level=1
        if (msg.pose.pose.position.z<9.0):
            level=1
        else:
            level=2 
        self.got_odom[5]=True
        self.update_pose(msg.pose.pose.position.x, msg.pose.pose.position.y, level, 6, '/robot6/odom') 

    def pose_callback_7(self, msg):  
        level=1
        if (msg.pose.pose.position.z<9):
            level=1
        else:
            level=2 
        self.got_odom[6]=True
        self.update_pose(msg.pose.pose.position.x, msg.pose.pose.position.y, level, 7, '/robot7/odom')  

    def pose_callback_8(self, msg):  
        level=1
        if (msg.pose.pose.position.z<9.0):
            level=1
        else:
            level=2 
        self.got_odom[7]=True
        self.update_pose(msg.pose.pose.position.x, msg.pose.pose.position.y, level, 8, '/robot8/odom') 

    def pose_callback_9(self, msg):  
        level=1
        if (msg.pose.pose.position.z<9):
            level=1
        else:
            level=2 
        self.got_odom[8]=True
        self.update_pose(msg.pose.pose.position.x, msg.pose.pose.position.y, level, 9, '/robot9/odom') 

    def pose_callback_10(self, msg):  
        level=1
        if (msg.pose.pose.position.z<9):
            level=1
        else:
            level=2 
        self.got_odom[9]=True
        self.update_pose(msg.pose.pose.position.x, msg.pose.pose.position.y, level, 10, '/robot10/odom') 

    def update_pose(self, x, y, level, number, topic):  
        self.robots[number-1].x_cur=x
        self.robots[number-1].y_cur=y
        self.robots[number-1].level=level
        self.robots[number-1].topic=topic

        self.robots_connection.x_cur[number-1]=x
        self.robots_connection.y_cur[number-1]=y

    def status_callback(self, msg):  
        self.update_status(msg.data, 1) 

    def status_callback_2(self, msg):  
        self.update_status(msg.data, 2) 

    def status_callback_3(self, msg):  
        self.update_status(msg.data, 3) 

    def status_callback_4(self, msg):  
        self.update_status(msg.data, 4) 

    def status_callback_5(self, msg):  
        self.update_status(msg.data, 5) 

    def status_callback_6(self, msg):  
        self.update_status(msg.data, 6) 

    def status_callback_7(self, msg):  
        self.update_status(msg.data, 7) 

    def status_callback_8(self, msg):  
        self.update_status(msg.data, 8) 

    def status_callback_9(self, msg):  
        self.update_status(msg.data, 9) 

    def status_callback_10(self, msg):  
        self.update_status(msg.data, 10) 

    def update_status(self, status, number):
        #if robot has done task
        print("I get status")
        print(self.robots[number-1])
        if (self.robots[number-1].status_robot==1 and status==0):
            for j in range(len(self.tasks_active)):
                if j<len(self.tasks_active) and self.tasks_active[j].number_robot==number:
                    self.tasks_active[j].finish=rospy.Time.now()
                    self.task_finish.append(self.tasks_active[j])
                    deleted_el=self.tasks_active.pop(j)
                    print("deleted from tasks:", deleted_el)
        if number==1:
            self.answer_from_planner_1=False
            while(self.script_1.is_alive()==True):
                self.rate.sleep()
        elif number==2:
            self.answer_from_planner_2=False
            while(self.script_2.is_alive()==True):
                self.rate.sleep()
        elif number==3:
            self.answer_from_planner_3=False
            while(self.script_3.is_alive()==True):
                self.rate.sleep()
        elif number==4:
            self.answer_from_planner_4=False
            while(self.script_4.is_alive()==True):
                self.rate.sleep()
        elif number==5:
            self.answer_from_planner_5=False
            while(self.script_5.is_alive()==True):
                self.rate.sleep()
        elif number==6:
            self.answer_from_planner_6=False
            while(self.script_6.is_alive()==True):
                self.rate.sleep()
        elif number==7:
            self.answer_from_planner_7=False
            while(self.script_7.is_alive()==True):
                self.rate.sleep()
        elif number==8:
            self.answer_from_planner_8=False
            while(self.script_8.is_alive()==True):
                self.rate.sleep()
        elif number==9:
            self.answer_from_planner_9=False
            while(self.script_9.is_alive()==True):
                self.rate.sleep()
        elif number==10:
            self.answer_from_planner_10=False
            while(self.script_10.is_alive()==True):
                self.rate.sleep()
        self.robots[number-1].status_robot=status
        print(status, number)



    def load_robots(self, counts_robots):
        i=1
        while (i<counts_robots+1):
            robot = Odom()
            robot.number_robot=i
            robot.status_robot=0
            self.robots.append(robot)
            i=i+1
        self.robots.sort(key=lambda x: x.number_robot, reverse=False)

    def load_task(self, count_task):
        for i in range(count_task):
            random_y = random.randrange(11)
            random_level=random.randrange(1,3)
            random_urgently=random.randrange(2)
            urgently=False
            if random_urgently==0:
                urgently=False
            else:
                urgently=True
            x_goal = random.randint(-27,27)
            y_goal = 24.6-7.2*random_y
            self.made_task(x_goal, y_goal, random_level, urgently, 0)
        self.tasks_active.sort(key=lambda x: x.urgently, reverse=True)
        # print("Loaded goals:")
        # print(self.tasks_active)

    def load_my_task(self):
        self.made_task(0, 0, 1, True, 0)
        self.made_task(0, 0, 1, True, 0)
        self.made_task(15, 24.6, 1, False, 0)
        self.made_task(15, 24.6, 1, True, 0)
        self.made_task(0, -11.4, 1, True, 0)
        self.made_task(0, 11.4, 2, False, 0)
        self.made_task(10, -47.4, 2, False, 0)
        self.made_task(10, -47.4, 2, False, 0)
        self.made_task(5, 3, 1, False, 0)
        self.made_task(5, 3, 2, True, 0)
        self.made_task(0, 10, 2, True, 0)
        self.made_task(0, 10, 1, True, 0)
        self.made_task(-15, 24.6, 1, False, 0)
        self.made_task(-15, 24.6, 2, True, 0)
        self.made_task(5, -11.4, 2, True, 0)
        self.made_task(5, 11.4, 2, False, 0)
        self.made_task(15, -47.4, 1, False, 0)
        self.made_task(15, -47.4, 2, False, 0)
        self.made_task(0, 3, 1, False, 0)
        self.made_task(0, 3, 2, True, 0)
        self.tasks_active.sort(key=lambda x: x.urgently, reverse=True)
        print(self.tasks_active)

    def made_task(self,x,y,level,urgently, status):
        task = Task()
        task.x=x
        task.y=y
        task.level=level
        task.status=status
        task.urgently=urgently
        self.tasks_active.append(task)

    def distribute(self,ready_robots):
        for j in range(len(self.tasks_active)):
            if j<len(self.tasks_active):
                if (self.tasks_active[j].status==0):
                    print("We have tasks")
                    if (len(ready_robots)>0):
                        one_level_list = list()
                        for i in range(len(ready_robots)):
                            if ready_robots[i].level == self.tasks_active[j].level:
                                one_level_list.append(ready_robots[i])
                                print("add robots in one_level_list",ready_robots[i])
                        if len(one_level_list)>0:
                            print(len(one_level_list))
                            if len(one_level_list)==1:
                                number=one_level_list[0].number_robot
                                self.robots[number-1].status_robot=1
                                self.tasks_active[j].status=1
                                self.tasks_active[j].number_robot=number
                                self.tasks_active[j].start=rospy.Time.now()
                                self.robots_connection.tasks_important[number-1]=self.tasks_active[j].urgently
                                self.robots_connection.x_goal[number-1]=self.tasks_active[j].x
                                self.robots_connection.y_goal[number-1]=self.tasks_active[j].y
                                self.robots_connection.level[number-1]=self.tasks_active[j].level

                                ready_robots =self.deleted_ready_robots(ready_robots, number)
                                
                                #send task
                                self.send_task(j)
                                self.rate.sleep()
                            else:
                                sorted_robots_for_goal = self.sort_robots_by_distance(one_level_list, self.tasks_active[j].x , self.tasks_active[j].y)
                                number=sorted_robots_for_goal[0].number_robot
                                print("Назначенный робот для задачи:", number)
                                rospy.loginfo("Задача ху, уровень:{0} {1} {2}".format(self.tasks_active[j].x, self.tasks_active[j].y, self.tasks_active[j].level))
                                self.robots_connection.tasks_important[number-1]=self.tasks_active[j].urgently
                                self.robots[number-1].status_robot=1
                                self.tasks_active[j].status=1
                                self.tasks_active[j].number_robot=number
                                self.tasks_active[j].start=rospy.Time.now()
                                self.robots_connection.x_goal[number-1]=self.tasks_active[j].x
                                self.robots_connection.y_goal[number-1]=self.tasks_active[j].y
                                self.robots_connection.level[number-1]=self.tasks_active[j].level

                                ready_robots =self.deleted_ready_robots(ready_robots, number)

                                #send task
                                self.send_task(j)
                                self.rate.sleep()

                                # print("!!!!!!Attention!!!!!")
                                # print("sorted lise",sorted_robots_for_goal)
                        else:
                            if self.elevator_free==True:
                                sorted_robots_for_goal = self.sort_robots_by_distance(ready_robots, -23 , 40)
                                number=sorted_robots_for_goal[0].number_robot
                                print("Назначенный робот для задачи:", number)
                                rospy.loginfo("Задача ху, уровень:{0} {1} {2}".format(self.tasks_active[j].x, self.tasks_active[j].y, self.tasks_active[j].level))
                                self.robots[number-1].status_robot=1
                                self.robots_connection.tasks_important[number-1]=self.tasks_active[j].urgently
                                self.tasks_active[j].status=1
                                self.tasks_active[j].number_robot=number
                                self.tasks_active[j].start=rospy.Time.now()
                                self.robots_connection.x_goal[number-1]=self.tasks_active[j].x
                                self.robots_connection.y_goal[number-1]=self.tasks_active[j].y
                                self.robots_connection.level[number-1]=self.tasks_active[j].level

                                ready_robots =self.deleted_ready_robots(ready_robots, number)
                                self.elevator_free=False
                                #send task
                                self.send_task(j)
                                self.rate.sleep()

                            else:
                                #go to other task
                                continue
                    else:
                        break
                else: 
                    continue
            else:
                break

    def send_task(self, j):
        if self.tasks_active[j].number_robot==1:
            if self.script_1==None or self.script_1.is_alive()==False:
                self.script_1 = self.launch_file('a_star','path_planner_robot_1.py')
                self.rate.sleep()
            while (self.answer_from_planner_1==False):
                self.pub_goal_1.publish(self.tasks_active[j])
                rospy.loginfo("I wait answer from planner 1")
                self.rate.sleep()

        elif self.tasks_active[j].number_robot==2:
            if self.script_2==None or self.script_2.is_alive()==False:
                self.script_2 = self.launch_file('a_star','path_planner_robot_2.py')
                self.rate.sleep()
            while (self.answer_from_planner_2==False):
                self.pub_goal_2.publish(self.tasks_active[j])
                rospy.loginfo("I wait answer from planner 2")
                self.rate.sleep()

        elif self.tasks_active[j].number_robot==3:
            if self.script_3==None or self.script_3.is_alive()==False:
                self.script_3 = self.launch_file('a_star','path_planner_robot_3.py')
                self.rate.sleep()
            while (self.answer_from_planner_3==False):
                self.pub_goal_3.publish(self.tasks_active[j])
                rospy.loginfo("I wait answer from planner 3")
                self.rate.sleep()

        elif self.tasks_active[j].number_robot==4:
            if self.script_4==None or self.script_4.is_alive()==False:
                self.script_4 = self.launch_file('a_star','path_planner_robot_4.py')
                self.rate.sleep()
            while (self.answer_from_planner_4==False):
                self.rate.sleep()
                self.pub_goal_4.publish(self.tasks_active[j])
                rospy.loginfo("I wait answer from planner 4")
                self.rate.sleep()

        elif self.tasks_active[j].number_robot==5:
            if self.script_5==None or self.script_5.is_alive()==False:
                self.script_5 = self.launch_file('a_star','path_planner_robot_5.py')
                self.rate.sleep()
            while (self.answer_from_planner_5==False):
                self.rate.sleep()
                self.pub_goal_5.publish(self.tasks_active[j])
                rospy.loginfo("I wait answer from planner 5")
                self.rate.sleep()

        elif self.tasks_active[j].number_robot==6:
            if self.script_6==None or self.script_6.is_alive()==False:
                self.script_6 = self.launch_file('a_star','path_planner_robot_6.py')
                self.rate.sleep()
            while (self.answer_from_planner_6==False):
                self.pub_goal_6.publish(self.tasks_active[j])
                rospy.loginfo("I wait answer from planner 6")
                self.rate.sleep()

        elif self.tasks_active[j].number_robot==7:
            if self.script_7==None or self.script_7.is_alive()==False:
                self.script_7 = self.launch_file('a_star','path_planner_robot_7.py')
                self.rate.sleep()
            while (self.answer_from_planner_7==False):
                self.pub_goal_7.publish(self.tasks_active[j])
                rospy.loginfo("I wait answer from planner 7")
                self.rate.sleep()

        elif self.tasks_active[j].number_robot==8:
            if self.script_8==None or self.script_8.is_alive()==False:
                self.script_8 = self.launch_file('a_star','path_planner_robot_8.py')
                self.rate.sleep()
            while (self.answer_from_planner_8==False):
                self.pub_goal_8.publish(self.tasks_active[j])
                rospy.loginfo("I wait answer from planner 8")
                self.rate.sleep()

        elif self.tasks_active[j].number_robot==9:
            if self.script_9==None or self.script_9.is_alive()==False:
                self.script_9 = self.launch_file('a_star','path_planner_robot_9.py')
                self.rate.sleep()
            while (self.answer_from_planner_9==False):
                self.rate.sleep()
                self.pub_goal_9.publish(self.tasks_active[j])
                rospy.loginfo("I wait answer from planner 9")
                self.rate.sleep()

        elif self.tasks_active[j].number_robot==10:
            if self.script_10==None or self.script_10.is_alive()==False:
                self.script_10 = self.launch_file('a_star','path_planner_robot_10.py')
                self.rate.sleep()
            while (self.answer_from_planner_10==False):
                self.rate.sleep()
                self.pub_goal_10.publish(self.tasks_active[j])
                rospy.loginfo("I wait answer from planner 10")
                self.rate.sleep()
        
    def launch_file(self,package, node):
        node = roslaunch.core.Node(package, node)
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        script = launch.launch(node)
        print(script.is_alive())
        return script

    def deleted_ready_robots(self, ready_robots, number_robot):
        for i in range(len(ready_robots)):
            if i>=len(ready_robots):
                break
            if ready_robots[i].number_robot==number_robot:
                print("delete robot from ready robots:")
                print(ready_robots.pop(i))
                print("------")

        return ready_robots

    def distance_to_target(self,robot,target_x, target_y):
        return math.sqrt(((robot.x_cur - target_x) ** 2 + (robot.y_cur - target_y) ** 2))
    
    def sort_robots_by_distance(self, robot_list, target_x, target_y):
        sorted_robots = sorted(robot_list, key=lambda robot: self.distance_to_target(robot, target_x, target_y))
        return sorted_robots

                
    def update_ready_robots(self):
        ready_robots=list()
        for i in range (0,len(self.robots)):
            if (self.robots[i].status_robot ==0):
                ready_robots.append(self.robots[i])

        return ready_robots
    
    def analyze_tasks(self):
        count_tasks=len(self.task_finish)
        sum_all=0
        min_time=math.inf
        max_time=0
        for i in range(count_tasks):
            time_goal=(self.task_finish[i].finish.secs-self.task_finish[i].start.secs)/60
            sum_all=sum_all+time_goal
            if time_goal<min_time:
                min_time=time_goal
            if time_goal>max_time:
                max_time=time_goal
        middle=sum_all/count_tasks
        rospy.loginfo("Cреднее время выполнения задачи, минимальное и максимальное время выполнения в минутах {0} {1} {2}".format(middle, min_time, max_time))
        rospy.loginfo("количество выполненных задач {0}".format(count_tasks))

if __name__ == '__main__':
    try:
        distributor=Disributor()
        find_robot_without_odom=False
        while(distributor.block_odom==True):
            for i in range(len(distributor.got_odom)):
                if distributor.got_odom[i]==False:
                    find_robot_without_odom=True
            if find_robot_without_odom==False:
                distributor.block_odom=False
                print("All robots get odoms")

        start_time=rospy.Time.now()
        while (len(distributor.tasks_active)!=0):
            ready_robots = distributor.update_ready_robots()
            distributor.pub_robots.publish(distributor.robots_connection)
            if (len(ready_robots)>0):
                distributor.distribute(ready_robots)
                # print("ready robots:", ready_robots)
            else:
                print("I sleep, wasn't ready robot")
                distributor.rate.sleep()
        print(distributor.task_finish)
        finish_time=rospy.Time.now()
        time_work=(finish_time.secs-start_time.secs)//60
        print("finish time in min")
        print(time_work)
        print("-----")
        distributor.analyze_tasks()
    except rospy.exceptions.ROSTimeMovedBackwardsException: pass
    except rospy.ROSInterruptException: pass



