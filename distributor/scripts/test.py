#!/usr/bin/env python3  
import rospy 
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8
import roslaunch
from std_msgs.msg import Bool
from distributor.msg import Task
from distributor.msg import Odom


class Disributor():  
  
    def __init__(self):  
        rospy.init_node('test_distibutor', anonymous=True)  
        rospy.loginfo("Node started")
        self.rate = rospy.Rate(20)    
        self.tasks_active = list()
        self.task_finish = list()
        self.robots= list()

        self.answer_from_planner=False 
        self.pub_goal_1 =rospy.Publisher('/robot1/goal/', Task, queue_size=1)
        self.subscriber1 =rospy.Subscriber('/planner/answer', Bool, self.planner_callback, queue_size=1)

    def planner_callback(self, msg):  
        self.answer_from_planner = msg.data
        rospy.loginfo("Planner get goal")   

    def launch_file(self,package, node):
        node = roslaunch.core.Node(package, node)
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        script = launch.launch(node)
        print(script.is_alive())

distributor=Disributor()
task=Task()
task.level=1
task.status=1
task.number_robot=1
task.x=0
task.y=13
distributor.launch_file('a_star','path_planner_robot_1.py')
distributor.rate.sleep()
while (distributor.answer_from_planner==False):
    distributor.pub_goal_1.publish(task)
print("Planner get goal")
time=False
while(time==False):
    print("I live")


