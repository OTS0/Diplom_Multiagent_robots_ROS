#!/usr/bin/env python
import rospy
import roslaunch
from alghorithms.astar import astar
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from a_star.msg import Coords
from nav_msgs.msg import Odometry
from distributor.msg import Task
from elevator.msg import Lift
from std_msgs.msg import Int8

class SimpleController():  
  
    def __init__(self):  
        rospy.init_node('robotlift', anonymous=True)  
        rospy.loginfo("Node started")  
        self.rate = rospy.Rate(20)

        self.lift_path=Coords()
        self.lift_path.x=[0]*5
        self.lift_path.y=[0]*5
        self.lift_path.x[0]=-23.5
        self.lift_path.y[0]=39.5
        self.lift_path.x[1]=-23.8
        self.lift_path.y[1]=39.4
        self.lift_path.x[2]=-24.3
        self.lift_path.y[2]=39.4
        self.lift_path.x[3]=-26
        self.lift_path.y[3]=39.4
        self.lift_path.x[4]=-27
        self.lift_path.y[4]=39.8

        self.answer_from_robot=False

        self.lift_path_from=Coords()
        self.lift_path_from.x=[0]*2
        self.lift_path_from.y=[0]*2
        self.lift_path_from.x[0]=-25.140731
        self.lift_path_from.y[0]=39.423090
        self.lift_path_from.x[1]=-23
        self.lift_path_from.y[1]=39.4

        self.sub_from_robot =rospy.Subscriber('/robot1/answer/getpath', Bool, self.robot_callback)  
        
        self.publisher_lift =rospy.Publisher('/elevator/command', Lift, queue_size=1)
        self.publisher_answer=rospy.Publisher('/planner1/answer', Bool, queue_size=1)
        self.publisher_path = rospy.Publisher('/robot1/path', Coords, queue_size=1)
    
    def robot_callback(self, msg):  
        self.answer_from_robot = msg.data
        rospy.loginfo("Robot get path for goal")   

lift_robot=SimpleController()
while (lift_robot.answer_from_robot!=True):  
    lift_robot.publisher_path.publish(lift_robot.lift_path)  
    lift_robot.rate.sleep()

msg=Lift()
msg.number_robot=1
msg.level = 1
msg.robot_finish=False
lift_robot.publisher_lift.publish(msg)
lift_robot.rate.sleep()