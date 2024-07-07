#!/usr/bin/env python
from std_msgs.msg import String
import rospy
from elevator.msg import Lift
class Lift_class():  
  
    def __init__(self):
        self.msg=Lift()
        self.msg.free_elevator=True
        self.msg.robot_finish=True
        self.msg.level_elevator=1
        self.info_list=list()
        rospy.init_node('liftnode', anonymous=True)  
        rospy.loginfo("Lift_nodes started")
        self.rate = rospy.Rate(20) 
        self.publisher_info = rospy.Publisher('/elevator/info', Lift, queue_size=1)
        self.subscriber_planner =rospy.Subscriber('/elevator/command', Lift, self.planner_callback, queue_size=1)
        self.publisher_command = rospy.Publisher('/diplom/elevator', String, queue_size=1)

    def planner_callback(self, msg):
        self.update_info(msg.number_robot, msg.level, msg.robot_finish)

    def update_info(self, number, level, robot_finish):
        if (self.msg.free_elevator==True):
            level_str=String()  
            if level ==1:
                level_str.data="0"
            elif level==2:
                level_str.data="1"
            self.publisher_command.publish(level_str)
            self.msg.free_elevator =False
            self.msg.number_robot =number
            self.msg.level =level
            self.msg.level_elevator =level
            self.msg.robot_finish =robot_finish
            self.info_list.append(self.msg)
        else:
            if (self.msg.number_robot==number):
                if (robot_finish!=True):
                    level_str=String()  
                    if level == 1:
                        level_str.data="0"
                    elif level==2:
                        level_str.data="1"
                    self.publisher_command.publish(level_str)
                    self.msg.level_elevator =level
                else:
                    self.msg.free_elevator = True

lift = Lift_class()
while not rospy.is_shutdown():  
    lift.publisher_info.publish(lift.msg)  
    lift.rate.sleep()
print(lift.info_list)
