#!/usr/bin/env python
import rospy
from alghorithms.astar import astar
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from a_star.msg import Coords
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point 


class Astar_planner():  
  
    def __init__(self):  
        rospy.init_node('Global_planning', anonymous=True)  
        rospy.loginfo("Node started")

        self.costmap=[]
        self.start_x=0
        self.start_y=0
        self.goal_x=0   
        self.goal_y=0
        self.origin_x=-30   
        self.origin_y=-50
        self.get_goal=False
        self.get_start=False
        self.rate = rospy.Rate(20)

        self.answer_from_robot = False

        self.subscriber_map =rospy.Subscriber('/map', OccupancyGrid, self.map_callback) 
        self.subscriber_odom =rospy.Subscriber('/odom', Odometry, self.start_pose_callback, queue_size=1)
        self.subscriber_goal =rospy.Subscriber('/goal/point', Point, self.goal_pose_callback, queue_size=1)

        self.publisher = rospy.Publisher('/path', Coords, queue_size=1)
        self.sub_from_robot =rospy.Subscriber('/answer/getpath', Bool, self.robot_callback)  

    def map_callback(self, msg):  
        self.costmap = msg.data
        while (len(self.costmap)==0):
            self.rate.sleep()
            rospy.loginfo("Wait map's update {0}".format(len(self.costmap)))

    def robot_callback(self, msg):  
        self.answer_from_robot = msg.data
        rospy.loginfo("Robot get path for goal")   

    def start_pose_callback(self, msg):  
        self.update_start(msg.pose.pose.position.x, msg.pose.pose.position.y)  
  
    def update_start(self, x, y):  
        self.start_x, self.start_y = x, y
        # rospy.loginfo("Start coordinates {0} {1}".format(self.start_x, self.start_y))
        self.get_start=True

    def goal_pose_callback(self, msg):  
        self.goal_update(msg.x, msg.y)  
  
    def goal_update(self, x, y):  
        self.goal_x, self.goal_y = x, y
        # rospy.loginfo("Goal coordinates {0} {1}".format(self.start_x, self.start_y))
        self.get_start=True



    def from_coord_to_index(self,x,y,height,width,resolution):
        index_x = int((x-self.origin_x)/resolution)
        index_y = int((y-self.origin_y)/resolution)
        index = int(index_y*width+index_x)
        # rospy.loginfo("Координаты х и у:{0} {1}".format(x,y))     
        # rospy.loginfo("index x:{0}".format(index_x))     
        # rospy.loginfo("common index:{0}".format(index)) 
        return index    

    def from_index_to_coords(self,height,width,resolution,index):
        index_x = index % width
        index_y = index // width
        x = index_x*resolution +self.origin_x
        y = index_y*resolution +self.origin_y
        return x, y   
    
    def make_plan(self):
        # number of columns in the occupancy grid
        width = 1200
        # number of rows in the occupancy grid
        height = 2000
        resolution = 0.05
        rospy.loginfo("Start index")     
        start_index = self.from_coord_to_index(self.start_x, self.start_y, height, width, resolution)
        rospy.loginfo("----------")     
        rospy.loginfo("Goal index")     
        goal_index = self.from_coord_to_index(self.goal_x, self.goal_y, height, width, resolution)
        # side of each grid map square in meters
        # origin of grid map
        (x,y) =self.from_index_to_coords(height,width,resolution,start_index)
        rospy.loginfo("Start coords from index х и у:{0} {1}".format(x,y))     
        (x,y) =self.from_index_to_coords(height,width,resolution,goal_index) 
        rospy.loginfo("Start goal from index х и у:{0} {1}".format(x,y))     
        origin = [-30, -50, 0]
        self.rate.sleep()
        path= astar(start_index, goal_index, width, height, self.costmap, resolution, origin)
        coords=Coords()
        coords.x=[0]*len(path)
        coords.y=[0]*len(path)

        for i in range(len(path)):
            (x,y) = self.from_index_to_coords(height,width,resolution,path[i])
            coords.x[i] = x
            coords.y[i] = y
            rospy.loginfo("---")     
            rospy.loginfo("Точка: {0}".format(i))
            rospy.loginfo("index: {0}".format(path[i]))     
            rospy.loginfo("Coords x and y: {0} {1}".format(x,y))

        while (self.answer_from_robot!=True):  
            self.publisher.publish(coords)  
            self.rate.sleep()
   
gplanner = Astar_planner()
while (gplanner.goal_x==gplanner.goal_y and gplanner.goal_x==0):
    gplanner.rate.sleep()
    rospy.loginfo("get_start and get_goal {0}{1}".format(gplanner.get_start, gplanner.get_goal))
gplanner.make_plan()            

