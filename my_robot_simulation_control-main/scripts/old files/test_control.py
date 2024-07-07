import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

class robot_controller:
    def __init__(self, node_name):
        rospy.init_node(node_name, anonymous=True)

        self.vel_speed = 50.0
        self.rotate_speed = 50.0
        self.left_wheel = 0.0
        self.right_wheel = 0.0

        self.goal_point = rospy.Subscriber("/centre/goal", LaserScan, self.lidar_cb)
        self.left_wheel_pub = rospy.Publisher("/robot/left_wheel_controller/command", Float64, queue_size=10)
        self.right_wheel_pub = rospy.Publisher("/robot/right_wheel_controller/command", Float64, queue_size=10)
        self.lidar_sub = rospy.Subscriber("/robot/laser/scan", LaserScan, self.move_to_goal)

    def move_to_goal(self, data):
        ang = np.arange(data.angle_min, data.angle_max + data.angle_increment, data.angle_increment)
        collision_with_front = np.array(data.ranges) * np.sin(ang)
        if any(abs(collision_with_front) < 0.2):
            print("Avoiding obstacle.")
            if np.argmin(collision_with_front) < collision_with_front.size/2:
                self.left_speed = self.straight_speed + self.rotation_speed
                self.right_speed = self.straight_speed - self.rotation_speed
            else:
                self.left_speed = self.straight_speed - self.rotation_speed
                self.right_speed = self.straight_speed + self.rotation_speed   
        else:
            print('Moving forward.')
            self.left_speed = self.straight_speed
            self.right_speed = self.straight_speed

        self.left_wheel_pub.publish(self.left_speed)
        self.right_wheel_pub.publish(self.right_speed)


if __name__ == '__main__':
    robot_controller("controller")
    rospy.spin()

