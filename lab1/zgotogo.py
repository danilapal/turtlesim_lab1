#! /usr/bin/python
import rospy
from geometry_msgs.msg  import Twist
from turtlesim.msg import Pose
from math import pow,atan2,sqrt

class turtlebot():

    def __init__(self):

        rospy.init_node('listener')
        self.velocity_publisher = rospy.Publisher('/turtle_2/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle_2/pose', Pose, self.callback)
	self._subscriber = rospy.Subscriber('/turtle_1/pose', Pose, self.move2goal)
	self.initPose = 1
        self.pose = None
        self.rate = rospy.Rate(10)

    def callback(self, data):
	
        self.pose = data

    def get_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))
        return distance

    def get_angle(self, goal_x, goal_y):
        angle = (atan2(goal_y - self.pose.y, goal_x - self.pose.x) - self.pose.theta)
        return angle

    def move2goal(self,msg):
	if self.pose is not None:
		goal_pose = Pose()
		goal_pose = msg
		distance_tolerance = 1.0
		vel_msg = Twist()

		if (self.get_distance(goal_pose.x, goal_pose.y) >= distance_tolerance):
   
			vel_msg.linear.x = 0.5 * self.get_distance(goal_pose.x, goal_pose.y)
			vel_msg.angular.z = 4 * self.get_angle(goal_pose.x, goal_pose.y)

			self.velocity_publisher.publish(vel_msg)
		    
x = turtlebot()
rospy.spin()
   
