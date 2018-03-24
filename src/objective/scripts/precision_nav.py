import rospy
import numpy as np
import rospy
import math
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def disstance_over_ground (x,y,bouy):

	x_diff = bouy[0]-x
	y_diff = bouy[1]-y

	disstance_over_ground = math.sqrt(math.pow(x_diff, 2) + math.pow(y_diff, 2))
	return disstance_over_ground

class Navigation_Node:
	def __init__(self):

		self.x = 0
		self.y = 0
		self.quat = 0
	
		self.disstance_over_ground = 0
		self.odom_sub = rospy.Subscriber("odometry/filtered",Odometry, 	self.localization_callback)
		
		buoy1 = rospy.get_param("buoyLocation1")
		buoy2 = rospy.get_param("buoyLocation2")
		buoy3 = rospy.get_param("bouyLocation3")

		self.current_goal = buoy1
		
		self.goalPublisher = rospy.Publisher("goal_point", Point, queue_size=10)
				

	def localization_callback(self, new_odom):
		self.x = new_odom.pose.pose.position.x
		self.y = new_odom.pose.pose.postiion.y
		
		self.quat = new_odom.pose.pose.orientation
		
	def update(self):
		disstance = disstance_over_ground(self.x, self.y, self.current_goal)
		if disstance < 10:
			if (current_goal == self.buoy1):
				self.current_goal = self.buoy2
			elif (self.current_gual == self.buoy2):
				self.current_goal =self.buoy3

					

		self.goalPublisher.publish(self.current_goal)


rospy.init_node("navigation", anonymous = false)
rate = rospy.Rate(rospy.get_param("~rate", 60))
node = Nivagation_Node()
while not rospy.is_shutdown():
	node.update()
	rate.sleep()



