import rospy
import numpy as np
import rospy
import math
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def disstance_over_ground (x,y,bouy1,bouy2)

	x_diff = bouy1-x
	y_diff = bouy2-y
	disstance_over_ground = math.sqrt(math.pow(x_diff, 2) + math.pow(y_diff, 2))
	return disstance_over_ground

class Navigation_Node
	def __init__(self):
		rospy.init_node("objective_navigation")		
		self.disstance_over_ground = 0
		self.odom_sub = rospy.Subscriber("odometry/filtered",Odometry, 	self.localization_callback)
		
		buoy1 = rospy.get_param("buoyLocation1", 0)
		buoy2 = rospy.get_param("buoyLocation1", 1)
		
		self.current_goal = 0
		self.disstance_over_ground
		self.goalPublisher = rospy.Publisher("goal_point", Point, queue_size=10)
		self.goalPublisher.publish(goal)
		
		while (disstance_over_ground < 10)
			update()
			currentGoal++		

	def localization_callback(self, new_odom)
		x = new_odom.pose.pose.linear.x
		y = new_odom.pose.pose.linear.y
		
		self.disstance_over_ground = math.sqrt(math.pow(x, 2) + math.pow(y, 2))
		quat = new_odom.pose.pose.orientation
		
	def update(self):
		
 	
	return

rospy.init_node("Nivagation", anonymous = false)
rate = rospy.Rate(rospy.get_param("~rate", 60))
node = Nivagation_Node()
while not rospy.is_shutdown():
	node.update()
	rate.sleep()



