#!/usr/bin/env python2
import rospy
import numpy as np
import rospy
import math
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from objective.msg import Goal
from geometry_msgs.msg import Point, PointStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf import TransformListener



class Navigation_Node:
	def __init__(self):

		self.x = 0
		self.y = 0
		self.quat = 0
		
		self.buoy1 = rospy.get_param("buoy1")
		self.buoy2 = rospy.get_param("buoy2")
		self.buoy3 = rospy.get_param("buoy3")


		if (self.buoy3[0]>0):
			self.buoy2a = [buoy2[0]-5, bouy2[1]]	
			self.buoy2b = [buoy2[0], bouy2[1]+5]
			self.buoy3a = [buoy3[0]+5, bouy3[1]+5]
			self.buoy3b = [buoy3[0]-5, bouy3[1]-5]
		elif(self.buoy3[0]<0):
			self.buoy2a = [buoy2[0]+5, bouy2[1]]
			self.buoy2b = [buoy2[0], bouy2[1]+5]
			self.buoy3a = [buoy3[0]-5, bouy3[1]-5]
			self.buoy3b = [buoy3[0]+5, bouy3[1]+5]


		self.current_goal = self.buoy1
		
		self.goalPublisher = rospy.Publisher("goal", Goal, queue_size=10)
		
		self.transform = TransformListener()
		self.odom_sub = rospy.Subscriber("odometry/filtered",Odometry, 	self.localization_callback)
				

	def localization_callback(self, new_odom):
		point = PointStamped(header=new_odom.header, point=new_odom.pose.pose.position)
		#boatPos = self.transform.transformPoint("utm", point)
		self.x = point.point.x
		self.y = point.point.y
		
		self.quat = new_odom.pose.pose.orientation

	def disstance_over_ground(self, x,y,bouy):

		x_diff = bouy[0]-x
		y_diff = bouy[1]-y

		disstance_over_ground = math.sqrt(math.pow(x_diff, 2) + math.pow(y_diff, 2))
		return disstance_over_ground

		
	def update(self):
		disstance = self.disstance_over_ground(self.x, self.y, self.current_goal)
		if disstance < 10:
			if (self.current_goal == self.buoy1):
				self.current_goal = self.buoy2a
			elif (self.current_goal == self.buoy2a):
				self.current_goal =self.buoy2b
			elif (self.current_goal == buoy2b):
				self.current_goal = self.buoy3a
			elif (self.current_goal == buoy3a):
				self.current_goal = self.buoy3b
			elif (self.current_goal == self.buoy3b):
				self.current_goal = bouy1

		msg = Goal()
		msg.goalType = 0 
		pointer = Point()

		pointer.x = self.current_goal[0]
		pointer.y = self.current_goal[1]
		pointer.z = 0

		msg.goalPoint = pointer
		msg.header.stamp = rospy.Time.now()			
		self.goalPublisher.publish(msg)


rospy.init_node("navigation", anonymous = False)
rate = rospy.Rate(rospy.get_param("~rate", 60))
node = Navigation_Node()
while not rospy.is_shutdown():
	node.update()
	rate.sleep()



