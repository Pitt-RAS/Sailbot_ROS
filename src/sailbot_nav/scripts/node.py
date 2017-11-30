#!/usr/bin/env python2

import rospy
from math import atan2, sqrt, degrees, pi
from tf.transformations import euler_from_quaternion 
from algorithm import heading

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, PointStamped


class SailbotNav:
    def __init__(self):
        self.newHeadingPub = rospy.Publisher("/cmd_heading", Float32, queue_size=10)
        self.odomSub = rospy.Subscriber("/odom", Odometry, self.updateOdom)
        self.trueWindSub = rospy.Subscriber("/true_wind", Vector3, self.updateTrueWind)
        self.goalPointSub = rospy.Subscriber("/goal", PointStamped, self.updateGoalPoint)

        self.beatingParam = rospy.get_param("~beating_parameter", 5)

        self.odom = None
        self.trueWind = None
        self.goal = None

    def updateOdom(self, odom):
        self.odom = odom
        self.update()

    def updateTrueWind(self, trueWind):
        self.trueWind = trueWind

    def updateGoalPoint(self, goal):
        self.goal = goal

    def update(self):
        # If we don't know the robot state, don't update the planner
        if self.odom is None or self.trueWind is None or self.goal is None:
            return

        # Get magnitude / direction from true wind vector 
        windHeading = atan2(-self.trueWind.y, -self.trueWind.x)
        windSpeed = sqrt(self.trueWind.x**2 + self.trueWind.y**2)
        
        # Get yaw angle
        boatQuat = self.odom.pose.pose.orientation
        boatHeading = (euler_from_quaternion([boatQuat.x, boatQuat.y, boatQuat.z, boatQuat.w])[2]) % 2*pi

        boatVelocity = self.odom.twist.twist.linear.x
        boatPosition = [self.odom.pose.pose.position.x, self.odom.pose.pose.position.y]
        goalPoint = [self.goal.point.x, self.goal.point.y]

        # Run the algorithm
        newHeading = heading(boatPosition, boatHeading, goalPoint, windSpeed, windHeading, self.beatingParam) 

        # Send new heading
        newHeadingMsg = Float32(degrees(newHeading))
        self.newHeadingPub.publish(newHeadingMsg)


# Init node and spin
rospy.init_node("sailbot_nav")
nav = SailbotNav()
rospy.spin()

