#!/usr/bin/env python2

import rospy
from math import atan2, sqrt, degrees, pi, radians
from tf.transformations import euler_from_quaternion 
from algorithm import heading

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, PointStamped
from sailbot_sim.msg import TrueWind


class SailbotNav:
    def __init__(self):
        self.newHeadingPub = rospy.Publisher("/cmd_heading", Float32, queue_size=10)
        self.odomSub = rospy.Subscriber("/odom", Odometry, self.updateOdom)
        self.trueWindSub = rospy.Subscriber("/true_wind", TrueWind, self.updateTrueWind)
        self.goalPointSub = rospy.Subscriber("/goal", PointStamped, self.updateGoalPoint)

        self.beatingParam = rospy.get_param("~beating_parameter", 5)

        self.odom = None
        self.windHeading = None
        self.windSpeed = None
        self.goal = None

    def updateOdom(self, odom):
        self.odom = odom
        self.update()

    def updateTrueWind(self, trueWind):
        self.windHeading = radians(trueWind.direction)
        self.windSpeed = trueWind.speed

    def updateGoalPoint(self, goal):
        self.goal = goal

    def update(self):
        # If we don't know the robot state, don't update the planner
        if self.odom is None or self.windHeading is None or self.windSpeed is None or self.goal is None:
            return

        # Get yaw angle
        boatQuat = self.odom.pose.pose.orientation
        boatHeading = (euler_from_quaternion([boatQuat.x, boatQuat.y, boatQuat.z, boatQuat.w])[2]) % 2*pi

        boatVelocity = self.odom.twist.twist.linear.x
        boatPosition = [self.odom.pose.pose.position.x, self.odom.pose.pose.position.y]
        goalPoint = [self.goal.point.x, self.goal.point.y]

        # Run the algorithm
        newHeading = heading(boatPosition, boatHeading, goalPoint, self.windSpeed, self.windHeading, self.beatingParam) 

        # Send new heading
        newHeadingMsg = Float32(newHeading)
        self.newHeadingPub.publish(newHeadingMsg)


# Init node and spin
rospy.init_node("sailbot_nav")
nav = SailbotNav()
rospy.spin()

