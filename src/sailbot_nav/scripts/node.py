#!/usr/bin/env python2

import rospy
from math import atan2, sqrt, degrees, radians, pi
from tf.transformations import euler_from_quaternion
from algorithm import heading

from std_msgs.msg import Int32, Float32
from nav_msgs.msg import Odometry
from sailbot_sim.msg import TrueWind
from objective.msg import Goal
from geometry_msgs.msg import Vector3, PointStamped


class SailbotNav:
    def __init__(self):
        self.newHeadingPub = rospy.Publisher("/cmd_heading", Float32, queue_size=10)
        self.odomSub = rospy.Subscriber("/odometry/filtered", Odometry, self.updateOdom)
        self.trueWindSub = rospy.Subscriber("/true_wind", TrueWind, self.updateTrueWind)
        self.goalPointSub = rospy.Subscriber("/goal", Goal, self.updateGoalPoint)

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
        self.goal = goal.goalPoint

    def update(self):
        # If we don't know the robot state, don't update the planner
        if self.odom is None or self.trueWind is None or self.goal is None:
            return

        # Get magnitude / direction from true wind vector
        windHeading = radians(self.trueWind.direction)
        windSpeed = self.trueWind.speed

        # Get yaw angle
        boatQuat = self.odom.pose.pose.orientation
        boatHeading = (euler_from_quaternion([boatQuat.x, boatQuat.y, boatQuat.z, boatQuat.w])[2]) % 2*pi

        boatVelocity = self.odom.twist.twist.linear.x
        boatPosition = [self.odom.pose.pose.position.x, self.odom.pose.pose.position.y]
        goalPoint = [self.goal.x, self.goal.y]

        # Run the algorithm
        newHeading = degrees(heading(boatPosition, boatHeading, goalPoint, windSpeed, windHeading, self.beatingParam))

        # Send new heading
        newHeadingMsg = Float32(newHeading)
        self.newHeadingPub.publish(newHeadingMsg)

        # Don't operate on stale data
        self.odom = None
        self.trueWind = None


# Init node and spin
rospy.init_node("sailbot_nav")
nav = SailbotNav()
rospy.spin()

