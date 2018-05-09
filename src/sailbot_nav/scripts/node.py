#!/usr/bin/env python2

import rospy
from math import atan2, sqrt, degrees, radians, pi
from tf.transformations import euler_from_quaternion
import tf
from algorithm import heading
import StationKeeping

from std_msgs.msg import Int32, Float32
from nav_msgs.msg import Odometry
from objective.msg import Goal
from sensors.msg import TrueWind
from geometry_msgs.msg import Vector3, PointStamped


class SailbotNav:
    def __init__(self):
        self.newHeadingPub = rospy.Publisher("/cmd_heading", Float32, queue_size=10)
        self.odomSub = rospy.Subscriber("/odometry/filtered", Odometry, self.updateOdom)
        self.trueWindSub = rospy.Subscriber("/true_wind", TrueWind, self.updateTrueWind)
        self.goalSub = rospy.Subscriber("/goal", Goal, self.updateGoal)

        self.beatingParam = rospy.get_param("~beating_parameter", 5)

        self.listener = tf.TransformListener()

        self.odom = None
        self.trueWind = None
        self.goalPoint = None
        self.goalType = 0
        self.goalDir = 0
        self.startStationKeeping = false

        # Might want to make this a param at some point
        # Width of the station keeping box if one is not specified
        self.defaultBoxWidth = 10

    def updateOdom(self, odom):
        self.odom = odom

    def updateTrueWind(self, trueWind):
        self.trueWind = trueWind

    def updateGoalPoint(self, goal):
        if self.goalType != 2 and goal.goalType == 2:
            self.goalType = goal.goalType
            self.startStationKeeping = true
            if goal.useBox:
                stationaryBox = [goal.box1, goal.box2, goal.box3, goal.box4]
            else:
                boatPosX = self.odom.pose.pose.position.x
                boatPosY =  self.odom.pose.pose.position.y
                # Boat position in the utm frame to find permanent box coordinates
                (trans, rot) = self.listener.lookupTransform('odom', 'utm', 0)
                boatPosX += trans.x
                boatPosY += trans.y
                corner1 = Point()
                corner1.x = boatPosX + self.defaultBoxWidth/2
                corner1.y = boatPosY + self.defaultBoxWidth/2
                corner1.z = 0
                corner2 = Point()
                corner2.x = boatPosX + self.defaultBoxWidth/2
                corner2.y = boatPosY - self.defaultBoxWidth/2
                corner2.z = 0
                corner3 = Point()
                corner3.x = boatPosX - self.defaultBoxWidth/2
                corner3.y = boatPosY - self.defaultBoxWidth/2
                corner3.z = 0
                corner4 = Point()
                corner4.x = boatPosX - self.defaultBoxWidth/2
                corner4.y = boatPosY + self.defaultBoxWidth/2
                corner4.z = 0
                stationaryBox = [corner1, corner2, corner3, corner4]
            self.stationKeeping = StationKeeping(stationaryBox)             

        self.goalPoint = goal.goalPoint
        self.goalDir = goal.goalDirection

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
        
        # Goal is to sail to a point
        if self.goalType == 0:
            goalPoint = [self.goalPoint.x, self.goalPoint.y]

            # Run the algorithm
            newHeading = degrees(heading(boatPosition, boatHeading, goalPoint, windSpeed, windHeading, self.beatingParam))

        
        # Goal is to sail in a direction
        elif self.goalType == 1:
            # TODO: implement this so you can sail into the wind
            newHeading = self.goalDir
        
        # Goal is to stay in place
        elif self.goalType == 2:
            (trans, rot) = self.listener.lookupTransform('odom', 'utm', 0)
            boatUTM[0] = boatPosition[0] + trans.x
            boatUTM[1] = boatPosition[1] + trans.y
            goalPoint = stationKeeping.getGoal(boatUTM, boatHeading, windSpeed, windHeading)
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

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    nav.update()
    rate.sleep()

