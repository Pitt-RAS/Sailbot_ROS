#!/usr/bin/env python2

import sys
import numpy as np
import rospy
import math
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from Sailbot_ROS.msg import TrueWind
from tf.transformations import euler_from_quaternion, quaternion_from_euler


#Returns the angle between the two directions
def direction_difference(direction1, direction2):

	if (direction1 > direction2):
		angle = direction1 - direction2
	if (direction2 > direction1):
		angle = direction2 - direction1
		
	if (angle > 180):
		angle = angle - 180
		angle = 180 - angle
        print "angle difference: %d" % (angle)	
	return angle

#Returns 0 if apparent_wind_direction is left of course_over_ground, or 1 if it is right of course_over_ground
#The direction of apparent_wind_direction relative to course_over_ground is used to determine whether the 
#angle between AWD and TWD is to the right of AWD or the left of AWD
def left_right(apparent_wind_direction, course_over_ground):
    right_cog = course_over_ground + 180
    if (right_cog > 360):
        angle_remainder = right_cog - 360
        if (apparent_wind_direction > course_over_ground or apparent_wind_direction < angle_remainder):
            print "wind is right of cog"
            return 1
        else:
            print "wind is left of cog"
            return 0
    if (apparent_wind_direction > course_over_ground and apparent_wind_direction < right_cog):
        print "wind is right of cog"
        return 1
    else:
        print "wind is left of cog"
        return 0
    
#Use the Law of Cosines to find the magnitude of the True Speed Vector

def find_true_speed(angle, apparent_wind_speed, speed_over_ground):
    angle = np.deg2rad(angle)
    #bc_squared = b^2 + c^2
    bc_squared = (apparent_wind_speed*apparent_wind_speed) + (speed_over_ground*speed_over_ground) 
    
    #a_squared = bc_squared - 2*a*b*cos(angle)
    opposite_squared = bc_squared - 2*apparent_wind_speed*speed_over_ground*(np.cos(angle))

    print "true_speed is: %f" % (np.sqrt(opposite_squared))
    return np.sqrt(opposite_squared)
    
#Use the Law of Sines to return the angle between the True Wind vector and Apparent Wind Direction
	
def find_true_angle(true_wind_speed,awa_cog_angle, speed_over_ground):
    awa_cog_angle = np.deg2rad(awa_cog_angle)
    print "awa_rad: %f" % awa_cog_angle
    sin_awa = np.sin(awa_cog_angle)
    print "sin_awa: %f" % sin_awa
    true_wind_angle = np.arcsin((sin_awa*speed_over_ground) / true_wind_speed)
    print "true_wind_angle rad: %f" % true_wind_angle
    true_wind_angle = np.rad2deg(true_wind_angle)
    if (true_wind_angle < 0):
        true_wind_angle = true_wind_angle*-1
    print "true wind angle: %f" % (true_wind_angle)
    return true_wind_angle


def find_true_wind_direction(true_wind_angle, apparent_wind_direction, diff_angle):
    if (diff_angle == 0):
        true_wind_direction = apparent_wind_direction - true_wind_angle
        if (true_wind_direction < 0):
            true_wind_direction = true_wind_direction + 360

    else:
        true_wind_direction = apparent_wind_direction + true_wind_angle
        if (true_wind_direction > 360):
            true_wind_direction = true_wind_direction - 360
    print "true_wind_direction: %f" % (true_wind_direction)
    return true_wind_direction

class TrueWind:
    def __init__(self):
        self.apparent_wind_speed = 0
        self.apparent_wind_direction = 0
        self.speed_over_ground = 0
        self.course_over_ground = 0
        
        topic = "true_wind"
        pub = rospy.Publisher(topic, TrueWind.msg, queue_size = 10)
        self.trueWindPub = rospy.init_node("true_wind", anonymous = False)
        message = TrueWind()
        rospy.Subscriber("wind_direction", Int32, wind_direction_callback)
        rospy.Subscriber("wind_speed", Int32, wind_speed_callback)
        rospy.Subscriber("odom", Odometry, localization_callback)
        apparent_wind_direction = None

        def wind_direction_callback(self, new_apparent_wind_direction):
            self.apparent_wind_direction = new_apparent_wind_direction
        
        def wind_speed_callback(sef, new_apparent_wind_speed):
            self.apparent_wind_speed = new_apparent_wind_speed
            
        def localization_callback(self, new_odom):
            x = new_odom.twist.twist.linear.x
            y = new_odom.twist.twist.linear.y
            
            self.speed_over_ground = math.sqrt(math.pow(x, 2) + math.pow(y, 2))
            quat = new_odom.pose.pose.orientation
            heading = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2]
            self.course_over_ground = math.floor((math.pi / 180)*heading)
            
        def update(self):
            #angle between course over ground and apparent wind angle
            awa_cog_angle = direction_difference(apparent_wind_direction, course_over_ground)
            true_wind_speed = find_true_speed(awa_cog_angle, apparent_wind_speed, speed_over_ground)
            true_wind_angle = find_true_angle(true_wind_speed, awa_cog_angle, speed_over_ground)
            diff_angle = left_right(apparent_wind_direction, course_over_ground)
            true_wind_direction = find_true_wind_direction(true_wind_angle, apparent_wind_direction, diff_angle)
             
            
            print "The True Wind Speed is: %f \n and the True Wind Direction is: %f" % (true_wind_speed, true_wind_direction)


rate = rospy.Rate(rospy.get_param("~rate", 60))
while not rospy.is_shutdown():
    self.update()
    rate.sleep()


