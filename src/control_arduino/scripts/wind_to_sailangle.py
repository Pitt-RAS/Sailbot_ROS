#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32,Int32
from math import degrees
from time import sleep
import numpy as np

sailangle = 0
pub = rospy.Publisher('cmd_sail_angle', Int32, queue_size=100) #publisher

def calculate(wind): # -180 < wind < 180
    global sailangle
    sailangle = degrees(wind.data)
rospy.init_node('wind_to_sailangle',anonymous=True)
rospy.Subscriber("/relative_wind_direction", Float32, calculate) #subscribe to wind sensor publisher

# Limit update rate to the Arduino
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    wind = sailangle
    if np.abs(wind) > 45: #if not in deadzone, calculate and publish sailangle
        sailangle = np.abs(wind)/2
        if sailangle > 80:
            sailangle=80
        print("commanding ", sailangle)
        pub.publish(Int32(sailangle)) #only publishes if the wind is within good angle
    else:
        print("in deadzone")
    rate.sleep()

