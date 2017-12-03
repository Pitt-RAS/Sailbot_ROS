#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import numpy as np

sailangle = 0
pub = rospy.Publisher('cmd_sail_angle', Float64, queue_size=100) #publisher

def calculate(wind): # -180 < wind < 180
    global sailangle
    if np.abs(wind) > 45: #if not in deadzone, calculate and publish sailangle
        sailangle = np.abs(wind)/2
        pub.publish(Float64(sailangle)) #only publishes if the wind is within good angle

def wind_to_sailangle():
  rospy.init_node('wind_to_sailangle',anonymous=True)
  rospy.Subscriber("chatter", Float64, calculate) #subscribe to wind sensor publisher
  rospy.spin() #if publishing using the bottom while loop, comment this out
  #rate = rospy.Rate(100) #100hz
  #while not rospy.is_shutdown():
    #pub.publish(Float64(sailangle))
    #rate.sleep()

if _name_ == '_main_':
    try:
        wind_to_sailangle()
    except rospy.ROSInterruptException:
        pass
