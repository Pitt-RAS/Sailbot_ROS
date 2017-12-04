#!/usr/bin/env python2
import rospy
from math import pi
from std_msgs.msg import Float32, Int32

# This node subscribes to the period of rotation
# published by the the wind sensor Arduino.
# It publishes the relative wind speed in radians/sec
# Currently no filtering is applied.

class WindspeedFilter:
    def __init__(self):
        self.windspeedSpeedPub = rospy.Publisher("/relative_wind_speed", Float32, queue_size=10)
        self.windspeedDtSub = rospy.Subscriber("/wind_sensor_tick_dt", Int32, self.updateWindSensorTick, queue_size=10)

    def updateWindSensorTick(self, tick):
        # dt in us to rps, then radians/sec
        radsSec = 0
        if ( tick.data != 0 ):
            radsSec = 1.0/(tick.data/1000.0)
        radsSec *= 2*pi
        self.windspeedSpeedPub.publish(Float32(radsSec))

rospy.init_node("windspeed_filter")
node = WindspeedFilter()
rospy.spin()

