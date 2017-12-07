#!/usr/bin/env python2
import rospy
from math import pi
from std_msgs.msg import Float32, Int32
import threading

# This node subscribes to the period of rotation
# published by the the wind sensor Arduino.
# It publishes the relative wind speed in radians/sec

class WindspeedFilter:
    def __init__(self):
        self.windspeedSpeedPub = rospy.Publisher("/relative_wind_speed", Float32, queue_size=10)
        self.windspeedDtSub = rospy.Subscriber("/wind_sensor_tick", Int32, self.updateWindSensorTick, queue_size=10)

        self.lastWindUpdate = rospy.Time.now()

        defaultTime = (rospy.Time.now(), 0)
        self.periodBuffer = [defaultTime, defaultTime, defaultTime, defaultTime]
        self.periodBufferIdx = 0

        self.lock = threading.Lock()

    def updateWindSensorTick(self, tick):
        self.lock.acquire()
        self.periodBuffer[self.periodBufferIdx] = (rospy.Time.now(), tick.data)
        self.periodBufferIdx = (self.periodBufferIdx+1) % 4
        self.lock.release()

    def update(self):
        self.lock.acquire()

        now = rospy.Time.now()
        velocityEstimate = 0
        consideredPoints = 0

        for p in self.periodBuffer:
            if (now-p[0]).to_sec() < 3:
                velocityEstimate += p[1]
                consideredPoints += 1

        if consideredPoints != 0 and velocityEstimate != 0:
            velocityEstimate /= consideredPoints
            # ms -> sec
            velocityEstimate /= 1000.0
            # period -> hz
            velocityEstimate = 1.0/velocityEstimate

        self.windspeedSpeedPub.publish(Float32(velocityEstimate))
        self.lock.release()

rospy.init_node("windspeed_filter")
node = WindspeedFilter()
rate = rospy.Rate(1)
while not rospy.is_shutdown():
    node.update()
    rate.sleep()

