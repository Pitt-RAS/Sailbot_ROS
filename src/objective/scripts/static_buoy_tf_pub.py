#!/usr/bin/env python2
import rospy
import tf
from math import atan2
from tf.transformations import quaternion_from_euler

rospy.init_node("buoy_tf")

class BuoyTFPublisher:
    def __init__(self):
        self._tf_broadcaster = tf.TransformBroadcaster()
        self._first_buoy = rospy.get_param("buoy1")
        self._second_buoy = rospy.get_param("buoy2")

        angle_diff = atan2(self._first_buoy[1] - self._second_buoy[1], self._first_buoy[0] - self._second_buoy[0])
        self.quat = quaternion_from_euler(0, 0, angle_diff)

        rospy.logwarn("Starting static TF buoy publisher: (x,y) offset is ({}, {}) angle difference is {}".format(self._first_buoy[0], self._first_buoy[1], angle_diff))

    def update(self):
        self._tf_broadcaster.sendTransform((self._first_buoy[0], self._first_buoy[1], 0),
                self.quat,
                rospy.Time.now(),
                "buoy",
                "utm")


node = BuoyTFPublisher()
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    rate.sleep()
    node.update()


