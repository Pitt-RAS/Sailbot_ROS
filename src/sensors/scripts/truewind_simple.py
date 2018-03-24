#!/usr/bin/env python2
import rospy
import numpy as np
import threading
from math import sin, cos, atan2, radians, degrees

from std_msgs.msg import Float32, Int32
from nav_msgs.msg import Odometry
from sailbot_sim.msg import TrueWind
from tf.transformations import euler_from_quaternion

def boundAngle(theta, limit):
    while theta > limit:
        theta -= limit
    while theta < 0:
        theta += limit
    return theta

class TrueWindNode:
    def __init__(self):
        self._lock = threading.Lock()
        self._odom = rospy.Subscriber("/odometry/filtered", Odometry, self._update_odom)
        self.wind_direction_sub = rospy.Subscriber("wind_direction", Int32, self._update_wind_direction)
        self.wind_speed_sub = rospy.Subscriber("wind_speed", Int32, self._update_wind_speed)
        self.true_wind_pub = rospy.Publisher("true_wind", TrueWind, queue_size=10)

        self._boat_velocity_vector = None
        self._wind_direction = None
        self._wind_speed = None
        self._boat_heading = None

    def _update_odom(self, odom):
        with self._lock:
            self._boat_velocity_vector = np.array([odom.twist.twist.linear.x, odom.twist.twist.linear.y])
            quat = odom.pose.pose.orientation
            self._boat_heading = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2]

    def _update_wind_direction(self, wind_direction):
        with self._lock:
            self._wind_direction = radians(wind_direction.data)

    def _update_wind_speed(self, wind_speed):
        with self._lock:
            self._wind_speed = wind_speed.data


    def update(self):
        with self._lock:
            if self._boat_velocity_vector is not None and self._wind_direction is not None and self._wind_speed is not None:
                true_wind_vector = self._wind_speed * np.array([cos(self._wind_direction), sin(self._wind_direction)])

                true_wind_vector += self._boat_velocity_vector
                true_wind_vector = np.matmul(true_wind_vector, np.array([[cos(self._boat_heading), sin(self._boat_heading)],
                                                                    [-sin(self._boat_heading), cos(self._boat_heading)]]))

                wind_msg = TrueWind()

                wind_msg.direction = boundAngle(degrees(atan2(true_wind_vector[1], true_wind_vector[0])), 360)
                wind_msg.speed = np.linalg.norm(true_wind_vector)
                self.true_wind_pub.publish(wind_msg)


rospy.init_node("true_wind", anonymous = False)
rate = rospy.Rate(rospy.get_param("~rate", 60))
node = TrueWindNode()
while not rospy.is_shutdown():
    node.update()
    rate.sleep()

