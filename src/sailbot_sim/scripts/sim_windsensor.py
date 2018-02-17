#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from math import atan2,sin,cos,pow,sqrt
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Provides a simulated wind sensor
# 
# Given a static true wind vector, this node will compute the relative 
# wind vector with robot velocity
#
# Subscribed topics:
# /odom
#
# Published topics: 
# /true_wind Vector3 - True wind as a Vector3
# /relative_wind Vector3 - Relative wind as a Vector3
# /true_wind_marker Marker - True wind for viewing in RViz
# /relative_wind_marker Marker - Relative wind for viewing in RViz (marker is in base_link frame)
#

class WindSensorSim:
    def __init__(self):
        # Setup static true wind vector
        self.trueWind = Vector3(rospy.get_param("/sim_wind_vector/x", 0), rospy.get_param("/sim_wind_vector/y", 0), 0)
        trueWindQat = quaternion_from_euler(0, 0, atan2(self.trueWind.y, self.trueWind.x)) 


        # Create true wind marker for rviz
        self.trueWindMarker = Marker()
        self.trueWindMarker.header.frame_id = "odom"
        self.trueWindMarker.scale.x = 1 
        self.trueWindMarker.scale.y = 0.1 
        self.trueWindMarker.scale.z = 0.01 
        self.trueWindMarker.pose.position.x = 0 
        self.trueWindMarker.pose.position.y = 0 
        self.trueWindMarker.pose.orientation.x = trueWindQat[0]
        self.trueWindMarker.pose.orientation.y = trueWindQat[1]
        self.trueWindMarker.pose.orientation.z = trueWindQat[2]
        self.trueWindMarker.pose.orientation.w = trueWindQat[3]
        self.trueWindMarker.color.a = 1.0 
        self.trueWindMarker.color.r = 0.0
        self.trueWindMarker.color.g = 1.0
        self.trueWindMarker.color.b = 0.0
        self.trueWindMarker.type = 0

        # Create relative wind marker for rviz
        self.relativeWindMarker = Marker()
        self.relativeWindMarker.header.frame_id = "base_link"
        self.relativeWindMarker.scale.x = 1 
        self.relativeWindMarker.scale.y = 0.1 
        self.relativeWindMarker.scale.z = 0.01 
        self.relativeWindMarker.pose.position.x = 0 
        self.relativeWindMarker.pose.position.y = 0 
        self.relativeWindMarker.color.a = 1.0 
        self.relativeWindMarker.color.r = 0.0
        self.relativeWindMarker.color.g = 1.0
        self.relativeWindMarker.color.b = 0.0
        self.relativeWindMarker.type = 0

        # self.trueWindPub = rospy.Publisher("/true_wind", Vector3, queue_size=10) 
        self.trueWindMarkerPub = rospy.Publisher("/true_wind_marker", Marker, queue_size=10)
        # self.relativeWindPub = rospy.Publisher("/relative_wind", Vector3, queue_size=10) 
        self.relativeWindSpeedPub = rospy.Publisher("/wind_speed", Int32, queue_size=10) 
        self.relativeWindDirectionPub = rospy.Publisher("/wind_direction", Int32, queue_size=10)
        self.relativeWindMarkerPub = rospy.Publisher("/relative_wind_marker", Marker, queue_size=10)
        self.odomSub = rospy.Subscriber("/odom", Odometry, self.update)

    # Update every time we get an odom update
    def update(self, odom):
        # Update relative wind vector
        quat = odom.pose.pose.orientation
        heading = -euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2]
        robot_velocity = odom.twist.twist.linear.x

        relative_wind_vector = [self.trueWind.x * cos(heading) - self.trueWind.y * sin(heading), 
                self.trueWind.x * sin(heading) + self.trueWind.y * cos(heading)]
        relative_wind_vector[0] -= robot_velocity
        relative_wind_speed = sqrt(pow(relative_wind_vector[0], 2) + pow(relative_wind_vector, 2))
        relative_wind_speed = math.floor(relative_wind_speed)
        relative_wind_direction = atan2(relative_wind_vector[1], relative_wind_vector[2])
        relative_wind_direction *= math.floor(180 / math.pi)
        if (relative_wind_direction < 0):
            relative_wind_direction = 360 + relative_wind_direction

        # Update orientation for relative wind marker
        relativeWindQat = quaternion_from_euler(0, 0, atan2(relative_wind_vector[1], relative_wind_vector[0])) 
        self.relativeWindMarker.pose.orientation.x = relativeWindQat[0]
        self.relativeWindMarker.pose.orientation.y = relativeWindQat[1]
        self.relativeWindMarker.pose.orientation.z = relativeWindQat[2]
        self.relativeWindMarker.pose.orientation.w = relativeWindQat[3]

        # Update topics
        self.relativeWindSpeedPub.publish(relative_wind_speed)
        self.relativeWindDirectionPub.publish(relative_wind_direction)
        # self.relativeWindPub.publish(Vector3(relative_wind_vector[0], relative_wind_vector[1], 0))
        self.relativeWindMarkerPub.publish(self.relativeWindMarker)
        # self.trueWindPub.publish(self.trueWind)
        self.trueWindMarkerPub.publish(self.trueWindMarker)

rospy.init_node("sim_windsensor")
windSensor = WindSensorSim()
rospy.spin()
