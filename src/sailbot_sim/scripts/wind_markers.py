#!/usr/bin/env python2

import rospy
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler
from math import radians
from std_msgs.msg import Int32
from sailbot_sim.msg import TrueWind

class WindMarkerPub:
    def __init__(self):
        self.trueWindSub = rospy.Subscriber("/true_wind", TrueWind, self.updateTrueWind)
        self.relativeWindSub = rospy.Subscriber("/wind_direction", Int32, self.updateRelativeWind)

        # Create true wind marker for rviz
        self.trueWindMarker = Marker()
        self.trueWindMarker.header.frame_id = "odom"
        self.trueWindMarker.scale.x = 1
        self.trueWindMarker.scale.y = 0.1
        self.trueWindMarker.scale.z = 0.01
        self.trueWindMarker.pose.position.x = 0
        self.trueWindMarker.pose.position.y = 0
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

        self.trueWindMarkerPub = rospy.Publisher("/true_wind_marker", Marker, queue_size=10)
        self.relativeWindMarkerPub = rospy.Publisher("/relative_wind_marker", Marker, queue_size=10)

    def updateTrueWind(self, true_wind):
        trueWindQuat = quaternion_from_euler(0, 0, radians(true_wind.direction))

        self.trueWindMarker.pose.orientation.x = trueWindQuat[0]
        self.trueWindMarker.pose.orientation.y = trueWindQuat[1]
        self.trueWindMarker.pose.orientation.z = trueWindQuat[2]
        self.trueWindMarker.pose.orientation.w = trueWindQuat[3]

        self.trueWindMarkerPub.publish(self.trueWindMarker)

    def updateRelativeWind(self, angle):
        relativeWindQuat = quaternion_from_euler(0, 0, radians(angle.data))

        self.relativeWindMarker.pose.orientation.x = relativeWindQuat[0]
        self.relativeWindMarker.pose.orientation.y = relativeWindQuat[1]
        self.relativeWindMarker.pose.orientation.z = relativeWindQuat[2]
        self.relativeWindMarker.pose.orientation.w = relativeWindQuat[3]

        self.relativeWindMarkerPub.publish(self.relativeWindMarker)


rospy.init_node("wind_markers")
node = WindMarkerPub()
rospy.spin()
