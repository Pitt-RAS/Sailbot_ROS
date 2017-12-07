#!/usr/bin/env python2
import rospy
import tf
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker
from math import pi, sin, cos, atan2

# This node takes the raw relative wind direction in degrees,
# applies Carlos' exponential weighting implementation,
# and publishes the result in degrees
#


class WindDirectionNode:
    def __init__(self):
        self.relativeWindMarkerPub = rospy.Publisher("/relative_wind_marker", Marker, queue_size=10)
        self.windDirectionRawSub = rospy.Subscriber("/relative_wind_direction/raw", Int32, self.updateRelativeWind, queue_size=10)
        self.windDirectionPub = rospy.Publisher("/relative_wind_direction", Float32, queue_size=10)
        self.tfBroadcaster = tf.TransformBroadcaster()

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

        self.memory = 0.9
        self.sinweight = 0
        self.cosweight = 0
        self.sinval = 0
        self.cosval = 0

    def updateRelativeWind(self, windDegrees):
        val = (windDegrees.data/4096.0)*(2.0*pi)

        self.sinweight = self.memory*self.sinweight + 1;
        self.sinval = (1-1/self.sinweight)*self.sinval + (1/self.sinweight)*sin(val);

        self.cosweight = self.memory * self.cosweight + 1;
        self.cosval = (1-1/self.cosweight)*self.cosval + (1/self.cosweight)*cos(val)

        windAngle = atan2(self.sinval, self.cosval)

        self.windDirectionPub.publish(Float32(windAngle))

        relativeWindQat = tf.transformations.quaternion_from_euler(0, 0, windAngle)
        self.relativeWindMarker.pose.orientation.x = relativeWindQat[0]
        self.relativeWindMarker.pose.orientation.y = relativeWindQat[1]
        self.relativeWindMarker.pose.orientation.z = relativeWindQat[2]
        self.relativeWindMarker.pose.orientation.w = relativeWindQat[3]
        self.relativeWindMarkerPub.publish(self.relativeWindMarker)



rospy.init_node("wind_direction")
node = WindDirectionNode()
rospy.spin()
