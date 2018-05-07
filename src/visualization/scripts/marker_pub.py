#!/usr/bin/env python2

import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32
import tf
from tf.transformations import quaternion_from_euler

class MarkerPub:
    def __init__(self):
        self.commandHeadingSub = rospy.Subscriber("cmd_heading", Float32, self.updateCmdHeading, queue_size = 10)
        self.currentHeadingSub = rospy.Subscriber("curr_heading", Float32, self.updateCurrHeading, queue_size = 10)

        self.cmdHeadingMarker = Marker()
        self.cmdHeadingMarker.header.frame_id = "boat"
        self.cmdHeadingMarker.scale.x = 1
        self.cmdHeadingMarker.scale.y = 1
        self.cmdHeadingMarker.scale.z = 0.01
        self.cmdHeadingMarker.pose.position.x = 0
        self.cmdHeadingMarker.pose.position.y = 0
        self.cmdHeadingMarker.color.a = 1.0
        self.cmdHeadingMarker.color.r = 0.0
        self.cmdHeadingMarker.color.g = 1.0
        self.cmdHeadingMarker.color.b = 0.0
        self.cmdHeadingMarker.type = 0

        self.currHeadingMarker = Marker()
        self.currHeadingMarker.header.frame_id = "boat"
        self.currHeadingMarker.scale.x = 1
        self.currHeadingMarker.scale.y = 1
        self.currHeadingMarker.scale.z = 0.01
        self.currHeadingMarker.pose.position.x = 0
        self.currHeadingMarker.pose.position.y = 0
        self.currHeadingMarker.color.a = 1.0
        self.currHeadingMarker.color.r = 1.0
        self.currHeadingMarker.color.g = 0.0
        self.currHeadingMarker.color.b = 0.0
        self.currHeadingMarker.type = 0
        
        self.cmdHeadingMarkerPub = rospy.Publisher("cmd_heading_marker", Marker, queue_size=10)
        self.currHeadingMarkerPub = rospy.Publisher("curr_heading_marker", Marker, queue_size=10)

        self.listener = tf.TransformListener()

    def updateCmdHeading(self, newHeading):
        (translation, rotation) = self.listener.lookupTransform('odom', 'boat', 0)
        headingQuat = quaternion_from_euler(0, 0, radians(newHeading))
        self.cmdHeadingMarker.pose.orientation.x = rotation.x + headingQuat.x
        self.cmdHeadingMarker.pose.orientation.y = rotation.y + headingQuat.y
        self.cmdHeadingMarker.pose.orientation.z = rotation.z + headingQuat.z
        self.cmdHeadingMarker.pose.orientation.w = rotation.w + headingQuat.w

        self.cmdHeadingMarkerPub.publish(self.cmdHeadingMarker)

    def updateCurrHeading(self, newHeading):
        (translation, rotation) = self.listener.lookupTransform('boat', 'odom', 0)
        self.currHeadingMarker.pose.orientation.x = rotation.x
        self.currHeadingMarker.pose.orientation.y = rotation.y
        self.currHeadingMarker.pose.orientation.z = rotation.z
        self.currHeadingMarker.pose.orientation.w = rotation.w

        self.currHeadingMarkerPub.publish(self.currHeadingMarker)

rospy.init_node("visualization_markers")
node = MarkerPub()
rospy.spin()
