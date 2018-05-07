#!/usr/bin/env python2

import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32

class MarkerPub
    def __init__(self):
        self.commandHeadingSub = rospy.Subscriber("cmd_heading", Float32, self.updateCmdHeading, queue_size = 10)
        self.currentHeadingSub = rospy.Subscriber("curr_heading", Float32, self.updateCurrHeading, queue_size = 10)

        self.cmdHeadingMarker = Marker()
        self.cmdHeadingMarker.header.frame_id = "boat"
        self.cmdHeadingMarker.scale.x = 1
          
