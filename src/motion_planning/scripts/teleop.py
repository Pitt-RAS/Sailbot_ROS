#!/usr/bin/env python2
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32

class TeleopNode:
    def __init__(self):
        self.rudderAnglePub = rospy.Publisher("cmd_heading", Int32, queue_size=10)
        self.js = rospy.Subscriber("/joy", Joy, self.update);

    def update(self, joy):
        rudderCmd = joy.axes[0]*90
        rudderCmd += 90
        self.rudderAnglePub.publish(rudderCmd)

rospy.init_node("teleop")

node = TeleopNode()

rospy.spin()



