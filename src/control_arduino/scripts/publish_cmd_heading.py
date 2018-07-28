#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from visualization.msg import BoatState.msg

class PublishCmdHeading:
    def __init__(self):
        self.headingPub = rospy.Publisher('cmd_heading', Float32, queue_size=100)
        self.headingPreset1 = rospy.get_param('heading1')
        self.headingPreset2 = rospy.get_param('heading2')
        self.presetSub = rospy.Subscriber('boat_state', BoatState)

    def presetCallback(self, boatState):
        self.headingPreset = boatState.presetOne

    def update(self):
        headingMsg = Float32()
        if self.headingPreset:
            headingMsg.data = self.headingPreset1
        else:
            headingMsg.data = self.headingPreset2
        self.headingPub.publish(Float32(headingMsg))


if __name__ == '__main__':
  rospy.init_node('cmd_heading_publisher')
  rate = rospy.Rate(5)
  node = PublishCmdHeading()
  while not rospy.is_shutdown():
    node.update()
    rate.sleep()
