#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from time import sleep
import numpy as np

class SailAngleNode:
  def __init__(self):
    self.sailAnglePub = rospy.Publisher('cmd_sail_angle', Int32, queue_size=100)
    self.cmdHeadingSub = rospy.Subscriber("/relative_wind_direction", Int32, self.wind_callback)
    self.wind = None
    self.sailAngle = 0

  def wind_callback(self, data):
    self.wind = data.data #0 to +/-180 (cw vs ccw)

  def update(self):
    if(self.wind is None):
      return

    if np.abs(self.wind) > 45: #avoids deadzone
      self.sailAngle = int(np.abs(self.wind)/2)

      if self.sailAngle > 70: #70 deg is max landsailer sail angle
        self.sailAngle = 70

    self.sailAnglePub.publish(Int32(self.sailAngle));
 
    print("wind " + str(self.wind))
    print("commanding sail to " + str(self.sailAngle))

if __name__ == '__main__':
  rospy.init_node('wind_to_sailangle')
  rate = rospy.Rate(50)
  node = SailAngleNode()
  while not rospy.is_shutdown():
    node.update()
    rate.sleep()

