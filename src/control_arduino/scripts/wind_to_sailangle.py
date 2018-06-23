#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, Float32


class SailAngleNode:
  def __init__(self):
    self.sailAnglePub = rospy.Publisher('cmd_sail_angle', Int32, queue_size=100)
    self.cmdHeadingSub = rospy.Subscriber("/relative_wind_direction", Float32, self.wind_callback)
    self.windAngle = None
    self.sailAngle = 0

  def wind_callback(self, windAngle):
    # relative_wind_direction is 0 deg when wind comes from right of boat (~east)
    # Store as 0 deg when wind comes from directly ahead
    self.windAngle = (windAngle.data - 270) % 360
    
    # Only care about theta, angle of wind with 0 deg axis
    if self.windAngle >= 180:
        self.windAngle = 360 - self.windAngle

  def update(self):
    if self.windAngle is None:
      return

    if self.windAngle >= 45: #avoids deadzone
      self.sailAngle = self.wind / 2

      if self.sailAngle >= 70: #70 deg is max landsailer sail angle
        self.sailAngle = 70

    self.sailAnglePub.publish(Int32(self.sailAngle));
 
if __name__ == '__main__':
  rospy.init_node('wind_to_sailangle')
  rate = rospy.Rate(50)
  node = SailAngleNode()
  while not rospy.is_shutdown():
    node.update()
    rate.sleep()

