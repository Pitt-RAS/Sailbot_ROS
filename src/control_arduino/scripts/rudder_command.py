#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Int32
from math import atan, pi, degrees, floor, radians
from nav_msgs.msg import Odometry
#from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import numpy as np

K_p = 1.0 #proportional constant

L = 0.6096 #distance from rudder to front wheels (2ft) in meters
V = 1 #velocity

class RudderCommandNode:
  def __init__(self):
    self.rudderAnglePub = rospy.Publisher('cmd_rudder_angle', Int32, queue_size=10)
    self.cmdHeadingSub = rospy.Subscriber("cmd_heading", Float32, self.cmd_callback)
    self.curHeadingSub = rospy.Subscriber("/odometry/gps/raw", Odometry, self.cur_callback)
    #self.curHeadingSub = rospy.Subscriber("/imu", Imu, self.cur_callback)
    self.goal_heading = None
    self.current_heading = None

  def cmd_callback(self, data):
    self.goal_heading = data.data

  def cur_callback(self, odom):
    temp_raw = euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])
    #temp_raw = euler_from_quaternion([odom.orientation.x, odom.orientation.y, odom.orientation.z, odom.orientation.w])
    temp_heading = degrees(temp_raw[2])
    if(temp_heading<0): #turns 0 to 180 (ccw) and 0 to -180 (cw) into 0-360
      temp_heading = 360 + temp_heading
    self.current_heading = temp_heading

  def update(self):
    if self.goal_heading is None or self.current_heading is None:
      return

    theta_e = self.goal_heading - self.current_heading
    
    if np.abs(theta_e) > 180: #fixes issue with crossing the 0
      if(theta_e > 0):
        theta_e = theta_e - 360
      else:
        theta_e = theta_e + 360

    #command_angle = self.calc_cmd_angle(self.goal_heading, theta_e)
    #self.rudderAnglePub.publish(Int32(command_angle))

    new_rudder_angle = (K_p * theta_e) + 90

    self.rudderAnglePub.publish(Int32(new_rudder_angle))

    print("heading "+str(self.current_heading))
    print("commanding "+str(new_rudder_angle))

if __name__ == '__main__':
  rospy.init_node('rudder_command')
  rate = rospy.Rate(50)
  node = RudderCommandNode()
  while not rospy.is_shutdown():
    node.update()
    rate.sleep()
