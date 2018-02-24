#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32, Int32
from math import atan, pi, degrees, floor, radians
from nav_msgs.msg import Odometry
#from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

L = 0.6096 #distance from rudder to front wheels (2ft) in meters
K_p = 1.0 #proportional constant
V = 1 #velocity

class RudderCommandNode:
  def __init__(self):
    self.rudderAnglePub = rospy.Publisher('cmd_rudder_angle', Int32, queue_size=10)
    self.cmdHeadingSub = rospy.Subscriber("cmd_heading", Float32, self.cmd_callback)
    self.curHeadingSub = rospy.Subscriber("/odometry/filtered", Odometry, self.cur_callback)
    #self.curHeadingSub = rospy.Subscriber("/imu", Imu, self.cur_callback)
    self.goal_heading = None
    self.current_heading = None

  def cmd_callback(self, data):
    self.goal_heading = data.data

  def cur_callback(self, odom):
    temp_raw = euler_from_quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)
    #temp_raw = euler_from_quaternion((odom.orientation.x, odom.orientation.y, odom.orientation.z, odom.orientation.w))
    temp_heading = degrees(temp_raw[2])
    if(temp_heading<0):
      temp_heading = 360 + temp_heading
    self.current_heading = temp_heading

  def calc_cmd_angle(self,goal_heading, theta_e):
    return int(degrees(atan((2*pi*L*radians(theta_e)*K_p)/V)))

  def update(self):
    if self.goal_heading is None or self.current_heading is None:
      return

    theta_e = self.goal_heading - self.current_heading
    #if(theta_e>180):
      #theta_e = 360 - theta_e

    #command_angle = self.calc_cmd_angle(self.goal_heading, theta_e)
    #self.rudderAnglePub.publish(Int32(command_angle))

    new_rudder_heading = theta_e+90
    if(new_rudder_heading<0):
      new_rudder_heading = 0
    if(new_rudder_heading>180):
      new_rudder_heading = 180

    self.rudderAnglePub.publish(Int32(K_p * new_rudder_heading))

    print("heading "+str(self.current_heading))
    print("commanding "+str(K_p * new_rudder_heading))

if __name__ == '__main__':
  rospy.init_node('rudder_command')
  rate = rospy.Rate(50)
  node = RudderCommandNode()
  while not rospy.is_shutdown():
    node.update()
    rate.sleep()
