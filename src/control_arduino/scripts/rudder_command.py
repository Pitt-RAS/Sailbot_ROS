#!/usr/bin/env pythin

import rospy
from std_msgs.msg import Float32, Int32
from math import atan, pi, degrees, floor
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

L = 0.6096 #distance from rudder to front wheels (2ft) in meters
K_p = 1 #proportional constant
V = 1 #velocity

class RudderCommandNode:
   def __init__(self):
      self.rudderAnglePub = rospy.Publisher('cmd_rudder_angle', Int32, queue_size=10)
      self.cmdHeadingSub = rospy.Subscriber("cmd_heading", Float32, self.cmd_callback)
      self.curHeadingSub = rospy.Subscriber("/odometry/filtered", Odometry, self.cur_callback)
      self.goal_heading = None
      self.current_heading = None

   def cmd_callback(self, data):
      self.goal_heading = data.data

   def cur_callback(self, odom):
      temp = euler_from_quaternion((odom.PoseWithCovariance.Pose.Quaternion.x, odom.PoseWithCovariance.Pose.Quaternion.y, odom.PoseWithCovariance.Pose.Quaternion.z, odom.PoseWithCovariance.Pose.Quaternion.w))
      self.current_heading = degrees(temp[2])

   def calc_cmd_angle(self,goal_heading, theta_e):
      return int(degrees(atan((2*pi*L*theta_e*K_p)/V)))

   def update(self):
      if self.goal_heading or self.current_heading is None:
          return

      theta_e = self.goal_heading - self.current_heading
      if(theta_e>180):
          theta_e = theta_e - 360
      print("commanding "+str(theta_e))
      #theta_e *= 2
      #self.rudderAnglePub.publish(Int32(theta_e+90))
      command_angle = self.calc_cmd_angle(self.goal_heading, theta_e)
      self.rudderAnglePub.publish(Int32(command_angle))

if __name__ == '__main__':
   rospy.init_node('rudder_command')
   rate = rospy.Rate(50)
   node = RudderCommandNode()
   while not rospy.is_shutdown():
      node.update()
      rate.sleep()
