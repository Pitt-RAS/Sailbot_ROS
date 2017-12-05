#!/usr/bin/env pythin

import rospy
from std_msgs.msg import Float32, Int32
from math import atan, pi, degrees, floor
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

goal_heading = 0.0
current_heading= 0.0
command_angle = 0
theta_e = 0.0
L = 0.6096 #distance from rudder to front wheels (2ft) in meters
K_p = 1 #proportional constant
V = 1 #velocity

class RudderCommandNode:
   def __init__(self):
      self.rudderAnglePub = rospy.Publisher('cmd_rudder_angle', Int32, queue_size=10)
      self.cmdHeadingSub = rospy.Subscriber("cmd_heading", Float32, self.cmd_callback)
      self.curHeadingSub = rospy.Subscriber("/imu", Imu, self.cur_callback)
      self.goal_heading = 0
      self.current_heading = 45  

   def cmd_callback(self, data):
      self.goal_heading = data.data

   def cur_callback(self, odom):
      temp = euler_from_quaternion((odom.orientation.x, odom.orientation.y, odom.orientation.z, odom.orientation.w))
      self.current_heading = degrees(temp[2])
      #if temp<0:
      #   current_heading = (temp*-1)+180
      #else:
      #   current_heading = temp
      #print(current_heading)

   def calc_cmd_heading(self,goal_heading, theta_e):
      return int(degrees(atan((2*pi*L*theta_e*k_e)/V)))

   def update(self):
      theta_e = self.current_heading - self.goal_heading
      print("commanding "+str(theta_e))
      theta_e *= 2
      self.rudderAnglePub.publish(Int32(theta_e+90))
      #command_angle = calc_cmd_heading(goal_heading, theta_e)



if __name__ == '__main__':
   rospy.init_node('rudder_command')
   rate = rospy.Rate(50)
   node = RudderCommandNode()
   while not rospy.is_shutdown():
      node.update()
      rate.sleep()
