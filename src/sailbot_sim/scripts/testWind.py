#!/usr/bin/env python2
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from geometry_msgs.msg import Vector3, Quaternion
from tf.transformations import quaternion_from_euler

rospy.init_node("wind_publisher")
speedPub = rospy.Publisher("wind_speed", Int32, queue_size=10)
dirPub = rospy.Publisher("wind_direction", Int32, queue_size=10)
odomPub = rospy.Publisher("odom", Odometry, queue_size=10)
speedMessage = 5
dirMessage = 90


odom = Odometry()
odom.header.stamp = rospy.Time.now()
odom.header.frame_id = "odom"

odom.pose.pose.position = Vector3(0, 0, 0)
headingQuat = quaternion_from_euler(0, 0, 0)
odom.pose.pose.orientation = Quaternion(headingQuat[0], headingQuat[1], headingQuat[2], headingQuat[3]) 

odom.child_frame_id = "base_link"

odom.twist.twist.linear = Vector3(5, 0, 0)
odom.twist.twist.angular = Vector3(0, 0, 0)


while not rospy.is_shutdown():
    speedPub.publish(speedMessage)
    dirPub.publish(dirMessage)
    odomPub.publish(odom)
    
    

