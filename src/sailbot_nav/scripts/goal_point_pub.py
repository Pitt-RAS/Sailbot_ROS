#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import PointStamped


rospy.init_node("test_goal_point_sender")

p = PointStamped()
p.header.frame_id = "odom"

p.point.x = 4
p.point.y = 0

goalPub = rospy.Publisher("/goal", PointStamped, queue_size=10)

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    p.header.stamp = rospy.Time.now()
    goalPub.publish(p)
    rate.sleep()

