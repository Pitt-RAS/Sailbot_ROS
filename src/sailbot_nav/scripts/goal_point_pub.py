#!/usr/bin/env python2
import rospy
from objective.msg import Goal


rospy.init_node("test_goal_point_sender")

goal = Goal()
goal.goalType = 0
goal.goalPoint.x = 4
goal.goalPoint.y = 0

goalPub = rospy.Publisher("/goal", Goal, queue_size=10)

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    goal.header.stamp = rospy.Time.now()
    goalPub.publish(goal)
    rate.sleep()

