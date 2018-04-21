#!/usr/bin/env python2
import rospy, sys
from geometry_msgs.msg import PointStamped, Point

class BuoyPublisher:
    def __init__(self, number):
        self._bouy_pub = rospy.Publisher("buoy/{}".format(number), PointStamped, queue_size=10)
        self._bouy_msg = PointStamped()
        self._bouy_msg.header.frame_id = rospy.get_param("~frame", "utm")

        buoy = rospy.get_param("buoy{}".format(number))
        self._bouy_msg.point = Point(buoy[0], buoy[1], 0)

        rospy.logwarn("Starting static buoy publisher for buoy #{}".format(number))

    def update(self):
        self._bouy_msg.header.stamp = rospy.Time.now()
        self._bouy_pub.publish(self._bouy_msg)

if len(sys.argv) < 2:
    print("Usage: static_buoy_pub.py {id}")
else:
    rospy.init_node("buoy_pub", anonymous=True)
    node = BuoyPublisher(int(sys.argv[1]))
    rate = rospy.Rate(rospy.get_param("~rate", 10))
    while not rospy.is_shutdown():
        node.update()
        rate.sleep()

