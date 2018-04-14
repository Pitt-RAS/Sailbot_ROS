#!/usr/bin/env python2
import rospy
import rostest
import unittest
import time
import numpy as np
import tf
from math import atan2, degrees, pi
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from sensors.msg import TrueWind
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped

class GPSOdomTest(unittest.TestCase):

    def test_gps_utm(self):
        fixPub = rospy.Publisher("fix", NavSatFix, queue_size=1)

        # Block until the publisher is setup
        while fixPub.get_num_connections() is 0:
            pass

        fix = NavSatFix()
        fix.latitude = 40.443256
        fix.longitude = -79.952530

        fixPub.publish(fix)
        time.sleep(0.1)

        # Move the robot
        fix.latitude = 40.443000
        fix.longitude = -79.952530

        fixPub.publish(fix)
        time.sleep(0.1)

        odom = rospy.wait_for_message("/odometry/gps/raw", Odometry)
        self.assertAlmostEqual(odom.pose.pose.position.x, 0.33703618752770126)
        self.assertAlmostEqual(odom.pose.pose.position.y, -28.416440724395216)

    def test_imu(self):
        testAngle = pi/4

        imuPub = rospy.Publisher("imu", Imu, queue_size=1)

        # Block until the publisher is setup
        while imuPub.get_num_connections() is 0:
            pass

        headingQuat = quaternion_from_euler(0, 0, pi/4)

        imuMsg = Imu()
        imuMsg.orientation.x = headingQuat[0]
        imuMsg.orientation.y = headingQuat[1]
        imuMsg.orientation.z = headingQuat[2]
        imuMsg.orientation.w = headingQuat[3]
        imuPub.publish(imuMsg)

        time.sleep(0.1)

        odom = rospy.wait_for_message("/odometry/gps/raw", Odometry)

        heading = euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])
        self.assertAlmostEqual(heading[2], testAngle)

    def test_gps_vel(self):
        velPub = rospy.Publisher("vel", TwistStamped, queue_size=1)

        # Block until the publisher is setup
        while velPub.get_num_connections() is 0:
            pass

        twistMsg = TwistStamped()
        twistMsg.twist.linear.x = 1
        twistMsg.twist.linear.y = 2
        velPub.publish(twistMsg)

        time.sleep(0.1)

        odom = rospy.wait_for_message("/odometry/gps/raw", Odometry)
        self.assertAlmostEqual(odom.twist.twist.linear.x, twistMsg.twist.linear.x)
        self.assertAlmostEqual(odom.twist.twist.linear.y, twistMsg.twist.linear.y)

    def test_odom_transform(self):
        listener = tf.TransformListener()
        fixPub = rospy.Publisher("fix", NavSatFix, queue_size=1)
        imuPub = rospy.Publisher("imu", Imu, queue_size=1)

        # Block until the publisher is setup
        while imuPub.get_num_connections() is 0 or fixPub.get_num_connections() is 0:
            pass

        headingQuat = quaternion_from_euler(0, 0, pi/4)

        imuMsg = Imu()
        imuMsg.orientation.x = headingQuat[0]
        imuMsg.orientation.y = headingQuat[1]
        imuMsg.orientation.z = headingQuat[2]
        imuMsg.orientation.w = headingQuat[3]
        imuPub.publish(imuMsg)

        fix = NavSatFix()
        fix.latitude = 40.443256
        fix.longitude = -79.952530

        fixPub.publish(fix)
        time.sleep(0.1)
        tfTime = rospy.Time.now()
        time.sleep(0.1)
        fixPub.publish(fix)

        listener.waitForTransform("odom", "boat", tfTime, rospy.Duration(4))
        (trans,rot) = listener.lookupTransform('odom', 'boat', tfTime)

        # Should be zeroed at first position
        self.assertEqual(trans[0], 0)
        self.assertEqual(trans[1], 0)

        # Rotation passed through from IMU
        self.assertAlmostEqual(rot[0], imuMsg.orientation.x)
        self.assertAlmostEqual(rot[1], imuMsg.orientation.y)
        self.assertAlmostEqual(rot[2], imuMsg.orientation.z)
        self.assertAlmostEqual(rot[3], imuMsg.orientation.w)

        # Move the robot
        fix.latitude = 40.443000
        fix.longitude = -79.952530

        fixPub.publish(fix)
        time.sleep(0.1)
        tfTime = rospy.Time.now()
        time.sleep(0.1)
        fixPub.publish(fix)

        listener.waitForTransform("odom", "boat", tfTime, rospy.Duration(4))
        (trans,rot) = listener.lookupTransform("odom", "boat", tfTime)

        # Should have moved
        self.assertAlmostEqual(trans[0], 0.33703618752770126)
        self.assertAlmostEqual(trans[1], -28.416440724395216)

        # Rotation passed through from IMU
        self.assertAlmostEqual(rot[0], imuMsg.orientation.x)
        self.assertAlmostEqual(rot[1], imuMsg.orientation.y)
        self.assertAlmostEqual(rot[2], imuMsg.orientation.z)
        self.assertAlmostEqual(rot[3], imuMsg.orientation.w)

    def test_utm_transform(self):
        listener = tf.TransformListener()
        fixPub = rospy.Publisher("fix", NavSatFix, queue_size=1)

        # Block until the publisher is setup
        while fixPub.get_num_connections() is 0:
            pass

        fix = NavSatFix()
        fix.latitude = 40.443256
        fix.longitude = -79.952530

        fixPub.publish(fix)
        time.sleep(0.1)
        tfTime = rospy.Time.now()
        time.sleep(0.1)
        fixPub.publish(fix)

        listener.waitForTransform("utm", "odom", tfTime, rospy.Duration(4))
        (trans,rot) = listener.lookupTransform("utm", "odom", tfTime)

        # Should be zeroed at first position
        self.assertAlmostEqual(trans[0], 588831.7070436851)
        self.assertAlmostEqual(trans[1], 4477482.9612701265)

        # Rotation for this frame is always 0
        self.assertAlmostEqual(rot[0], 0)
        self.assertAlmostEqual(rot[1], 0)
        self.assertAlmostEqual(rot[2], 0)
        self.assertAlmostEqual(rot[3], 1)

        # Move the robot
        fix.latitude = 40.443000
        fix.longitude = -79.952530

        fixPub.publish(fix)
        time.sleep(0.1)
        tfTime = rospy.Time.now()
        time.sleep(0.1)
        fixPub.publish(fix)

        listener.waitForTransform("utm", "odom", tfTime, rospy.Duration(4))
        (trans,rot) = listener.lookupTransform("utm", "odom", tfTime)

        # utm->odom should never change
        self.assertAlmostEqual(trans[0], 588831.7070436851)
        self.assertAlmostEqual(trans[1], 4477482.9612701265)


rospy.init_node("gps_odom_test")
rostest.rosrun('gps_odom', 'gps_odom_test', GPSOdomTest)

