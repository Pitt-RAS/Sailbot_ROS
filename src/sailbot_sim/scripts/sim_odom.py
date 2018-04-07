#!/usr/bin/env python2
import rospy
import tf
import numpy as np
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import Vector3, Quaternion
from sailbot_sim.msg import TrueWind
from nav_msgs.msg import Odometry
from sailbot_sim.srv import ResetPose, ResetPoseResponse
from math import sqrt, sin, cos, atan2, pi, radians, degrees
import threading

def boundAngle(theta, limit):
    while theta > limit:
        theta -= limit
    while theta < 0:
        theta += limit
    return theta

def boatPolarFunction(windVelocity, angleBetweenWind):
    if windVelocity == 0:
        return 0

    if angleBetweenWind > radians(43) and angleBetweenWind < radians(151):
        return 0.3
    return 0

class OdomSim:
    def __init__(self):
        self.lock = threading.Lock()
        self.lock.acquire()

        self.windVector = Vector3(rospy.get_param("/sim_wind_vector/x", 0), rospy.get_param("/sim_wind_vector/y", 0), 0)

        self.x = 0
        self.y = 0
        self.heading = 0
        self.commandHeading = 0
        self.omega_max = 0
        self.lastTime = rospy.Time.now()

        rate = rospy.get_param("~rate", 60)
        radSec = rospy.get_param("~maxomega", 3.14)
        self.dTheta = radSec / rate

        self.angleSetpointSubscriber = rospy.Subscriber("cmd_heading", Float32, self.updateAngleSetpoint, queue_size=10)
        self.odomPublisher = rospy.Publisher("odometry/filtered", Odometry, queue_size=10)
        self.resetPoseService = rospy.Service("sim_reset_pose", ResetPose, self.resetPose)

        self.wind_direction_pub = rospy.Publisher("wind_direction", Float32, queue_size=10)
        self.wind_speed_pub = rospy.Publisher("wind_speed", Float32, queue_size=10)

        self.tfBroadcaster = tf.TransformBroadcaster()

        self.lock.release()

    def updateAngleSetpoint(self, angle):
        with self.lock:
            self.commandHeading = boundAngle(radians(angle.data), 2*pi)

    def resetPose(self, req):
        with self.lock:
            self.x = 0
            self.y = 0
            self.heading = 0
        return ResetPoseResponse()

    def update(self):
        with self.lock:
            now = rospy.Time.now()
            dt = (now-self.lastTime).to_sec()
            self.lastTime = now

            angleBetweenWind = boundAngle(abs(self.heading - atan2(self.windVector.y, self.windVector.x)), pi)

            velocity = boatPolarFunction(sqrt(self.windVector.x**2 + self.windVector.y**2), angleBetweenWind)

            if self.heading != self.commandHeading:
                error = self.commandHeading-self.heading
                if abs(error) > pi:
                    error = -error

                if abs(error) < self.dTheta:
                    self.heading = self.commandHeading
                else:
                    self.heading += self.dTheta*np.sign(error)
                    self.heading = boundAngle(self.heading, 2*pi)

            self.y += velocity * dt * sin(self.heading)
            self.x += velocity * dt * cos(self.heading)

            headingQuat = quaternion_from_euler(0, 0, self.heading)

            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = "odom"

            odom.pose.pose.position = Vector3(self.x, self.y, 0)
            odom.pose.pose.orientation = Quaternion(headingQuat[0], headingQuat[1], headingQuat[2], headingQuat[3])

            odom.child_frame_id = "boat"

            odom.twist.twist.linear = Vector3(velocity*cos(self.heading), velocity*sin(self.heading), 0)
            odom.twist.twist.angular = Vector3(0, 0, 0)

            self.odomPublisher.publish(odom)

            self.tfBroadcaster.sendTransform((self.x, self.y, 0),
                                            headingQuat,
                                            now,
                                            "boat",
                                            "odom");

            relative_wind_vector = self.calculateRelativeWindVector(odom.twist.twist.linear, self.windVector, self.heading)
            relative_wind_speed = Float32(sqrt(relative_wind_vector.x**2 + relative_wind_vector.y**2))
            relative_wind_direction = Float32(degrees(atan2(relative_wind_vector.y, relative_wind_vector.x)))

            self.wind_speed_pub.publish(relative_wind_speed)
            self.wind_direction_pub.publish(relative_wind_direction)

    def calculateRelativeWindVector(self, boatVelocity, trueWind, heading):
        boatVelocity = np.array([boatVelocity.x, boatVelocity.y])
        trueWind = np.array([trueWind.x, trueWind.y])
        apparentWind = trueWind - boatVelocity
        apparentWind = np.matmul(apparentWind, np.array([[cos(heading), -sin(heading)], [sin(heading), cos(heading)]]));
        return Vector3(apparentWind[0], apparentWind[1], 0)

rospy.init_node("sim_odom")
rate = rospy.Rate(rospy.get_param("~rate", 60))
sim = OdomSim()

while not rospy.is_shutdown():
    sim.update()
    rate.sleep()


