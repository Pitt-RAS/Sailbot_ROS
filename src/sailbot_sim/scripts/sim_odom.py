#!/usr/bin/env python2
import rospy
import tf
import numpy as np
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3, Quaternion
from Sailbot_ROS.msg import TrueWind
from nav_msgs.msg import Odometry
from sailbot_sim.srv import ResetPose, ResetPoseResponse
from math import sqrt, sin, cos, atan2, pi, radians 
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

        self.windVector = Vector3(0,0,0)

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
        # self.windVectorSubscriber = rospy.Subscriber("true_wind", Vector3, self.updateWindVector, queue_size=10)
        self.windSubscriber = rospy.Subscriber("true_wind", TrueWind, self.updateWind, queue_size=10)
        self.odomPublisher = rospy.Publisher("odom", Odometry, queue_size=10)
        self.resetPoseService = rospy.Service("sim_reset_pose", ResetPose, self.resetPose)
        self.tfBroadcaster = tf.TransformBroadcaster()

        self.lock.release()

    def updateAngleSetpoint(self, angle):
        self.lock.acquire()
        self.commandHeading = boundAngle(angle.data, 2*pi)
        self.lock.release()

    def updateWind(self, newWind):
        self.lock.acquire()
        wind = newWind.speed*[cos((pi / 180)*newWind.direction), sin((pi / 180)*newWind.direction), 0]
        self.windVector = Vector3(newWind)
        self.lock.release()

    def resetPose(self, req):
        self.lock.acquire()
        self.x = 0
        self.y = 0
        self.heading = 0
        self.lock.release()
        return ResetPoseResponse()
    
    def update(self):
        self.lock.acquire()
        now = rospy.Time.now()
        dt = (now-self.lastTime).to_sec()
        self.lastTime = now

        
        angleBetweenWind = boundAngle(abs(self.heading - atan2(self.windVector.y, self.windVector.x)), pi)
        velocity = boatPolarFunction(sqrt(self.windVector.x**2 + self.windVector.y**2), angleBetweenWind)

        if self.heading != self.commandHeading:
            error = self.commandHeading-self.heading
            if abs(error) > pi:
                error = -error

            if abs(error-self.dTheta) < 0.00001:
                self.heading = self.commandHeading
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

        odom.child_frame_id = "base_link"

        odom.twist.twist.linear = Vector3(velocity, 0, 0)
        odom.twist.twist.angular = Vector3(0, 0, 0)

        self.odomPublisher.publish(odom)

        self.tfBroadcaster.sendTransform((self.x, self.y, 0),
                                        headingQuat,
                                        now,
                                        "base_link",
                                        "odom");
        self.lock.release()

rospy.init_node("sim_odom")
rate = rospy.Rate(rospy.get_param("~rate", 60))
sim = OdomSim()

while not rospy.is_shutdown():
    sim.update()
    rate.sleep()


