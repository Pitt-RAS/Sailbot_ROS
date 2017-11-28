#!/usr/bin/env python2
import rospy
import tf
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3, Quaternion
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

    if angleBetweenWind > 43 and angleBetweenWind < 151:
        return 0.3
    return 0

class OdomSim:
    def __init__(self):
        self.windVector = Vector3(0,0,0)

        self.x = 0
        self.y = 0
        self.heading = 0
        self.lastTime = rospy.Time.now()

        self.angleSetpointSubscriber = rospy.Subscriber("cmd_heading", Float32, self.updateAngleSetpoint, queue_size=10)
        self.windVectorSubscriber = rospy.Subscriber("true_wind", Vector3, self.updateWindVector, queue_size=10)
        self.odomPublisher = rospy.Publisher("odom", Odometry, queue_size=10)
        self.resetPoseService = rospy.Service("sim_reset_pose", ResetPose, self.resetPose)
        self.tfBroadcaster = tf.TransformBroadcaster()

        self.lock = threading.Lock()


    def updateAngleSetpoint(self, angle):
        self.lock.acquire()
        self.heading = radians(boundAngle(angle.data, 360.0))
        self.lock.release()

    def updateWindVector(self, newWind):
        self.lock.acquire()
        self.windVector = newWind
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

        angleBetweenWind = boundAngle(degrees(abs(self.heading - atan2(self.windVector.y, self.windVector.x))), 180.0)
        velocity = boatPolarFunction(sqrt(self.windVector.x**2 + self.windVector.y**2), angleBetweenWind)

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


