import cv2
import rospy
#import tf
import numpy as np
#from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose, Point, Quaternion
#from sensor_msgs.msg import CompressedImage
#from cv_bridge import CvBridge
#from matplotlib import pyplot as plt

class BoatState(object):
	def __init__(self):	
		self.disabled = False
		self.autonomous = False
		self.transmittingROS = False
		self.navigation = False
		self.longDistance = False
		self.search = False
		self.stationKeeping = False
	def changeVarable(self,varName):
		self.disabled = False
curState = BoatState()

rospy.set_param('trueParam', 'navigation')
trueParam = rospy.get_param('trueParam')

for attri in curState.__dict__.keys():
	if attri == trueParam:
		curState.attri = True


rospy.init_node('camnode')			#are there a reason u name it camnode?
rate = rospy.Rate(1) # 10hz
publishedState = rospy.Publisher('curState', BoatState)


while not rospy.is_shutdown():
	publishedState.publish(curState)
	rate.sleep()

