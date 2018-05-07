import cv2
import rospy
import tf
import numpy as np
from visualization.msg import BoatState

#img_test = cv2.imread('buoy2.jpeg')

def callback(data):		
	print data
	#img = bridge.compressed_imgmsg_to_cv2(data, desired_encoding="passthrough")
	#cv2.imshow('cv_img', img_test)
	#cv2.waitKey(10)
	#print(img)
	

#bridge = CvBridge()
rospy.init_node('listener', anonymous=True)
receivIm = rospy.Subscriber("curState", BoatState, callback)
rospy.spin()

	
