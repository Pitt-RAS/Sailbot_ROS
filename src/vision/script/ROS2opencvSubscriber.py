import cv2
import rospy
import tf
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from matplotlib import pyplot as plt

img_test = cv2.imread('buoy2.jpeg')

def callback(data):		
	img = bridge.compressed_imgmsg_to_cv2(data, desired_encoding="passthrough")
	cv2.imshow('cv_img', img_test)
	cv2.waitKey(10)
	print(img)
	

bridge = CvBridge()
rospy.init_node('listener', anonymous=True)
receivIm = rospy.Subscriber("imagetopic", CompressedImage, callback)
rospy.spin()

	
