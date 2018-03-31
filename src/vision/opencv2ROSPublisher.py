import cv2
import rospy
import tf
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from matplotlib import pyplot as plt




name = 'buoy2.jpeg'
img = cv2.imread(name)

pose_message = PoseWithCovarianceStamped()
pose_buoy = PoseStamped()
rospy.init_node('camnode')			#are there a reason u name it camnode?
rate = rospy.Rate(1) # 10hz
pubIm = rospy.Publisher('imagetopic', CompressedImage, queue_size = 15)
bridge = CvBridge()

while not rospy.is_shutdown():

	img_message = bridge.cv2_to_compressed_imgmsg(img,dst_format='jpg')
	pubIm.publish(img_message)

	rate.sleep()
#cap = cv2.VideoCapture(0)
#ret, frame = cap.read()
#imageDim = (len(frame[0]), len(frame))

