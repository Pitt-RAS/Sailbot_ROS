#!/usr/bin/env python

import cv2
import rospy
import tf
import numpy as np
import primOdom1 as po
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

def newROI(frame):
	frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	thresh1 = cv2.inRange(frame, np.array([0,150,50]), np.array([5,255,255]))
	thresh2 = cv2.inRange(frame, np.array([170,150,50]), np.array([180,255,255]))
	thresh = cv2.bitwise_or(thresh1, thresh2)
	h, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
	bigCon = [contour for contour in contours if len(contour) == max([len(c) for c in contours]) ]
	print bigCon
	if len(bigCon)==0:
		bigCon=[np.array([[[300,300]]])]
	x,y,w,h = cv2.boundingRect(bigCon[0])
	print x,y,w,h
	thresh = cv2.rectangle(cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR), (x,y), (x+w, y+h), (0,255,0), 2)
	roi = frame[y:y+h, x:x+h]
	hsvr = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
	histr = cv2.calcHist([hsvr], [0,1], None, [180,256], [0,180,0,256])
	newWindow = (x,y,w,h)
	return histr, newWindow, thresh


pose_message = PoseWithCovarianceStamped()
pose_buoy = PoseStamped()
rospy.init_node('camnode')
pub1 = rospy.Publisher('buoypos', PoseStamped, queue_size=50)
pub2 = rospy.Publisher('camtopic', PoseWithCovarianceStamped, queue_size=50)
pubIm = rospy.Publisher('imagetopic', CompressedImage, queue_size = 15)

bridge = CvBridge()

cap = cv2.VideoCapture(0)
ret, frame = cap.read()
imageDim = (len(frame[0]), len(frame))

'''
##################################################################################################################
instance of camera odometry updater. 
ARGS: buoy position, boat initial position, camera FOV in video mode (degrees), dimensions of frame e.g. (480,640)
##################################################################################################################
'''
buoyPosition = [10,10]
odometry = po.odom(buoyPosition, [0,0], 70, imageDim)
counter = 0;
term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 2 )
#Uncomment to remove "sanity check"
#histr, window1 = newROI(frame)
#window2 = window1

while not rospy.is_shutdown():
	ret, frame = cap.read()

	if counter%100==0:
		histr, window1, thresh = newROI(frame)
		window2 = window1

	if ret==True:
		hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
		prob = cv2.calcBackProject([hsv], [0,1], histr, [0,180,0,256],1)

		ret1, window1 = cv2.CamShift(prob, window1, term_crit)
		ret2, window2 = cv2.meanShift(prob, window2, term_crit)



		x,y,w,h = window2
		img2 = cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0) ,2)
		pts = cv2.boxPoints(ret1)

		# To-Do: combine both methods below
		odometry.poseVec(pts) #calc boat direction (also plots)
		position, angle = odometry.posePos(pts) #position is updated
		print "position = " + str(position)


		pose_buoy.header.frame_id = 'buoy'
		pose_buoy.header.stamp = rospy.Time.now()
		pose_buoy.pose = Pose( Point(buoyPosition[0],buoyPosition[1],0), Quaternion(0,0,0,1) )
		pub1.publish(pose_buoy)

		pose_message.header.frame_id = 'world'
		pose_message.header.stamp = rospy.Time.now()
		quatty = tf.transformations.quaternion_from_euler(0, 0, angle)
		pose_message.pose.pose = Pose( Point(position[0],position[1],0), Quaternion(*quatty) )
		pose_message.pose.covariance = [.1,0,0,0,0,0,  0,.1,0,0,0,0,   0,0,.1,0,0,0,   0,0,0,.1,0,0,   0,0,0,0,.1,0,    0,0,0,0,0,.1]
		pub2.publish(pose_message)

		pts = np.int0(pts)
		img2 = cv2.polylines(frame, [pts], True, (255,255,0),2)
		img_message = bridge.cv2_to_compressed_imgmsg(thresh)

		pubIm.publish(img_message)

		counter+=1

		key = cv2.waitKey(1) & 0xff
		if key==27:
			break
	else:
		break

cap.release()
