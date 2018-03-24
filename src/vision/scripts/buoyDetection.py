import cv2
import rospy
import tf
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from matplotlib import pyplot as plt

def detectBlob(name):				#blob will check the far away situation for the buoy
	im = cv2.imread(name, cv2.IMREAD_GRAYSCALE) 
	params = cv2.SimpleBlobDetector_Params()
	ver = (cv2.__version__).split('.')
	if int(ver[0]) < 3 :
	    detector = cv2.SimpleBlobDetector(params)
	else : 
	    detector = cv2.SimpleBlobDetector_create(params)

	keypoints = detector.detect(im)

	return keypoints

def newROI(frame, name):
	keypoints = detectBlob(name)		
	img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	
	ret,shapeThresh = cv2.threshold(img,127,255,1)
	h1,shapeContours,hierarchy = cv2.findContours(shapeThresh,1,2)

	frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	thresh = cv2.inRange(frame, np.array([0,150,30]), np.array([23,255,255]))
	
	h, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
	count = 0
	target=[]
	targetFound = 0
	for contour in contours:
		x,y,w,h = cv2.boundingRect(contour)
		
		if w!=1 and h!=1 and (w/h<1.5 or h/w<1.5):	#check with keypoints here, if the detected points has h/w or w/h ratio of 1.5
			keyptLen = len(keypoints)
			ptCount = 0
			while(ptCount<keyptLen):
				if abs(keypoints[ptCount].pt[0]-x)<10 and abs(keypoints[ptCount].pt[1]-y)<10 :
					target = [contour]
					targetFound = 1
				ptCount = ptCount +1			
	if len(target)==0:			#situation when the bouy is close to the sailboat
		target = [contour for contour in contours if len(contour) == max([len(c) for c in contours]) ]
		targetFound = 1	

	if len(target)==0:			#not in close range nor the far range, the bouy dones't exist	
		target=[np.array([[[300,300]]])]
		targetFound = 0
	x,y,w,h = cv2.boundingRect(target[0])
	print x,y,w,h
	if targetFound ==1:
		center = [ x+w/2, y+h/2  ]		#the center of the target, need it for the sailbot to sail to	
	else:
		center = [ -1, -1 ]			#we can tell if the target if found or not by checking the center

	histr = 0
	newWindow = 0
	if targetFound ==1:
		thresh = cv2.rectangle(cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR), (x,y), (x+w, y+h), (0,255,0), 2)
		roi = frame[y:y+h, x:x+w]
		hsvr = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
		histr = cv2.calcHist([hsvr], [0,1], None, [180,256], [0,180,0,256])
		newWindow = (x,y,w,h)

	return histr, newWindow, thresh, center,targetFound
	
name = 'buoy7.jpeg'
img = cv2.imread(name)

histr, window1, thresh, center,targetFound = newROI(img,name)
print center
plt.subplot(121),plt.imshow(img,cmap = 'gray')
plt.title('Original Image'), plt.xticks([]), plt.yticks([])
if targetFound ==1:
	plt.subplot(122),plt.imshow(thresh,cmap = 'gray')
	plt.title('Edge Image'), plt.xticks([]), plt.yticks([])

plt.show()



