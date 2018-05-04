#!/usr/bin/env python2
#this part is related to the compiler part, don't worry about it
import cv2
import rospy
import numpy as np
from visualization.msg import BoatState

curState = BoatState()
curState.disabled = False
curState.autonomous = False
curState.transmittingROS = False

curState.navigation = False
curState.longDistance = False
curState.search = False
curState.stationKeeping = False

#rospy.set_param('event', 'search')
trueParam = rospy.get_param('event')

if trueParam =="navigation":
	curState.navigation = True
elif trueParam =="longDistance":
	curState.longDistance = True
elif trueParam =="search":
	curState.search = True
else:
	curState.stationKeeping = True 


rospy.init_node('camnode')			#are there a reason u name it camnode?
rate = rospy.Rate(1) # 10hz
publishedState = rospy.Publisher('curState', BoatState, queue_size = 15)	#might want to change the queue_size later


while not rospy.is_shutdown():
	publishedState.publish(curState)
	rate.sleep()

