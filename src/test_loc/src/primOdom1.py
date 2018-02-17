import numpy as np
import cv2
import math


class odom:

	def __init__(self, buoyPosition, boatPosition, FOVdegrees, imageDimensions):
		self.buoyPos = buoyPosition
		self.position = boatPosition
		self.cameraFOV =  FOVdegrees*math.pi/180
		self.imageDim = imageDimensions
		self.k = 0
		self.kBuild = 0
		self.captureKMode = True
		self.captureKCount = 0
		boatDir = -1
		buoyDir = -1

	def poseVec(self, boxPts):	
		m = cv2.moments(boxPts)
		if m['m00']==0:
			m['m00']=1
		cx = int(m['m10']/m['m00'])
		deltaDir = self.cameraFOV - cx*self.cameraFOV/self.imageDim[0]
		self.buoyDir = np.arctan2(self.buoyPos[1]-self.position[1], self.buoyPos[0]-self.position[0])
		self.boatDir = self.buoyDir-deltaDir
		v = [math.cos(self.boatDir), math.sin(self.boatDir)]


	def posePos(self, boxPts):
		dist = ( (self.buoyPos[0]-self.position[0])**2 + (self.buoyPos[1]-self.position[1])**2 )**.5
		print "dist = " + str(dist)
		perim = cv2.arcLength(boxPts, True)
		if perim<=0 or math.isnan(perim):
			perim = 1.0
		print "perim = " + str(perim)
		if self.captureKMode==True: #k not defined yet
			self.kBuild += ( -1*dist/(math.log(perim/500)) )/5
			self.captureKCount += 1
		if self.captureKCount < 5:
			self.k = -1*dist/(math.log(perim/500)) #don't move boat until you know K
		else: 
			self.k = self.kBuild
			self.captureKMode = False
		print "k = " + str(self.k)
		newDist = -1*self.k*math.log(perim/500)
		print "newDist = " + str(newDist)

		#directionV = [math.cos((direction+self.buoyDir)/2), math.sin((direction+self.buoyDir)/2)]	
		directionV = [math.cos(self.boatDir), math.sin(self.boatDir)]	
		Bx = self.buoyPos[0]
		By = self.buoyPos[1]
		x0 = self.position[0]
		y0 = self.position[1]
		vx = directionV[0]
		vy = directionV[1]
		r = newDist
		a = vx**2 + vy**2
		b = 2*( (x0-Bx)*vx + (y0-By)*vy )
		c = (x0-Bx)**2 + (y0-By)**2 - r**2
	
		try:
			t = min(  (-1*b + (b**2-4*a*c)**.5)/(2*a), (-1*b - (b**2-4*a*c)**.5)/(2*a)   )
		except:
			t = .1

		newPos = [ self.position[0] + t*.5*directionV[0] , self.position[1] + t*.5*directionV[1] ]

		self.position = newPos

		return newPos, self.boatDir
