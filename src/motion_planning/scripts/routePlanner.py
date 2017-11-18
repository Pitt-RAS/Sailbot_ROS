import sys
import re
import math

#this program is designed to calculate the route for sailing the sailboat upwind, it will consider the wind and the wave directions and the speed(this is just the prototype which cocatergorized the wind and wave speed together.) and 
#in this program, we only considering turing 1 angle during the upwind sail, we assume the direction between the current sailboat location and the destination as the positive y-axis and its perpendicular direction as the x-axis

shipSpeed = 30 #we also assume the speed of the ship won't change 
totalDist = 50000 

def getTempXspeed( speedOfCurrent, shipDirection, currentDirection ):
	tempXSpeed = shipSpeed*math.sin( shipDirection ) + speedOfCurrent* math.sin(currentDirection)
	return tempXSpeed

def getTempYspeed( speedOfCurrent, shipDirection, currentDirection ):
	tempYSpeed = shipSpeed*math.cos( shipDirection ) + speedOfCurrent* math.sin(currentDirection)
	return tempYSpeed

def getDistanceLeft( passedDist ):
	distanceLef = totalDist - passedDist
	return distanceLef

def getDistancePassed( speedOfCurrent, shipDirection, currentDirection, time ):#calculate the distance passed assuming the angle and the speed didn't change
	xSpeed = getTempXspeed( speedOfCurrent, shipDirection, currentDirection )
	ySpeed = getTempYspeed( speedOfCurrent, shipDirection, currentDirection )
	distancePassed = math.sqrt( xSpeed*xSpeed + ySpeed*ySpeed )* time
	return distancePassed

angle = 30
speedOfCurrent = 12
currentDirection = 20
time = 1
distanceLeft = totalDist
while distanceLeft >= 0:
	passedDist = getDistancePassed( speedOfCurrent, angle , currentDirection,time  )
	distanceLeft = getDistanceLeft( passedDist )
	time = time + 1 
	
	if distanceLeft > totalDist/2:
		angle = angle/2
		speedOfCurrent = speedOfCurrent +30
	#else:
	#	angle = angle + 5
	#	speedOfCurrent = speedOfCurrent/2
	
	print(distanceLeft, time)

