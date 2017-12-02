import sys
import re
import math
#import numpy as np	#might have issues with this part

#this program is designed to calculate the route for sailing the sailboat upwind, it will consider the wind and the wave directions and the speed(this is just the prototype which cocatergorized the wind and wave speed together.) and 
#in this program, we only considering turing 1 angle during the upwind sail, we assume the direction between the current sailboat location and the destination as the positive y-axis and its perpendicular direction as the x-axis

#all the functions below are for the initialization purpose
alpha = 0
boatPosi = [0,0]
targetPosi = [0,0]
boatSpeed = 0
def getBoatPosi():
	global boatPosi
	boatPosi = [ 3,3 ]
	
def getTargetPosi():
	global targetPosi
	targetPosi = [ 5,5 ]

def getBoatSpeed():
	global boatSpeed
	boatSpeed = 5
	
def getT():
	getTargetPosi()
	getBoatPosi()
	row = (targetPosi[ 0 ]- boatPosi[ 0 ])** 2 
	column = (targetPosi[ 1 ]- boatPosi[ 1 ])** 2 
	t =[row,column]
	
	return t
	
def getHysterFactor():
	HFactor = 1+ 5/( ( (targetPosi[ 0 ]- boatPosi[ 0 ])** 2 ) + ( (targetPosi[ 1 ]- boatPosi[ 1 ])** 2 ) ) #we assume that 5 is the beating parameter
	return HFactor
	
def getTildaWAbs():		
	return 30#honestly, i currently have no clue how to calculate this constant, so i will just assume it to be 30

def getWindAngle():#not super sure where to use the alpha in the formula, i am just assuming it is the delta alpha
	return 20	
	
def getWindSpeed():#assume the speed is 15 km/h and change it later
	return 15
	
def calculateVhep(WSpeedABS, angleSum):
	vectorVhep=[];
	if angleSum> 43 and angleSum<151:
		return [20,5]
	else:
		return [0,0]
		
def rightHandOptimum():	
	alpha = 0		#alpha 
	vtrMax = 0	#vtrMax
	thildaBMaxR = 30#getTildaWAbs() #you need to update the number into getTildaWAbs when you finish the function
	thildaBMaxR = whileLoopInrightHandOptimum(alpha, vtrMax, thildaBMaxR) # the while loop is the part 
	print(thildaBMaxR)
	return thildaBMaxR

def calculateVt(VbHype):
	Vt = 0
	for i, j in zip(VbHype, getT()):
		#print("i is ",i," j is ",j)
		tempSum = i*j
		Vt = Vt +tempSum
	return 	Vt
	
def whileLoopInrightHandOptimum(alpha, vtrMax, thildaBMaxR):	
	while(alpha <= 180):
		VbHype = calculateVhep(abs( getWindSpeed() ), thildaBMaxR+alpha)
		
		Vtr = calculateVt( VbHype )
		#print("vtr is ",Vtr)
		if Vtr > vtrMax:
			vtrMax = Vtr #replace value as the formula said
			thildaBMaxR = thildaBMaxR + alpha		#might have issues here, but will change it later
		if alpha>180:
			break	#break the loop so that we can go to the next function
		else:
			alpha = alpha + getWindAngle() #honestly, not sure about delta alpha, so i assume it as a reasonable number
	return 	thildaBMaxR


def leftHandOptimum():	
	alpha = 0		#alpha 
	vtlMax = 0	#vtrMax
	thildaBMaxL = 20 #getTildaWAbs() #you need to update the number into getTildaWAbs when you finish the function
	thildaBMaxL = whileLoopInLeftHandOptimum(alpha, vtlMax, thildaBMaxL)
	print(thildaBMaxL)
	return thildaBMaxL
	
def whileLoopInLeftHandOptimum(alpha, vtlMax, thildaBMaxL):	
	while(alpha <= 180):
		VbHype = calculateVhep(abs( getWindSpeed() ), thildaBMaxL+alpha)
		Vtl = calculateVt( VbHype )
		if Vtl > vtlMax:
			vtlMax = Vtl #replace value as the formula said
			thildaBMaxL = thildaBMaxL + alpha		#might have issues here, but will change it later
		if alpha>180:
			break	#break the loop so that we can go to the next function
		else:
			alpha = alpha + getWindAngle() #honestly, not sure about delta alpha, so i assume it as a reasonable number
	return 	thildaBMaxL

def getVtmaxR():#need to change it later, getting the data from somewhere
	return 20
	
def getVtmaxL():
	return 10
	
def choosingNewDirection( thildaBMaxL,thildaBMaxR ):
	VtmaxR = getVtmaxR()
	VtmaxL = getVtmaxL()
	print("fysterfactor is ",getHysterFactor())
	if abs(thildaBMaxL )< abs( thildaBMaxR ):
		#print("fysterfactor is ",getHysterFactor())
		if VtmaxR *getHysterFactor() < VtmaxL:    #it is supposed to be getHysterFactor(), but we can't deal with the vector calculation yet.....
			return thildaBMaxL
		else:
			return thildaBMaxR
	else:
		if VtmaxL *getHysterFactor() < VtmaxR:
			return thildaBMaxR
		else :
			return thildaBMaxL
			
thildaBMaxR = rightHandOptimum()
thildaBMaxL = leftHandOptimum()
angle = choosingNewDirection( thildaBMaxL,thildaBMaxR )
print("the angle is ", angle)