import rospy
from std_msgs import Float64
class gpsBuoy():
	self.currentDir
	self.buoyDir
	self.buoyGPS
	
	def gpsConversion(gps):
		self.gps
		//gotta do this


	def buoyAngle():
	
		if buoyGPS != null:
			self.buoyDistance(angle)
		angle = currentDir-buoyDir
		
		angle = math.abs(angle)
		if angle > 180:
			return 180 - angle
		else:
			return angle
		
			
	def buoyDistance():
		//gotta do this
	
			
	def updateCurrentDir(newCurrentDir):
		self.currentDir = newCurrentDir
		
	def upateBuoyDir(newBuoyDir):
		self.buoyDir = newBuoyDir
				
	def listener():
		rospy.init_node('gpsBuoy')
		rospy.Subscriber("CurrentDirection", Float64, self.updateCurrentDir)
		rospy.Subscriber("BuoyDirection", Float64, self.updateBuoyDir)
		
		rospy.spin()
		self.calculateHeading()
		
		pub = rospy.Publisher('Heading', Float64, queue_size = 10)
		
	def main():
		listener()
	
