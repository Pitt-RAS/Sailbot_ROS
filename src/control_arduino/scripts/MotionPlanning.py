import rospy
from std_msgs import Float64

class MotionPlanner:
	def __init__(self, critAngle):
		self.critAngle = critAngle
		self.windDir = 0
		self.waypointDir = 0
		rospy.init_node('MotionPlanner')
		self.pub = rospy.Publisher('Heading', Float64, queue_size = 10)
		rospy.Subscriber("WaypointDirection", Float64, self.updateWaypoint)
		rospy.Subscriber("WindDirection", Float64, self.updateWind)
		
	
	def updateWaypoint(self, newWaypointDir):
		self.waypointDir = newWaypointDir.data
		heading = self.calculateHeading()
		self.publishHeading(heading)
		
	def upateWind(self, newWindDir):
		self.windDir = newWindDir.data
	
	def calculateHeading(self):
		heading = self.waypointDir
		targetWindAngle = self.parseAngle(self.waypointDir - self.windDir)
		if targetWindAngle < self.critAngle:
			if waypointDir > self.windDir:
				heading = self.critAngle + self.windDir
			else
				heading = self.windDir - self.critAngle
		return heading % 360
		
	def parseAngle(self, angle):
		angle = math.abs(angle)
		if angle > 180:
			return 180 - angle
		else:
			return angle
		
	def publishHeading(self):
		self.pub.publish(heading)
		
		
	def main():
		listener()