#!/usr/bin/env python2

from Tkinter import *
import rospy
from sensors.msg import TrueWind
from geometry_msgs.msg import PointStamped

class ShoreVisualize:
    def __init(self):
            
        self.currentState = State()
        self.battery = -1
        self.windDirection = -1
        self.windSpeed = -1
        self.cmdHeading = -1
        self.currHeading = -1
        self.cmdSail = -1
        self.currSail = -1
        self.cmdRudder = -1
        self.currRudder = -1
        self.goalPoint = -1
        self.goalDir = -1
        self.goalType = -1

        self.eventLabel
  
        self.stateSub = rospy.Subscriber("state", BoatState, self.updateState, queue_size = 10)
        self.batterySub = rospy.Subscriber("battery_voltage", Float32, self.updateBattery, queue_size = 10)
        self.trueWindSub = rospy.Subscriber("true_wind", TrueWind, self.updateWind, queue_size = 10)
        self.commandHeadingSub = rospy.Subscriber("cmd_heading", Float32, self.updateCmdHeading, queue_size = 10)
        self.currentHeadingSub = rospy.Subscriber("curr_heading", Float32, self.updateCurrHeading, queue_size = 10)
        self.commandSailAngleSub = rospy.Subscriber("cmd_sail_angle", Int32, self.updateCmdSailAngle, queue_size = 10)
        self.currentSailAngleSub = rospy.Subscriber("curr_sail_angle", Int32, self.updateCurrSailAngle, queue_size = 10)
        self.commandRudderAngleSub = rospy.Subscriber("cmd_sail_angle", Int32, self.updateCmdRudderAngle, queue_size = 10)
        self.currentRudderAngleSub = rospy.Subscriber("curr_rudder_angle", Int32, self.updateCurrRudderAngle, queue_size = 10)
        self.goalSub = rospy.Subscriber("goal", Goal, self.updateGoal, queue_size = 10)
        
        self.goalPointPub = rospy.Publisher("goal_point", PointStamped, queue_size = 10)
        self.goalDirectionPub = rospy.Publisher("goal_direction", Int32, queue_size = 10)

        root = Tk()
        self.app = TkApp(master = root)
        self.app.mainloop()
        root.destroy() 
        
    def updateState(self, newState):
        self.currentState.disabled = newState.disabled
        self.currentState.autonomous = newState.autonomous
        self.currentState.transmittingROS = newState.transmittingROS
        if newState.navigation:
            self.currentState.event = "Navigation" 
        if newState.stationKeeping:
            self.currentState.event = "StationKeeping"
        if newState.search:
            self.currentState.event = "Search"
        if newState.longDistance:
              self.currentState.event = "LongDistance"

    def updateBattery(self, newBattery):
        self.battery = newBattery
        
    def updateWind(self, newWind):
        self.windDirection = newWind.direction
        self.windSpeed = newWind.speed
        
    def updateCmdHeading(self, newCmdHeading):
        self.cmdHeading = newCmdHeading
        
    def updateCurrHeading(self, newCurrHeading):
        self.currHeading = newCurrHeading
        
    def updateCmdSailAngle(self, newCmdSailAngle):
        self.cmdSail = newCmdSailAngle
        
    def updateCurrSailAngle(self, newCurrSailAngle):
        self.currSail = newCurrSailAngle
        
    def updateCmdRudderAngle(self, newCmdRudderAngle):
        self.cmdRudder = newCmdRudderAngle
        
    def updateCurrRudderAngle(self, newCurrRudderAngle):
        self.currRudder = newCurrRudderAngle
        
    def updateGoal(self, newGoal):
        self.goalType = newGoal.goalType
        self.goalPoint = newGoal.goalPoint
        # TODO: stamp the goal point for rvis
        self.goalDir = newGoal.Direction
        
    def update(self):
        stateString = " "
        objectiveString = " "
        transmissionString = " "
        
        if self.currentState.trasmittingROS:
            transmissionString = "Transmitting ROS"
        else:
            transmissionString = "Transmitting Teensy only"
        
        if self.goalType == 0:
            goalPointStamped = PointStamped()
            goalPointStamped.header.stamp = rospy.Time.now()
            goalPointStamped.point = self.goalPoint
            goalPointPub.publish(goalPointStamped)
        elif self.goalType == 1:
            goalDirectionPub.publish(goalDirection)
        self.app.updateTKWidgets()           
        
        class State:  
            self.disabled = False
            self.autonomous = False
            self.transmittingROS = False
            self.event = ""

    class TkApp(Frame): 
        
        def createTkWidgets(self):
            self.quitButton = Button(self)
            self.quitButton["text"] = "QUIT"
            self.quitButton["fg"] = "red"
            self.quitButton["command"] = self.quit
            self.quitButton.pack({"side": "left"})
            
            self.eventLabel = Label(self)
            self.eventLabel['text'] = " "
            self.eventLabel.pack({'side': 'left'})
            
        def updateTkWidgets(self):
            self.eventLabel['text'] = self.event

        def __init__(self, master=None):
            Frame.__init__(self, master)
            self.pack()
            self.createWidgets()


rospy.init_node("shore_visualization")
rate = rospy.Rate(rospy.get_param("~rate", 60))
node = ShoreVisualize()

while not rospy.is_shutdown():
    node.update()
    rate.sleep()
            
            
                
            
