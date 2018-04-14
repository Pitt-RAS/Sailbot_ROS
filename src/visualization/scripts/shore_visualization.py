#!/usr/bin/env python2

from Tkinter import *
import threading
import rospy
from sensors.msg import TrueWind
from geometry_msgs.msg import PointStamped
from visualization.msg import BoatState
from std_msgs.msg import Float32, Int32
from objective.msg import Goal

class ShoreVisualize:
    def __init__(self):
            
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

        self.transmissionString = 'Transmission not initialized'
        self.stateString = 'Boat state not initialized'
        self.eventString = 'Event not initialized'
        self.operationalString = 'Operational not initialized'

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
        self.app = self.TkApp(master = root)
      
    def updateState(self, newState):
        self.currentState.disabled = newState.disabled
        self.currentState.autonomous = newState.autonomous
        self.currentState.transmittingROS = newState.transmittingROS
       
        if self.currentState.autonomous:
            self.stateString = "Autonomous Navigation"
        else:
            self.stateString = "Remote Control"
        if self.currentState.disabled:
            self.operationalString = "Boat Disabled"
        else: 
            self.operationalString = " "
        if self.currentState.transmittingROS:
            self.transmissionString = "Transmitting ROS"
        else:
            self.transmissionString = "Transmitting Teensy only"
        if newState.navigation:
            self.eventString = "Navigation" 
        if newState.stationKeeping:
            self.currentState.event = "StationKeeping"
        if newState.search:
            self.eventString = "Search"
        if newState.longDistance:
              self.eventString = "LongDistance"

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
        # todo: stamp the goal point for rvis
        self.goalDir = newGoal.Direction
        
    def update(self):
        # print('Getting to the update')
        if self.goalType == 0:
            goalPointStamped = PointStamped()
            goalPointStamped.header.stamp = rospy.Time.now()
            goalPointStamped.point = self.goalPoint
            goalPointPub.publish(goalPointStamped)
        elif self.goalType == 1:
            goalDirectionPub.publish(goalDirection)
        self.app.updateStateLabels(self.eventString, self.stateString, self.transmissionString, self.operationalString)           

    class TkApp(Frame): 
        def updateStateLabels(self, eventString, stateString, transmissionString, operationalString):
            #  print('Updating widgets')
            self.eventLabel['text'] = eventString
            self.autonomousLabel['text'] = stateString
            self.transmissionLabel['text'] = transmissionString
            self.operationalLabel['text'] = operationalString

        def __init__(self, master=None):
            Frame.__init__(self, master)
            self.pack()
            self.quitButton = Button(self)
            self.quitButton["text"] = "QUIT"
            self.quitButton["fg"] = "red"
            self.quitButton["command"] = self.quit
            self.quitButton.grid(column=0, row=4)
            
            self.eventLabel = Label(self)
            self.eventLabel.grid(column=0, row=3)

            self.autonomousLabel = Label(self)
            self.autonomousLabel.grid(column=0, row=1)

            self.transmissionLabel = Label(self)
            self.transmissionLabel.grid(column=0, row=2)

            self.operationalLabel = Label(self)
            self.operationalLabel.grid(column=0, row=0)


rospy.init_node("shore_visualization")

node = ShoreVisualize()
def updateROS():
  global node
  rate = rospy.Rate(rospy.get_param("~rate", 60))

  while not rospy.is_shutdown():
      node.update()
      rate.sleep()

myThread = threading.Thread(target = updateROS)
myThread.start()
node.app.mainloop()  

