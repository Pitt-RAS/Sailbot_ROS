#!/usr/bin/env python2

class StationKeeping:
    def __init__(self, stationaryBox):
        self.stationaryBox = stationaryBox
        self.state = "init"

    def getHeading(boatPosition, boatHeading, windSpeed, windHeading):
        if self.state == "init":
            getGoalPoints(windHeading)

        elif self.state == "tack":

        elif self.state == "jibe":

    def getGoalPoints(windHeading):
        yDiff = self.stationaryBox[0].y - self.stationaryBox[1].y
        xDiff = self.stationartBox[0].x - self.stationaryBox[1].x
        boxOrientation = (yDiff / xDiff)


