#!/usr/bin/env python2

from math import atand, cos, sin, abs, pow

class StationKeeping:
    def __init__(self, stationaryBox):
        self.stationaryBox = stationaryBox
        self.state = "init"
        self.tackGoal = [0, 0]
        self.jibeGoal = [0, 0]
        self.tackToSide = [0, 0]
        # How far should goal points be?
        self.distFromBox = 10
        self.maxWindTol = 10
        self.turnAroundDist = 5
        self.setWind = 0

    def getGoal(boatPosition, boatHeading, windSpeed, windHeading):
        if self.state == "init":
            setGoalPoints(windHeading)
            self.setWind = windHeading
            distToTack = distToLine(boatPosition[0], boatPosition[1], self.stationaryBox[self.tackToSide[0]], self.stationaryBox[self.tackToSide[1]])
            distToJibe = distToLine(boatPosition[0], boatPosition[1], self.stationaryBox[self.jibeToSide[0]], self.stationaryBox[self.jibeToSide[1]])
            if (distToTack < distToJibe):
                self.state = "jibe"
                return makePoint(jibeGoal)
            else:
                self.state = "tack"
                return makePoint(tackGoal)

        elif self.state == "tack":
            if abs(self.setWind - windHeading) > self.maxWindTol:
                self.state = "init"
            if distToLine(boatPosition[0], boatPosition[1], self.stationaryBox[self.tackToSide[0]], self.stationaryBox[self.tackToSide[1]] <= self.turnAroundDist):
                self.state = "jibe"
            return makePoint(tackGoal)

        elif self.state == "jibe":
            if abs(self.setWind - windHeading) > self.maxWindTol:
                self.state = "init"
            if distToLine(boatPosition[0], boatPosition[1], self.stationaryBox[self.jibeToSide[0]], self.stationaryBox[self.jibeToSide[1]] <= self.turnAroundDist):
                self.state = "tack"
            return makePoint(tackGoal)

    def setGoalPoints(self, windHeading):
        yDiff = self.stationaryBox[0].y - self.stationaryBox[3].y
        xDiff = self.stationartBox[0].x - self.stationaryBox[3].x
        boxOrientation = atand(yDiff / xDiff)
        windRelToBox = (windHeading - boxOrientation - 45) % 360
        # Boat should tack up to side of box between 0 and 1
        if windRelToBox >= 0 and windRelToBox < 90:
            self.tackToSide = [0, 1]
            self.tackGoal = getPoint(0, 1, 2, 3) 
            self.jibeGoal = getPoint(2, 3, 0, 1)
        # Tack up to side of box between 1 and 2
        elif windRelToBox < 180:
            self.tackToSide = [1, 2]
            self.tackGoal = getPoint(1, 2, 3, 0) 
            self.jibeGoal = getPoint(3, 0, 1, 2)
        # Tack up to side of box between 2 and 3
        elif windRelToBox < 270:
            self.tackToSide = [2, 3]
            self.tackGoal = getPoint(2, 3, 0, 1)
            self.jibeGoal = getPoint(0, 1, 2, 3)
        # Tack up to side of box between 3 and 0
        else:
            self.tackToSide = [3, 0]
            self.tackGoal = getPoint(3, 0, 1, 2)
            self.jibeGoal = getPoint(1, 2, 3, 0)

    def getPoint(self, p1, p2, otherP1, otherP2):
        yDiff = self.stationaryBox[p1].y - self.stationaryBox[p2].y
        xDiff = self.stationaryBox[p1].x - self.stationaryBox[p2].x
        slope = -1/(yDiff / xDiff)
        theta = atand(slope)
        yMid = (self.stationaryBox[0].y + self.stationaryBox[1].y) / 2
        xMid = (self.stationaryBox[0].x + self.stationaryBox[1].x) / 2
        intercept = yMid - slope*xMid
        newX = midx + (self.distFromBox / cos(theta))
        newY = midY + (self.distFromBox / sin(theta))
        dist1 = distToLine(newX, newY, p1, p2)
        dist2 = distToLine(newX, newY, otherP1, otherP2)
        if dist1 < dist2:
            return [newX, newY]
        else:
            newX = midX - (self.distFromBox / cos(theta))
            newY = midY - (self.distFromBox / sin(theta))
            return [newX, newY]

    def distToLine(self, newX, newY, p1, p2):
        distNum = newX*(self.stationaryBox[p2].y - self.stationaryBox[p1].y)
        distNum -= newY*(self.stationaryBox[p2].x - self.stationaryBox[p1].x)
        distNum += self.stationaryBox[p2].x * self.stationaryBox[p1].y
        distNum -= self.stationaryBox[p2].y * self.stationaryBox[p1].x
        distNum = abs(dist1Num)
        distDen = pow((self.stationaryBox[p2].y - self.stationaryBox[p1].y), 2)
        distDen = pow((self.stationaryBox[p2].x - self.stationaryBox[p1].x), 2)
        distDen = sqrt(dist1Den)
        dist = dist1Num / dist1Den
        return dist

    def makePoint(coor):
        newPoint = Point()
        newPoint.x = coor[0]
        newPoint.y = coor[1]
        newPoint.z = 0
        return newPoint



