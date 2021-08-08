#!/usr/bin/env python
# -*- coding: utf-8 -*-

class Point:
    def __init__(self,xv=0.0,yv=0.0):
        self.x, self.y = xv, yv

def CubicBezier(t, controlPoint):
    p = Point()

    part0 = float(controlPoint[0].x * t ** 3)
    part1 = float(3.0 * controlPoint[1].x * t ** 2 * (1 - t))
    part2 = float(3.0 * controlPoint[2].x * t * (1 - t) ** 2)
    part3 = float(controlPoint[3].x * (1 - t) ** 3)
    p.x = part0 + part1 + part2 + part3

    part0 = float(controlPoint[0].y * t ** 3)
    part1 = float(3.0 * controlPoint[1].y * t ** 2 * (1 - t))
    part2 = float(3.0 * controlPoint[2].y * t * (1 - t) ** 2)
    part3 = float(controlPoint[3].y * (1 - t) ** 3)
    p.y = part0 + part1 + part2 + part3

    return p

class KeyFrame:
    def __init__(self,p=Point(),l=Point(),r=Point()):
        self.point = p
        self.leftCP = Point(self.point.x + l.x, self.point.y + l.y)
        self.rightCP = Point(self.point.x + r.x, self.point.y + r.y)

class BezierPlan:
    def __init__(self, dalta=10, frames=[], speedLimit=0.2, minData=-180, maxData=180):
        self.daltaX = dalta # ms
        self.frames = frames
        self.endFrame = frames[-1]
        self.maxSpeed=speedLimit
        self.minValues=minData
        self.maxValues=maxData
        self.frameIdx = 0
        self.trajectory = []
        self.over = False

    def setFrames(self, frames):
        self.frames = frames

    def getTrajLen(self):
        return len(self.trajectory)

    def valuesLimit(self,valueIn):
        if valueIn<self.minValues:
            valuesOut=self.minValues
        elif valueIn>self.maxValues:
            valuesOut=self.maxValues
        else:
            valuesOut=valueIn
        return valuesOut

    def planing(self):
        self.frameIdx = self.frameIdx + 1
        if(self.frameIdx==1):
            self.trajectory.append(self.frames[self.frameIdx-1].point)   # 添加第一个点
        if self.frameIdx >= len(self.frames):
            self.over = True
            return False

        controlPoint = []
        controlPoint.append(self.frames[self.frameIdx-1].point)
        controlPoint.append(self.frames[self.frameIdx-1].rightCP)
        controlPoint.append(self.frames[self.frameIdx].leftCP)
        controlPoint.append(self.frames[self.frameIdx].point)

        originCurve = []
        daltaT = float(self.daltaX) / (self.frames[self.frameIdx].point.x - self.frames[self.frameIdx-1].point.x)
        t = 1.0-daltaT
        while t>0:
            originCurve.append(CubicBezier(t,controlPoint))
            t = t-daltaT
        originCurve.append(self.frames[self.frameIdx].point)

        time = self.frames[self.frameIdx-1].point.x + self.daltaX
        index = 1
        curveLimit = []
        curveLimit.append(Point(self.trajectory[-1].x, self.valuesLimit(self.trajectory[-1].y)))
        while time < self.frames[self.frameIdx].point.x:
            kk = (originCurve[index].y-curveLimit[-1].y)/(originCurve[index].x-curveLimit[-1].x)
            #maxSpeed
            if(abs(kk)>self.maxSpeed):
                if(kk>0):
                    pointY = curveLimit[-1].y + self.maxSpeed*(time - curveLimit[-1].x)
                else:
                    pointY = curveLimit[-1].y - self.maxSpeed*(time - curveLimit[-1].x)
            else:
                pointY = curveLimit[-1].y + kk*(time - curveLimit[-1].x)
            curveLimit.append(Point(time, self.valuesLimit(pointY)))
            time = time + self.daltaX
            while time >  originCurve[index].x:
                index = index + 1
        
        # curveLimit添加最后一个点
        k = (self.frames[self.frameIdx].point.y-curveLimit[-1].y)/(self.frames[self.frameIdx].point.x-curveLimit[-1].x)
        if(abs(k)>self.maxSpeed):
            if(k>0):
                pointY = curveLimit[-1].y + self.maxSpeed*(time - curveLimit[-1].x)
            else:
                pointY = curveLimit[-1].y - self.maxSpeed*(time - curveLimit[-1].x)
        else:
            pointY = curveLimit[-1].y + k*(time - curveLimit[-1].x)
        curveLimit.append(Point(self.frames[self.frameIdx].point.x, self.valuesLimit(pointY)))
        curveLimit.pop(0)
        self.trajectory.extend(curveLimit)

        return True

    def getTrajectory(self, length):
        if length <= len(self.trajectory):
            out = self.trajectory[:length]
            del self.trajectory[:length]
            return out
        raise Exception("input length out of range!")

def toKeyFrames(frames):
    out = []
    for i in range(len(frames)):
        out.append(KeyFrame( Point(frames[i][0][0], frames[i][0][1]), Point(frames[i][1][0], frames[i][1][1]), Point(frames[i][2][0], frames[i][2][1])))  # 得出　目标点　左右控制点
    return out

class MultiBeizerPlan:
    def __init__(self,number, dalta, frames, angleLimitSet, speedLimit=0.2):
        self.numberOfTraj = number
        self.bPlanObj = []
        for i in range(self.numberOfTraj):
            self.bPlanObj.append(BezierPlan(dalta, toKeyFrames(frames[i]), speedLimit, angleLimitSet[i][0], angleLimitSet[i][1]))

    def planing(self, length):
        for i in range(self.numberOfTraj):
            if self.bPlanObj[i].over == False:
                while self.bPlanObj[i].getTrajLen() <= length:
                    if self.bPlanObj[i].planing() == False:
                        break
    
    def getTrajectory(self, length):
        traj = []
        for i in range(self.numberOfTraj):
            if self.bPlanObj[i].over == False:
                temp = self.bPlanObj[i].getTrajectory(length)
            else:
                temp = self.bPlanObj[i].getTrajectory(self.bPlanObj[i].getTrajLen())
            if len(temp) < length:
                for l in range(len(temp), length):
                    temp.append(self.bPlanObj[i].endFrame.point)
            traj.append(temp)
        return traj

    def sendComplete(self):
        number = 0
        for i in range(self.numberOfTraj):
            if self.bPlanObj[i].getTrajLen() == 0:
                number += 1
        if number == self.numberOfTraj:
            return True
        return False


# example
if __name__ == '__main__':

    from matplotlib import pyplot as plt 

    angleLimitSet = [
        [-70,70]
    ]
    speedLimitSet = 0.2
    frames = [
        [[500, -50], [-100, 0], [100, 0]],
        [[1100, 0], [-100, 0], [100, 0]],
        [[1400, 50], [-100, 0], [100, 0]],
        [[2000, -50], [-100, 0], [100, 0]],
        [[2400, -80], [-100, 0], [100, 0]],
        [[3000, 70], [-100, 0], [100, 0]],
        [[3400, 30], [-100, 0], [100, 0]],
    ]
    frames.insert(0, [[0, -50], [0, 0], [100, 0]])

    bPlan = BezierPlan(10, toKeyFrames(frames), speedLimitSet, angleLimitSet[0][0], angleLimitSet[0][1])
    
    plotX = []
    plotY = []

    while True:
        while bPlan.getTrajLen() <= 12:
            if bPlan.planing() == False:
                break

        if bPlan.over == True:
            break
        
        trajectory = bPlan.getTrajectory(4)
        for i in range(len(trajectory)):
            plotX.append(trajectory[i].x)
            plotY.append(trajectory[i].y)

    trajectory = bPlan.getTrajectory(bPlan.getTrajLen())
    for i in range(len(trajectory)):
        plotX.append(trajectory[i].x)
        plotY.append(trajectory[i].y)

    plt.scatter(plotX, plotY, marker='.') 
    plt.show()
