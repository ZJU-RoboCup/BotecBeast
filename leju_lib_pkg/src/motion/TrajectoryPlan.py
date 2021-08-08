
class Point:
    x = 0.0
    y = 0.0

    def __init__(self,xv=0.0,yv=0.0):
        self.x, self.y = xv, yv

class AutoBezier:
    abOriginPoint = []
    abControlPoint = []
    abCurveScaling = 0.3
    abInterval = 100.0
    abDaltaX = 10.0

    def __init__(self, daltaX=10):
        self.abDaltaX = daltaX
        self.abOriginPoint = [Point() for i in range(3)]
        self.abControlPoint = [Point() for i in range(4)]

    def setCurveScaling(self, v):
        self.abCurveScaling = v

    def setDaltaX(self, x):
        self.abDaltaX = x
    
    def bezier3(self, t, controlP):
        p = Point()

        part0 = float(controlP[0].x * t**3)
        part1 = float(3.0 * controlP[1].x * t**2 * (1 - t))
        part2 = float(3.0 * controlP[2].x * t * (1 - t)**2)
        part3 = float(controlP[3].x * (1 - t)**3)
        p.x = part0 + part1 + part2 + part3

        part0 = float(controlP[0].y * t**3)
        part1 = float(3.0 * controlP[1].y * t**2 * (1 - t))
        part2 = float(3.0 * controlP[2].y * t * (1 - t)**2)
        part3 = float(controlP[3].y * (1 - t)**3)
        p.y = part0 + part1 + part2 + part3

        return p


    def begin(self, firstPoint, secondPoint):
        self.abOriginPoint[0] = (firstPoint)
        self.abOriginPoint[1] = (secondPoint)

        self.abInterval = float(self.abOriginPoint[1].x - self.abOriginPoint[0].x)
        self.abControlPoint[0] = (self.abOriginPoint[0])
        self.abControlPoint[1] = (Point(self.abOriginPoint[0].x+(self.abInterval*self.abCurveScaling), self.abOriginPoint[0].y))
        

    def processing(self, nextPoint, curvePoint):
        self.abOriginPoint[2] = nextPoint

        if (((self.abOriginPoint[0].y < self.abOriginPoint[1].y) and (self.abOriginPoint[1].y < self.abOriginPoint[2].y)) or
            ((self.abOriginPoint[0].y > self.abOriginPoint[1].y) and (self.abOriginPoint[1].y > self.abOriginPoint[2].y))):
            v1 = Point(self.abOriginPoint[0].x - self.abOriginPoint[1].x, self.abOriginPoint[0].y - self.abOriginPoint[1].y)
            v2 = Point(self.abOriginPoint[2].x - self.abOriginPoint[1].x, self.abOriginPoint[2].y - self.abOriginPoint[1].y)
            if v1.x/v2.x == v1.y/v2.y:
                k = v1.y/v1.x
            else:
                v = Point((self.abOriginPoint[0].x+self.abOriginPoint[1].x) - (self.abOriginPoint[1].x+self.abOriginPoint[2].x), 
                          (self.abOriginPoint[0].y+self.abOriginPoint[1].y) - (self.abOriginPoint[1].y+self.abOriginPoint[2].y))
                k = v.y/v.x
        else:
            k = 0.0

        self.abControlPoint[2] = Point(self.abOriginPoint[1].x-(self.abInterval*self.abCurveScaling), self.abOriginPoint[1].y-k*(self.abInterval*self.abCurveScaling))
        self.abControlPoint[3] = self.abOriginPoint[1]

        originCurve = []
        daltaT = self.abDaltaX / (self.abOriginPoint[1].x - self.abOriginPoint[0].x)
        t = 1.0-daltaT
        while t>0:
            originCurve.append(self.bezier3(t, self.abControlPoint))
            t = t-daltaT
        originCurve.append(self.abOriginPoint[1])

        time = self.abOriginPoint[0].x + self.abDaltaX
        index = 1
        while time < self.abOriginPoint[1].x:
            k = (originCurve[index].y-originCurve[index-1].y)/(originCurve[index].x-originCurve[index-1].x)
            curvePoint.append(Point(time, originCurve[index-1].y+k*(time-originCurve[index-1].x)))
            time = time + self.abDaltaX
            if time >= originCurve[index].x:
                index = index + 1
        curvePoint.append(self.abOriginPoint[1])
            
        self.abInterval = float(self.abOriginPoint[2].x - self.abOriginPoint[1].x)
        
        self.abControlPoint[0] = self.abOriginPoint[1]
        self.abControlPoint[1] = Point(self.abOriginPoint[1].x+(self.abInterval*self.abCurveScaling), self.abOriginPoint[1].y+k*(self.abInterval*self.abCurveScaling))
        
        self.abOriginPoint[0] = self.abOriginPoint[1]
        self.abOriginPoint[1] = self.abOriginPoint[2]

    def end(self, curvePoint):
        self.abControlPoint[2] = Point(self.abOriginPoint[1].x-(self.abInterval*self.abCurveScaling), self.abOriginPoint[1].y)
        self.abControlPoint[3] = self.abOriginPoint[1]
        
        originCurve = []
        daltaT = self.abDaltaX / (self.abOriginPoint[1].x - self.abOriginPoint[0].x)
        t = 1.0-daltaT
        while t>0:
            originCurve.append(self.bezier3(t, self.abControlPoint))
            t = t-daltaT
        originCurve.append(self.abOriginPoint[1])

        time = self.abOriginPoint[0].x + self.abDaltaX
        index = 1
        while time < self.abOriginPoint[1].x:
            k = (originCurve[index].y-originCurve[index-1].y)/(originCurve[index].x-originCurve[index-1].x)
            curvePoint.append(Point(time, originCurve[index-1].y+k*(time-originCurve[index-1].x)))
            time = time + self.abDaltaX
            if time >= originCurve[index].x:
                index = index + 1
        curvePoint.append(self.abOriginPoint[1])

class TrajectoryPlanning:
    tpAbObject = []
    tpNumberOfTraj = 22
    tpInterval = 1000.0
    tpX = 0.0

    def __init__(self, numberOfTraj=22, daltaX=10.0):
        self.tpNumberOfTraj = numberOfTraj
        self.tpAbObject = [AutoBezier() for i in range(self.tpNumberOfTraj)]
        for i in range(self.tpNumberOfTraj):
            self.tpAbObject[i].setDaltaX(daltaX)
            self.tpAbObject[i].setCurveScaling(0.2)

    def setDaltaX(self, daltaX):
        for i in range(self.tpNumberOfTraj):
            self.tpAbObject[i].setDaltaX(daltaX)

    def setInterval(self, v):
        self.tpInterval = v

    def planningBegin(self, firstGroupValue, secondGroupValue):
        firstGroupPoint = []
        secondGroupPoint = []
        for i in range(self.tpNumberOfTraj):
            firstGroupPoint.append(Point(self.tpX, firstGroupValue[i]))
            secondGroupPoint.append(Point(self.tpX+self.tpInterval, secondGroupValue[i]))
            self.tpAbObject[i].begin(firstGroupPoint[i], secondGroupPoint[i])
        self.tpX = self.tpX + self.tpInterval

    def planning(self, nextGroupValue):
        curvePoint = []
        nextGroupPoint = []
        self.tpX = self.tpX + self.tpInterval
        for i in range(self.tpNumberOfTraj):
            curPoint = []
            nextGroupPoint.append(Point(self.tpX, nextGroupValue[i]))
            self.tpAbObject[i].processing(nextGroupPoint[i], curPoint)
            curvePoint.append(curPoint)
        return curvePoint

    def planningEnd(self):
        curvePoint = []
        for i in range(self.tpNumberOfTraj):
            curPoint = []
            self.tpAbObject[i].end(curPoint)
            curvePoint.append(curPoint)
        self.tpX = 0.0
        return curvePoint



# auto bezier example code:
if __name__ == '__main__':
    pList = [Point(0.0, 100.0), Point(100.0, -100.0), Point(200.0, 100.0), Point(300.0, -100.0)]
    curvePoint = []

    abObject = AutoBezier()
    abObject.begin(pList[0],pList[1])
    abObject.processing(pList[2],curvePoint)
    # get curvePoint
    for i in curvePoint:
        print(i.y)
    curvePoint.clear()
    abObject.processing(pList[3],curvePoint)
    # get curvePoint
    for i in curvePoint:
        print(i.y)
    curvePoint.clear()
    abObject.end(curvePoint)
    # get curvePoint
    for i in curvePoint:
        print(i.y)
    curvePoint.clear()
