from cmath import sqrt
from point3D import point3D


class vector3D:
    pointA = 0
    pointB = 0
    def __init__(self, pointA, pointB):
        self.pointA = point3D(pointA.x, pointA.y, pointA.z)
        self.pointB = point3D(pointB.x, pointB.y, pointB.z)
    
    def add(self, pointA, pointB):
        self.pointA = pointA
        self.pointB = pointB

    def norm(self):
        return sqrt((self.pointB.x - self.pointA.x)**2 + (self.pointB.y - self.pointA.y)**2 + (self.pointB.z - self.pointA.z)**2)

    def getstr(self):
        return self.pointA.getstr() + ' -> ' + self.pointB.getstr()
    
    
