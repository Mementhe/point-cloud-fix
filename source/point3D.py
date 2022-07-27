from cmath import sqrt
import math
class point3D:
    x = 0
    y = 0
    z = 0
    def __init__(self,x,y,z):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
    
    def getstr(self):
        return str(self.x) +' ' +str(self.y)+' ' +str(self.z)

    def print(self):
        print(self.getstr())

    def dist(self):
        return sqrt(self.x**2 + self.y**2 + self.z**2)
    
    
