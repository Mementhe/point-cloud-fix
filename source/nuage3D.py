from point3D import point3D
from vector3D import vector3D
from vectorField import vectorField

class nuage3D:
    cloud = []
    def __init__(self):
        self.cloud = []
    
    def print(self):
        for point in self.cloud:
            print(point.getstr())
            
    def add(self, point):
        self.cloud.append(point)

    def getPoint(self, i):
        return self.cloud[i]

    def get(self):
        return self.cloud

    def length(self):
        return len(self.cloud)

    def diff(self, cloudExt):
        field = vectorField()
        if self.length() == cloudExt.length():
            for i in range(0,self.length()):
                vector = vector3D(self.getPoint(i), cloudExt.getPoint(i))
                #print(vector.getstr())
                field.add(vector)
        return field



