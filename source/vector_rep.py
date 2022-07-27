from attr import field
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import numpy as np
from vectorField import *
from vector3D import *
from nuage3D import *
from shifting import vectorshift


def vector_rep():
    field = vectorshift()
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    for vector in field.get():
        ax.quiver(vector.pointA.x, vector.pointA.y, vector.pointA.z, vector.pointB.x, vector.pointB.y, vector.pointB.z, length=0.1, color = 'black')
    plt.show()