from difflib import Differ
import open3d as o3d
import math
import numpy as np
from nuage3D import nuage3D
from point3D import point3D

#ok
def shifting(input, station_nb):
    for i in range(1,station_nb):
        originPCD = open(input % 
                 i, "r")
        new_file = open(r"../data/out/station_%d_Final.pts" % 
                        i, "w")

        print("Fixing PCD")

        #first line
        for line in originPCD:
            new_file.write("%s" % line)
            break
        lines = originPCD.readlines()[1:]
        for line in lines:
        #clean data/create new lists
            RGB=[]
            intensity=[]
            L = line.split()
            if len(L)!=0:
                intensity.append(L[3])
                RGB.append(L[4])
                RGB.append(L[5])
                RGB.append(L[6])
                del L[3:]

        #collect coord data
            M=[]
            if len(L)!=0:
                for item in L:
                    M.append(float(item))
                #x
                #dx = math.sqrt((M[1])**2 + (M[2])**2)
                #M[0] = (2.198*dx+246.873*float(M[0]))/244.21
                #y
                #dy = math.sqrt((M[0])**2 + (M[2])**2)
                #M[1] = (3.917*dy+103.283*float(M[1]))/101.322
                M[0] = 0.99869*float(M[0])+0.07113
                M[1] = 1.00576*float(M[1])+0.06145
                #z
                dz = math.sqrt(M[0]**2 + M[1]**2)
                M[2] = -(4.704*dz-63.734*float(M[2])-44.497)/56.049
                
                

                #D = math.sqrt(M[0]**2 + M[1]**2 + M[2]**2)
                #D = 0.93465*D

                #M[1] = math.sqrt(D**2 - M[0]**2 - M[2]**2)
                #M[0] = math.sqrt(D**2 - M[1]**2 - M[2]**2)
        
                M[0] = float(round(M[0],10))
                M[1] = float(round(M[1],10))
                M[2] = float(round(M[2],10))
                
        #Write new file
            if len(L)!=0:
                full_line = M + intensity + RGB
                list_joined = " ".join(map(str, full_line))
                new_file.write("%s" % list_joined)
                new_file.write("\n")
            M=[]

        #visu
        new_pcd = o3d.io.read_point_cloud("../data/out/station_%d_Final.pts" 
                                          % i)
        #new_pcd.translate((0,0,10), relative=True)
        o3d.visualization.draw_geometries([new_pcd])

def vectorshift():
    result = open("../data/out/result.xyz", "r")
    source = open("../data/out/source.xyz", "r")

    cloudRes = nuage3D()
    cloudSrc = nuage3D()
    
    for row in result:
        res = []
        res.append(row.split())
        point3D_1 = point3D(res[0][0],res[0][1],res[0][2])
        cloudRes.add(point3D_1)

    for row in source:
        src = []
        src.append(row.split())
        point3D_2 = point3D(src[0][0],src[0][1],src[0][2])
        cloudSrc.add(point3D_2)

    field = cloudRes.diff(cloudSrc)

    for vector in field.get():
        print(str(vector.norm())+ ',' + str(vector.pointA.dist()))
    return field

