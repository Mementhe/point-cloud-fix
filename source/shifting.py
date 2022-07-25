import open3d as o3d
import math

#ok
def shifting(input, station_nb):
    for i in range(1,station_nb):
        originPCD = open(input % 
                 i, "r")
        new_file = open(r"out/station_%d_Final.pts" % 
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
                dx = math.sqrt((M[1])**2 + (M[2])**2)
                M[0] = -(-0.459*dx-109.99*float(M[0]))/109.345
                #z
                dz = math.sqrt(M[0]**2 + M[1]**2)
                M[2] = -(3.58951*dz-101.97345*float(M[2]))/90.80112
                #y
                dy = math.sqrt((M[0])**2 + (M[2])**2)
                M[1] = -(-0.022*dy-80.882*float(M[1]))/80.118


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
        new_pcd = o3d.io.read_point_cloud("out/station_%d_Final.pts" 
                                          % i)
        #new_pcd.translate((0,0,10), relative=True)
        o3d.visualization.draw_geometries([new_pcd],
                                          zoom=0.3412,
                                          front=[0.4257, -0.2125, -0.8795],
                                          lookat=[2.6172, 2.0475, 1.532],
                                          up=[-0.0694, -0.9768, 0.2024])