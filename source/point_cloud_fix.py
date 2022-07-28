from origin import *
from shifting import *
from registration import *
from module1 import *
from vector_rep import *

#origin("../data/in/station%d.pts", "../data/out/station_%d_Origin.pts", "../data/in/XYZ.txt", 3)
#shifting("../data/out/station_%d_Origin.pts", 3)
#registration("../data/out/station_%d.pts", "../data/out/station_Final.pts", 3)
non_rigid_pcd("../data/out/station_1.pts", "../data/out/station_2.pts")
#vectorshift()
#vector_rep()