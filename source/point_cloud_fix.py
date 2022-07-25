from origin import *
from shifting import *
from registration import *
from module1 import *

origin("../data/in/station%d.pts", "../data/out/station_%d_Origin.pts", "../data/in/XYZ.txt", 3)
#shifting("../data/out/station_%d_Origin.pts", 3)
#registration("../data/out/station_%d.pts", "../data/out/station_%d_Final.pts", 4)
test("../data/out/station_1_Origin.pts", "../data/out/station_2_Origin.pts")