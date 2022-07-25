from origin import *
from shifting import *
from registration import *
from module1 import *

origin("in/station%d.pts", "out/station_%d_Origin.pts", "in/XYZ.txt", 3)
shifting("out/station_%d_Origin.pts", 3)
registration("out\station_%d.pts", "out\station_%d_Final.pts", 4)
test("out/station_1_Final.pts", "out/station_2_Final.pts")