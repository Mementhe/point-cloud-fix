import copy
import numpy as np
import open3d as o3
from probreg import cpd

def non_rigid_pcd(input1, input2):
    # load source and target point cloud
    source = o3.io.read_point_cloud(input1)
    target = o3.io.read_point_cloud(input2)
    o3.visualization.draw_geometries([target, source])

    target = target.voxel_down_sample(voxel_size=0.5)
    source = source.voxel_down_sample(voxel_size=0.5)
    # compute cpd registration
    tf_param, _, _ = cpd.registration_cpd(source, target)
    result = copy.deepcopy(source)
    result.points = tf_param.transform(result.points)

    pcd = open("../data/in/XYZ.txt")
    pcdtxt = pcd.readlines
    for line in pcdtxt:
            if line == 0:
                station = line.split(",")
                scanX = float(station[0])
                scanY = float(station[1])
                scanZ = float(station[2])
                target.translate((scanX, scanY, scanZ), relative = True)
                source.translate((scanX, scanY, scanZ), relative = True)
            if line == 1:
                station = line.split(",")
                scanX = -float(station[0])
                scanY = -float(station[1])
                scanZ = -float(station[2])
                target.translate((scanX, scanY, scanZ), relative = True)
                source.translate((scanX, scanY, scanZ), relative = True)
    

    # draw result
    #target.scale(100, center=(0, 0, 0))
    #result.scale(100, center=(0, 0, 0))

    o3.io.write_point_cloud("../data/out/target.xyz", target)
    o3.io.write_point_cloud("../data/out/result.xyz", result)
    o3.io.write_point_cloud("../data/out/source.xyz", source)
    o3.visualization.draw_geometries([target, result, source])

