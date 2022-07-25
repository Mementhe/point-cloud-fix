import copy
import numpy as np
import open3d as o3
from probreg import cpd

def test(input1, input2):
    # load source and target point cloud
    source = o3.io.read_point_cloud(input1)
    target = o3.io.read_point_cloud(input2)

    # compute cpd registration
    tf_param, _, _ = cpd.registration_cpd(source, target)
    result = copy.deepcopy(source)
    result.points = tf_param.transform(result.points)

    # draw result
    #target.scale(100, center=(0, 0, 0))
    #result.scale(100, center=(0, 0, 0))

    o3.io.write_point_cloud("data/out/target.pts", target)
    o3.io.write_point_cloud("data/out/result.pts", result)
    o3.visualization.draw_geometries([target, result])

