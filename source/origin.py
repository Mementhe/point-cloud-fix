import open3d as o3d

#ok
def origin(input, output, scan_pos, station_nb):
    for i in range(1,station_nb):
        pcd = o3d.io.read_point_cloud(input %
                i)
        #move pcd origin
        scan = open(scan_pos)
        scanpos = scan.readlines()
        station = []
        lineIndex = 0
        for line in scanpos:
            lineIndex = lineIndex + 1
            if lineIndex == i:
                station = line.split(",")
                scanX = -float(station[0])
                scanY = -float(station[1])
                scanZ = -float(station[2])

            if len(station)!=0 and lineIndex == i:
                pcd.translate((scanX,scanY,scanZ), relative=True)
                #pcd.scale(0.01, center=(0, 0, 0))
                pcd = pcd.voxel_down_sample(voxel_size=0.05)
                o3d.io.write_point_cloud(output % i, pcd)
    
        


