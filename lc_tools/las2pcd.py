import laspy
import numpy as np
import open3d as o3d
import sys
import os

# get args
if len(sys.argv) != 2:
    print("Usage: python las2pcd.py <las_file_path>")
    sys.exit(1)

las_file_path = sys.argv[1]

# read las file
las_data = laspy.read(las_file_path)

# extract point cloud data
x = las_data.x
y = las_data.y
z = las_data.z

# create point cloud
points = np.vstack((x, y, z)).transpose()
cloud = o3d.geometry.PointCloud()
cloud.points = o3d.utility.Vector3dVector(points)

# save as pcd file
pcd_file_path = os.path.join(os.path.dirname(las_file_path), "cloud_merged.pcd")
o3d.io.write_point_cloud(pcd_file_path, cloud)

print("Conversion from LAS to PCD completed.")
