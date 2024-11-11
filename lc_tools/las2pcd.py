import laspy
import numpy as np
import open3d as o3d

# 读取 LAS 文件
las_file_path = '/home/zyp/DATA/transmission_data/0927/cloud_merged.las'  # 替换为你的 LAS 文件路径
las_data = laspy.read(las_file_path)

# 提取点云数据
x = las_data.x
y = las_data.y
z = las_data.z

# 创建点云
points = np.vstack((x, y, z)).transpose()
cloud = o3d.geometry.PointCloud()
cloud.points = o3d.utility.Vector3dVector(points)

# 保存为 PCD 文件
pcd_file_path = '/home/zyp/DATA/transmission_data/0927/cloud_merged.pcd'  # 替换为输出的 PCD 文件路径
o3d.io.write_point_cloud(pcd_file_path, cloud)

print("Conversion from LAS to PCD completed.")
