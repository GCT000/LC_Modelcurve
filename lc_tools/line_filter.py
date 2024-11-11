import open3d as o3d
import numpy as np

# 读取 PCD 文件
pcd_file_path = '/home/zyp/DATA/transmission_data/0927/test3/filter.pcd'  # 替换为你的 PCD 文件路径
cloud = o3d.io.read_point_cloud(pcd_file_path)

# 使用 RANSAC 拟合平面（直线近似）
def fit_line_ransac(cloud):
    plane_model, inliers = cloud.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
    inlier_cloud = cloud.select_by_index(inliers)
    outlier_cloud = cloud.select_by_index(inliers, invert=True)
    return inlier_cloud, outlier_cloud

# 拟合第一条曲线
line1_cloud, remaining_cloud = fit_line_ransac(cloud)

# 在剩余的点云中拟合第二条曲线
line2_cloud, _ = fit_line_ransac(remaining_cloud)

# 比较两条曲线的点数
if len(line1_cloud.points) > len(line2_cloud.points):
    final_cloud = line1_cloud
else:
    final_cloud = line2_cloud

# 保存最终保留的点云
output_pcd_file_path = '/home/zyp/DATA/transmission_data/0927/test3/line.pcd'  # 替换为输出的 PCD 文件路径
o3d.io.write_point_cloud(output_pcd_file_path, final_cloud)

print("Filtered point cloud saved.")
