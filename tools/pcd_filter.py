import open3d as o3d
import numpy as np
import argparse

# Set up argument parsing
parser = argparse.ArgumentParser(description='Process some point clouds.')
parser.add_argument('--input_dir', type=str, default='/home/zyp/LC-CurveModel/tools/test1/', help='Input pointcloud directory')
args = parser.parse_args()

input_file = args.input_dir + "extracted_points.pcd"

# Load the point cloud
cloud = o3d.io.read_point_cloud(input_file)

# Apply PassThrough filter on the x-axis
cloud_filtered = cloud.select_by_index(
    np.where((np.asarray(cloud.points)[:, 0] >= 10.0) & (np.asarray(cloud.points)[:, 0] <= 100.0))[0])

# Apply PassThrough filter on the y-axis
cloud_filtered = cloud_filtered.select_by_index(
    np.where((np.asarray(cloud_filtered.points)[:, 1] >= -4.0) & (np.asarray(cloud_filtered.points)[:, 1] <= -0.5))[0])

# Apply PassThrough filter on the z-axis
cloud_filtered = cloud_filtered.select_by_index(
    np.where((np.asarray(cloud_filtered.points)[:, 2] >= -6.0) & (np.asarray(cloud_filtered.points)[:, 2] <= 2.40))[0])

# # Define CropBox parameters
# min_bound = np.array([18.0, 0.4, -1.0])
# max_bound = np.array([20.0, 0.5, -0.2])

# # Apply CropBox filter with the negative option
# bounding_box = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
# cloud_filtered2 = cloud_filtered.crop(bounding_box, invert=True)

# Save the filtered point cloud
output_file = args.input_dir + "filter.pcd"
o3d.io.write_point_cloud(output_file, cloud_filtered)
