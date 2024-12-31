import open3d as o3d
import numpy as np
import argparse
from sklearn.decomposition import PCA
from sklearn.mixture import GaussianMixture

def main():
    # Set up argument parsing
    parser = argparse.ArgumentParser(description='Process some point clouds.')
    parser.add_argument('--input_file', type=str, default='', help='Input pointcloud directory')
    parser.add_argument('--x_min', type=float, default=20.0, help='x min')
    parser.add_argument('--x_half', type=float, default=49.0, help='x half')
    parser.add_argument('--x_max', type=float, default=90.0, help='x max')
    parser.add_argument('--y_min', type=float, default=-2.7, help='y min')
    parser.add_argument('--y_half', type=float, default=-2.3, help='y half')
    parser.add_argument('--y_max', type=float, default=-1.5, help='y max')
    parser.add_argument('--z_min', type=float, default=-1.5, help='z min')
    parser.add_argument('--z_max', type=float, default=1.5, help='z max')
    args = parser.parse_args()

    input_file = args.input_file
    print("input file: " + input_file)

    # Load the point cloud
    cloud = o3d.io.read_point_cloud(input_file)

    # Apply PassThrough filter on the x-axis
    cloud_filtered = cloud.select_by_index(
        np.where((np.asarray(cloud.points)[:, 0] >= args.x_min) & (np.asarray(cloud.points)[:, 0] <= args.x_max))[0])

    # Apply PassThrough filter on the y-axis
    cloud_filtered = cloud_filtered.select_by_index(
        np.where((np.asarray(cloud_filtered.points)[:, 1] >= args.y_min) & (np.asarray(cloud_filtered.points)[:, 1] <= args.y_max))[0])

    # Apply PassThrough filter on the z-axis
    cloud_filtered = cloud_filtered.select_by_index(
        np.where((np.asarray(cloud_filtered.points)[:, 2] >= args.z_min) & (np.asarray(cloud_filtered.points)[:, 2] <= args.z_max))[0])

    for i in range(len(cloud_filtered.points)):
        if cloud_filtered.points[i][0] > args.x_half:
            if cloud_filtered.points[i][1] < args.y_half:
                cloud_filtered.points[i] = [0, 0, 0]  # 将不符合条件的点设为原点，后续会被过滤掉
    
    # 过滤掉坐标为(0,0,0)的点
    cloud_filtered = cloud_filtered.select_by_index(
        np.where((np.asarray(cloud_filtered.points)[:, 0] != 0) & 
                (np.asarray(cloud_filtered.points)[:, 1] != 0) & 
                (np.asarray(cloud_filtered.points)[:, 2] != 0))[0])

    # Save the filtered point cloud
    output_file = input_file.rsplit('/', 1)[0] + '/filtered.pcd'

    if len(cloud_filtered.points) == 0:
        print("Filtered point cloud is empty, cannot save.")
    else:
        o3d.io.write_point_cloud(output_file, cloud_filtered)

if __name__ == "__main__":
    main()


