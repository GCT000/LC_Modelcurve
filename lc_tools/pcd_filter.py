import open3d as o3d
import numpy as np
import argparse

def main():
    # Set up argument parsing
    parser = argparse.ArgumentParser(description='Process some point clouds.')
    # 这里的参数主要设置下输入的点云文件input_file，然后x的范围x_min和x_max就行，z那两个先不用管
    parser.add_argument('--input_file', type=str, default='', help='Input pointcloud directory')
    parser.add_argument('--x_min', type=float, default=20.0, help='x min')
    parser.add_argument('--x_max', type=float, default=90.0, help='x max')
    parser.add_argument('--z_min', type=float, default=-3.0, help='z min')
    parser.add_argument('--z_max', type=float, default=1.0, help='z max')
    args = parser.parse_args()

    input_file = args.input_file
    print("input file: " + input_file)

    # Load the point cloud
    cloud = o3d.io.read_point_cloud(input_file)

    # save original cloud points to txt
    output_file = input_file.rsplit('/', 1)[0] + '/original_points.txt'
    with open(output_file, 'w') as f:
        for point in cloud.points:
            if point[0] < 120:
                f.write(f"{point[0]} {point[1]} {point[2]}\n")

    # Apply PassThrough filter on the x-axis
    cloud_filtered = cloud.select_by_index(
        np.where((np.asarray(cloud.points)[:, 0] >= args.x_min) & (np.asarray(cloud.points)[:, 0] <= args.x_max))[0])

    # Apply PassThrough filter on the z-axis
    cloud_filtered = cloud_filtered.select_by_index(
        np.where((np.asarray(cloud_filtered.points)[:, 2] >= args.z_min) & (np.asarray(cloud_filtered.points)[:, 2] <= args.z_max))[0])

    # Apply filter on the y-axis
    y_coords = np.asarray(cloud_filtered.points)[:, 1]
    y_mean = np.mean(y_coords)
    y_std = np.std(y_coords)

    y_threshold = 5 * y_std
    cloud_filtered = cloud_filtered.select_by_index(
        np.where((y_coords >= y_mean - y_threshold) & (y_coords <= y_mean + y_threshold))[0])

    # Save the filtered point cloud
    output_file = input_file.rsplit('/', 1)[0] + '/filtered.pcd'

    if len(cloud_filtered.points) == 0:
        print("Filtered point cloud is empty, cannot save.")
    else:
        o3d.io.write_point_cloud(output_file, cloud_filtered)


if __name__ == "__main__":
    main()


