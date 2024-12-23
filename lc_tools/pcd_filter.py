import open3d as o3d
import numpy as np
import argparse

def main():
    # Set up argument parsing
    parser = argparse.ArgumentParser(description='Process some point clouds.')
    parser.add_argument('--input_dir', type=str, default='/home/zyp/LC-CurveModel/tools/0912_5/', help='Input pointcloud directory')
    args = parser.parse_args()

    input_file = args.input_dir + "extracted_points.pcd"
    print("input file: " + input_file)

    # Load the point cloud
    cloud = o3d.io.read_point_cloud(input_file)

    # Apply PassThrough filter on the x-axis
    cloud_filtered = cloud.select_by_index(
        np.where((np.asarray(cloud.points)[:, 0] >= 35.0) & (np.asarray(cloud.points)[:, 0] <= 60))[0])

    # Apply PassThrough filter on the y-axis
    cloud_filtered = cloud_filtered.select_by_index(
        np.where((np.asarray(cloud_filtered.points)[:, 1] >= 4.40) & (np.asarray(cloud_filtered.points)[:, 1] <= 4.50))[0])

    # Apply PassThrough filter on the z-axis
    cloud_filtered = cloud_filtered.select_by_index(
        np.where((np.asarray(cloud_filtered.points)[:, 2] >= 0.40) & (np.asarray(cloud_filtered.points)[:, 2] <= 1.90))[0])

    # Save the filtered point cloud
    output_file = args.input_dir + "filtered.pcd"

    if len(cloud_filtered.points) == 0:
        print("Filtered point cloud is empty, cannot save.")
    else:
        o3d.io.write_point_cloud(output_file, cloud_filtered)

if __name__ == "__main__":
    main()


