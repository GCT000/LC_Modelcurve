import rosbag
import cv2
import open3d as o3d
import numpy as np
import sys
import os
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2

def main():
    # get args
    if len(sys.argv) < 2:
        print("Usage: python rosbag_extraction.py <bag_path> [image_topic] [lidar_topic]")
        sys.exit(1)

    bag_path = sys.argv[1]
    # 可选参数处理
    image_topic = sys.argv[2] if len(sys.argv) > 2 else None
    lidar_topic = sys.argv[3] if len(sys.argv) > 3 else None

    if not image_topic and not lidar_topic:
        print("Error: At least one topic (image or lidar) must be specified")
        sys.exit(1)

    # create output dir
    bag_dir = os.path.dirname(bag_path)
    output_dir = os.path.join(bag_dir, 'extracted')
    os.makedirs(output_dir, exist_ok=True)

    # Initialize CV bridge if needed
    bridge = CvBridge() if image_topic else None

    # Open the bag file
    bag = rosbag.Bag(bag_path)
    start_time = bag.get_start_time()
    end_time = bag.get_end_time()

    # 准备要读取的话题列表
    topics_to_read = [topic for topic in [image_topic, lidar_topic] if topic]

    # Initialize variables
    point_clouds = [] if lidar_topic else None
    images = [] if image_topic else None

    # Loop over the bag file
    for topic, msg, t in bag.read_messages(topics=topics_to_read):
        time_in_sec = t.to_sec()
        
        # Extract point cloud data
        if lidar_topic and topic == lidar_topic and start_time <= time_in_sec <= end_time:
            pc_array = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
            point_clouds.append(pc_array)
        
        # Extract image data
        if image_topic and topic == image_topic and start_time <= time_in_sec <= end_time:
            img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            images.append((time_in_sec, img))

    # Save point cloud if exists
    if point_clouds and point_clouds:
        pc_combined = np.concatenate(point_clouds, axis=0)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pc_combined)
        pcd_path = os.path.join(output_dir, "extracted_points.pcd")
        o3d.io.write_point_cloud(pcd_path, pcd)
        print(f"Saved point cloud to: {pcd_path}")

    # Save images if exists
    if images and images:
        for time_in_sec, img in images:
            sec_timestamp = int(time_in_sec - start_time)
            img_path = os.path.join(output_dir, f"image_{sec_timestamp}.png")
            cv2.imwrite(img_path, img)
            print(f"Saved image to: {img_path}")

    # Close the bag file
    bag.close()

    # Print summary
    if point_clouds:
        print(f"Total point clouds processed: {len(point_clouds)}")
    if images:
        print(f"Total images processed: {len(images)}")

if __name__ == "__main__":
    main()