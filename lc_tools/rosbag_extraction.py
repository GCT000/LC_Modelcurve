import rosbag
import cv2
import open3d as o3d
import numpy as np
import sys
import os
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2

# get args
if len(sys.argv) != 4:
    print("Usage: python rosbag_filter.py <bag_path> <image_topic> <lidar_topic>")
    sys.exit(1)

bag_path = sys.argv[1]
image_topic = sys.argv[2]
lidar_topic = sys.argv[3]

# create output dir
bag_dir = os.path.dirname(bag_path)  # get bag file dir
output_dir = os.path.join(bag_dir, 'extracted')  # create extracted sub dir
os.makedirs(output_dir, exist_ok=True)  # create dir, if exists, do not report error

# Initialize CV bridge
bridge = CvBridge()

# Open the bag file
bag = rosbag.Bag(bag_path)

# Define the desired time range
start_time = bag.get_start_time()  # the start time
end_time = bag.get_end_time()    # the start time

# Initialize variables
point_clouds = []
images = []

# Loop over the bag file
for topic, msg, t in bag.read_messages(topics=[image_topic, lidar_topic]):
    time_in_sec = t.to_sec()
    
    # Extract point cloud data
    if topic == lidar_topic and start_time <= time_in_sec <= end_time:
        pc_array = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
        point_clouds.append(pc_array)
    
    # Extract image data
    if topic == image_topic and start_time <= time_in_sec <= end_time:
        img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        images.append((time_in_sec, img))

# Save point cloud as a PCD file
if point_clouds:
    pc_combined = np.concatenate(point_clouds, axis=0)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc_combined)
    print("save pcd to: ", os.path.join(output_dir, "extracted_points.pcd"))
    o3d.io.write_point_cloud(os.path.join(output_dir, "extracted_points.pcd"), pcd)

# Save images at each second
for time_in_sec, img in images:
    # Calculate the second timestamp relative to the start time
    sec_timestamp = int(time_in_sec - start_time)
    # Save image
    cv2.imwrite(os.path.join(output_dir, f"image_{sec_timestamp}.png"), img)
    print("save image to: ", os.path.join(output_dir, f"image_{sec_timestamp}.png"))
# Close the bag file
bag.close()
