import rosbag
import cv2
import open3d as o3d
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2

# Initialize CV bridge
bridge = CvBridge()

# Open the bag file
bag = rosbag.Bag('/home/zyp/transmission_data/0912/test0912_3.bag')

# Get the actual start time of the bag
bag_start_time = bag.get_start_time()

# Define the desired time range
start_time = 0.0 + bag_start_time  # the start time
end_time = 26.0 + bag_start_time    # the start time

# Initialize variables
point_clouds = []
images = []

# Loop over the bag file
for topic, msg, t in bag.read_messages(topics=['/camera/image_raw', '/livox/lidar']):
    time_in_sec = t.to_sec()
    
    # Extract point cloud data
    if topic == '/livox/lidar' and start_time <= time_in_sec <= end_time:
        pc_array = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
        point_clouds.append(pc_array)
    
    # Extract image data
    if topic == '/camera/image_raw' and start_time <= time_in_sec <= end_time:
        img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        images.append((time_in_sec, img))

# Save point cloud as a PCD file
if point_clouds:
    pc_combined = np.concatenate(point_clouds, axis=0)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc_combined)
    o3d.io.write_point_cloud("extracted_points.pcd", pcd)

# Save images at each second
for time_in_sec, img in images:
    # Calculate the second timestamp relative to the start time
    sec_timestamp = int(time_in_sec - bag_start_time)
    # Save image
    cv2.imwrite(f"image_{sec_timestamp}.png", img)

# Close the bag file
bag.close()
