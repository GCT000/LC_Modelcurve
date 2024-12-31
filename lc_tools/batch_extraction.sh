#!/bin/bash

# 定义数据目录
DATA_DIR="/ssd/DATA/Transmisson/PJ/data"

# 遍历数据目录下的每个文件夹
for folder in "$DATA_DIR"/*; do
  if [ -d "$folder" ]; then
    # 获取文件夹的名称
    folder_name=$(basename "$folder")

    # 构造bag文件名
    bag_file="record_${folder_name}.bag"

    # 打印当前执行的命令
    echo "Processing $folder_name with bag file: $bag_file"

    # 执行指定的Python命令
    python rosbag_extraction.py "${folder}/${bag_file}" /camera_1/image_raw /livox/lidar
  fi
done

