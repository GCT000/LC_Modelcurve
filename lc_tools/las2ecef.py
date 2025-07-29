import laspy
import numpy as np
from pyproj import Transformer
import argparse

def las_to_pcd_ascii(input_las, output_pcd):
    """
    将EPSG:4547坐标系的LAS文件转换为ECEF坐标的ASCII格式PCD文件
    """
    try:
        # 1. 读取LAS文件
        print(f"读取LAS文件: {input_las}")
        las = laspy.read(input_las)
        x, y, z = las.x, las.y, las.z  # EPSG:4547坐标
        print(f"点云数量: {len(x)}")

        # 2. 转换坐标到ECEF（EPSG:4978）
        print("转换坐标到ECEF...")
        transformer = Transformer.from_crs("EPSG:4547", "EPSG:4978", always_xy=True)
        ecef_x, ecef_y, ecef_z = transformer.transform(x, y, z)

        # 3. 写入ASCII格式PCD文件
        print(f"写入ASCII PCD文件: {output_pcd}")
        with open(output_pcd, 'w') as f:
            # PCD文件头（ASCII格式规范）
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write(f"VERSION 0.7\n")
            f.write(f"FIELDS x y z\n")  # 仅保留XYZ坐标（可添加其他字段）
            f.write(f"SIZE 4 4 4\n")    # 每个字段占4字节（float32）
            f.write(f"TYPE F F F\n")    # 类型为浮点型
            f.write(f"COUNT 1 1 1\n")   # 每个字段的元素数量
            f.write(f"WIDTH {len(ecef_x)}\n")  # 点云数量
            f.write(f"HEIGHT 1\n")      # 非organized点云
            f.write(f"VIEWPOINT 0 0 0 1 0 0 0\n")  # 视点（默认原点）
            f.write(f"POINTS {len(ecef_x)}\n")    # 点数量
            f.write(f"DATA ascii\n")    # 数据格式为ASCII

            # 写入点数据（每行一个点，x y z空格分隔）
            for px, py, pz in zip(ecef_x, ecef_y, ecef_z):
                f.write(f"{px:.6f} {py:.6f} {pz:.6f}\n")  # 保留6位小数

        print("转换完成!")

    except Exception as e:
        print(f"转换失败: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', required=True, help='输入LAS文件路径')
    parser.add_argument('--output', required=True, help='输出PCD文件路径')
    args = parser.parse_args()
    las_to_pcd_ascii(args.input, args.output)
