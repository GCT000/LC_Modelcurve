import numpy as np

def read_pcd_header(file):
    """读取PCD文件的头部信息"""
    header = {}
    while True:
        line = file.readline().strip()
        if not line:
            continue
        if line.startswith('DATA'):
            header['DATA'] = line.split()[1]
            break
        parts = line.split()
        if len(parts) >= 2:
            header[parts[0]] = parts[1:]
    return header

def read_pcd_points(file, header):
    """读取PCD文件中的点数据"""
    points = []
    num_points = int(header['POINTS'][0])
    
    # 对于ASCII格式的点云
    if header['DATA'] == 'ascii':
        for _ in range(num_points):
            line = file.readline().strip()
            while not line:  # 跳过空行
                line = file.readline().strip()
            x, y, z = map(float, line.split())
            points.append([x, y, z, 1.0])  # 添加齐次坐标w=1
    
    return np.array(points, dtype=np.float64)

def transform_points(points, transform_matrix):
    """应用变换矩阵到点集"""
    # 矩阵乘法: 变换矩阵 × 点矩阵(转置)
    transformed_points = np.dot(transform_matrix, points.T).T
    # 移除齐次坐标，只保留x, y, z
    return transformed_points[:, :3]

def write_transformed_pcd(output_filename, header, transformed_points):
    """写入变换后的PCD文件"""
    with open(output_filename, 'w') as f:
        # 写入头部
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        for key, value in header.items():
            if isinstance(value, list):
                f.write(f"{key} {' '.join(value)}\n")
            else:
                f.write(f"{key} {value}\n")
        
        # 写入点数据
        for point in transformed_points:
            f.write(f"{point[0]} {point[1]} {point[2]}\n")

def main(input_filename, output_filename, transform_matrix):
    """主函数：读取、变换、写入PCD文件"""
    with open(input_filename, 'r') as f:
        # 读取头部
        header = read_pcd_header(f)
        print(f"读取到点云数据，包含 {header['POINTS'][0]} 个点")
        
        # 读取点
        points = read_pcd_points(f, header)
        
        # 应用变换
        transformed_points = transform_points(points, transform_matrix)
        print("点云变换完成")
        
        # 写入结果
        write_transformed_pcd(output_filename, header, transformed_points)
        print(f"变换后的点云已保存到 {output_filename}")

if __name__ == "__main__":
    # 输入和输出文件路径
    input_pcd = '/home/gct/tempfinal_line_points.pcd'    # 替换为你的输入PCD文件路径
    output_pcd = '/home/gct/tempfinal_line_points11.pcd'  # 变换后的PCD文件路径
    
    # 定义变换矩阵 (旋转矩阵 + 平移向量)
    transform_matrix = np.array([
        [0.306968, -0.903160, 0.300122, 103.940971],
        [0.787380, 0.063865, -0.613151, -134.328033],
        [0.534607, 0.424527, 0.730734, -82.432396],
        [0.000000, 0.000000, 0.000000, 1.000000]
    ], dtype=np.float64)
    
    # 执行主函数
    main(input_pcd, output_pcd, transform_matrix)
