import open3d as o3d
import numpy as np
import argparse

def compute_nearest_neighbor_stats(source_file, target_file):
    """
    计算源点云中每个点到目标点云的最近点距离，并返回统计结果
    
    参数:
        source_file: 源PCD文件路径（需要计算距离的点云）
        target_file: 目标PCD文件路径（被比较的点云）
    """
    try:
        # 读取点云文件
        source_pcd = o3d.io.read_point_cloud(source_file)
        target_pcd = o3d.io.read_point_cloud(target_file)
        
        # 检查点云是否为空
        if not source_pcd.has_points():
            raise Exception(f"源点云文件不包含点: {source_file}")
        if not target_pcd.has_points():
            raise Exception(f"目标点云文件不包含点: {target_file}")
        
        print(f"源点云包含 {len(source_pcd.points)} 个点")
        print(f"目标点云包含 {len(target_pcd.points)} 个点")
        
        # 构建KDTree用于高效的最近邻搜索
        target_kdtree = o3d.geometry.KDTreeFlann(target_pcd)
        
        # 存储所有距离的列表
        distances = []
        
        # 对源点云中的每个点，查找目标点云中的最近点
        for point in source_pcd.points:
            # 查找最近的1个点
            [_, idx, dist] = target_kdtree.search_knn_vector_3d(point, 1)
            distances.append(np.sqrt(dist[0]))  # dist返回的是平方距离，需要开方
        
        # 转换为numpy数组以便计算统计值
        distances_np = np.array(distances)
        
        # 计算统计结果
        min_dist = np.min(distances_np)
        max_dist = np.max(distances_np)
        mean_dist = np.mean(distances_np)
        std_dist = np.std(distances_np)  # 标准差，作为额外统计信息
        
        # 输出结果
        print("\n距离统计结果:")
        print(f"最小距离: {min_dist:.6f}")
        print(f"最大距离: {max_dist:.6f}")
        print(f"平均距离: {mean_dist:.6f}")
        print(f"距离标准差: {std_dist:.6f}")
        
        return {
            "min": min_dist,
            "max": max_dist,
            "mean": mean_dist,
            "std": std_dist,
            "all_distances": distances_np
        }
        
    except FileNotFoundError as e:
        print(f"错误: 找不到文件 - {str(e)}")
    except Exception as e:
        print(f"处理过程中发生错误: {str(e)}")
    return None

if __name__ == "__main__":
    # 设置命令行参数
    parser = argparse.ArgumentParser(description='计算源点云中每个点到目标点云最近点的距离统计')
    parser.add_argument('source', help='源PCD文件路径（需要计算距离的点云）')
    parser.add_argument('target', help='目标PCD文件路径（被比较的点云）')
    
    args = parser.parse_args()
    
    # 执行计算
    compute_nearest_neighbor_stats(args.source, args.target)
    