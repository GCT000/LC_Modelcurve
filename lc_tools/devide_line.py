import open3d as o3d
import numpy as np
from scipy.spatial import KDTree

def extract_two_lines_isolated(pcd_file_path, 
                              angle_threshold=5.0,  # 两条线的最小夹角（度）
                              distance_threshold_xyz=(0.1, 0.005, 0.005),
                              ransac_num_iterations=5000):
    """
    彻底隔离两条线，避免第二条线混入第一条线的漏检点
    参数:
        pcd_file_path: PCD文件路径
        angle_threshold: 两条线的最小夹角（度），过滤方向相似的点
        distance_threshold_xyz: x/y/z轴差异化距离阈值
    """
    # 1. 加载原始点云
    print("加载原始PCD文件...")
    pcd = o3d.io.read_point_cloud(pcd_file_path)
    points_original = np.asarray(pcd.points)
    n_points = len(points_original)
    print(f"原始点云数量: {n_points}")
    if n_points == 0:
        raise ValueError("PCD文件为空或无法读取")
    
    # 2. 归一化点云（消除x轴大尺度影响）
    centroid = np.mean(points_original, axis=0)
    points_normalized = points_original - centroid
    scale = np.max(np.abs(points_normalized), axis=0)
    scale[scale == 0] = 1.0
    points_normalized = points_normalized / scale
    
    # 3. 增强版3D直线拟合（返回方向向量+内点）
    def fit_3d_line_enhanced(points_in, threshold_norm=0.01, iterations=5000):
        """返回：内点索引、直线中心点、直线方向向量"""
        best_inliers = []
        best_centroid = None
        best_dir = None
        
        for _ in range(iterations):
            # 随机选2个点
            if len(points_in) < 2:
                return [], None, None
            idx = np.random.choice(len(points_in), 2, replace=False)
            p1, p2 = points_in[idx]
            dir_vec = p2 - p1
            if np.linalg.norm(dir_vec) < 1e-6:
                continue
            dir_vec = dir_vec / np.linalg.norm(dir_vec)
            
            # 计算点到直线距离
            vec = points_in - p1
            cross = np.cross(vec, dir_vec)
            distances = np.linalg.norm(cross, axis=1)
            inliers = np.where(distances < threshold_norm)[0]
            
            if len(inliers) > len(best_inliers):
                best_inliers = inliers
                # 计算更精准的直线参数
                inlier_points = points_in[best_inliers]
                best_centroid = np.mean(inlier_points, axis=0)
                cov = np.cov(inlier_points.T)
                eig_vals, eig_vecs = np.linalg.eig(cov)
                best_dir = eig_vecs[:, np.argmax(eig_vals)]
        
        return best_inliers, best_centroid, best_dir
    
    # 4. 第一步：拟合第一条线（保留核心特征）
    print("提取第一条线（核心特征）...")
    inliers1, centroid1, dir1 = fit_3d_line_enhanced(
        points_normalized,
        threshold_norm=0.01,
        iterations=ransac_num_iterations
    )
    if len(inliers1) == 0:
        raise ValueError("未拟合出第一条线")
    # 生成第一条线点云
    line1_pcd = o3d.geometry.PointCloud()
    line1_pcd.points = o3d.utility.Vector3dVector(points_original[inliers1])
    line1_pcd.paint_uniform_color([1, 0, 0])
    print(f"第一条线提取点数: {len(inliers1)}")
    
    # 5. 关键：预处理剩余点云，过滤第一条线的漏检点
    print("过滤第一条线的漏检点...")
    # 剩余点云（归一化）
    remaining_points_norm = points_normalized[~np.isin(np.arange(n_points), inliers1)]
    remaining_points_original = points_original[~np.isin(np.arange(n_points), inliers1)]
    
    # 计算剩余点与第一条线的方向相似度+距离，过滤漏检点
    def filter_same_line_points(points_norm, line_centroid, line_dir, angle_thresh=5.0, dist_thresh=0.02):
        """
        过滤与目标线方向相似、距离近的点（即第一条线的漏检点）
        angle_thresh: 角度阈值（度），小于该值则判定为同方向
        dist_thresh: 距离阈值（归一化空间）
        """
        # 计算每个点到第一条线的距离
        vec = points_norm - line_centroid
        cross = np.cross(vec, line_dir)
        distances = np.linalg.norm(cross, axis=1)
        
        # 计算每个点与第一条线的方向夹角（度）
        point_dirs = vec / np.linalg.norm(vec, axis=1, keepdims=True)
        point_dirs[np.isnan(point_dirs)] = 0  # 处理零向量
        dot_products = np.dot(point_dirs, line_dir)
        angles = np.degrees(np.arccos(np.clip(np.abs(dot_products), 0, 1)))
        
        # 过滤条件：角度>阈值 OR 距离>阈值 → 保留；否则（同方向+近距离）→ 过滤
        filter_mask = (angles > angle_thresh) | (distances > dist_thresh)
        return filter_mask, remaining_points_norm[filter_mask], remaining_points_original[filter_mask]
    
    # 执行过滤，彻底移除第一条线的漏检点
    filter_mask, filtered_points_norm, filtered_points_original = filter_same_line_points(
        remaining_points_norm,
        centroid1,
        dir1,
        angle_thresh=angle_threshold,
        dist_thresh=0.02
    )
    print(f"过滤前剩余点数: {len(remaining_points_norm)}, 过滤后剩余点数: {len(filtered_points_norm)}")
    
    # 6. 第二步：拟合第二条线（仅用过滤后的点）
    print("提取第二条线（无第一条线混入）...")
    inliers2, centroid2, dir2 = fit_3d_line_enhanced(
        filtered_points_norm,
        threshold_norm=0.01,
        iterations=ransac_num_iterations
    )
    if len(inliers2) == 0:
        raise ValueError("未拟合出第二条线，请调整角度/距离阈值")
    
    # 生成第二条线点云（确保无第一条线漏检点）
    line2_pcd = o3d.geometry.PointCloud()
    line2_pcd.points = o3d.utility.Vector3dVector(filtered_points_original[inliers2])
    line2_pcd.paint_uniform_color([0, 1, 0])
    print(f"第二条线提取点数: {len(inliers2)}")
    
    # 7. 验证两条线的夹角（可选）
    dot_product = np.dot(dir1, dir2)
    line_angle = np.degrees(np.arccos(np.clip(np.abs(dot_product), 0, 1)))
    print(f"两条线的夹角: {line_angle:.2f} 度")
    
    # 8. 保存+可视化
    o3d.io.write_point_cloud("line1_isolated.pcd", line1_pcd)
    o3d.io.write_point_cloud("line2_isolated.pcd", line2_pcd)
    
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="彻底隔离的两条线")
    vis.add_geometry(line1_pcd)
    vis.add_geometry(line2_pcd)
    opt = vis.get_render_option()
    opt.light_on = False
    opt.background_color = np.array([0, 0, 0])
    opt.point_size = 3
    vis.run()
    vis.destroy_window()
    
    return line1_pcd, line2_pcd

# 主程序调用
if __name__ == "__main__":
    PCD_FILE = "1.pcd"  # 替换为你的PCD文件路径
    try:
        line1, line2 = extract_two_lines_isolated(
            PCD_FILE,
            angle_threshold=3.0,  # 可根据实际夹角调整（建议2-5度）
            distance_threshold_xyz=(0.1, 0.005, 0.005),
            ransac_num_iterations=5000
        )
        print("两条线已彻底隔离提取完成！")
    except Exception as e:
        print(f"处理出错: {e}")
