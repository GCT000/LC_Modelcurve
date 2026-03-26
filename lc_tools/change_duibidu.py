import cv2
import numpy as np
import matplotlib.pyplot as plt

def enhance_power_line_contrast(image_path, output_path=None, visualize=False):
    """
    增强输电线图片的对比度，专门针对输电线与天空背景对比度弱的场景
    
    参数:
        image_path: 输入图片路径
        output_path: 输出图片路径（可选）
        visualize: 是否可视化对比结果（True/False）
    返回:
        增强后的图像数组
    """
    # 读取图片，以彩色模式读取
    img = cv2.imread(image_path)
    if img is None:
        raise ValueError(f"无法读取图片，请检查路径: {image_path}")
    
    # 1. 转换到HSV色彩空间，分离亮度通道
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    
    # 2. 对亮度通道应用CLAHE增强对比度（避免全局直方图均衡化的过曝问题）
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    v_enhanced = clahe.apply(v)
    
    # 3. 合并HSV通道
    hsv_enhanced = cv2.merge([h, s, v_enhanced])
    img_enhanced = cv2.cvtColor(hsv_enhanced, cv2.COLOR_HSV2BGR)
    
    # 4. 转换为灰度图，进一步增强线条
    gray = cv2.cvtColor(img_enhanced, cv2.COLOR_BGR2GRAY)
    
    # 5. 使用形态学操作突出线性结构（针对输电线的细长特征）
    # 创建线性结构元素，方向可根据输电线方向调整（这里用垂直和水平）
    kernel_vertical = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 3))
    kernel_horizontal = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 1))
    
    # 膨胀操作突出线条
    gray_dilated_v = cv2.dilate(gray, kernel_vertical, iterations=1)
    gray_dilated_h = cv2.dilate(gray, kernel_horizontal, iterations=1)
    gray_dilated = cv2.addWeighted(gray_dilated_v, 0.5, gray_dilated_h, 0.5, 0)
    
    # 6. 锐化处理，让线条更清晰
    kernel_sharpen = np.array([[-1, -1, -1],
                               [-1,  9, -1],
                               [-1, -1, -1]])
    img_final = cv2.filter2D(gray_dilated, -1, kernel_sharpen)
    
    # 可选：保存增强后的图片
    if output_path:
        cv2.imwrite(output_path, img_final)
    
    # 可选：可视化原始图和增强后的对比
    if visualize:
        plt.figure(figsize=(12, 6))
        
        # 原始图
        plt.subplot(1, 2, 1)
        plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        plt.title('原始图片')
        plt.axis('off')
        
        # 增强后的图
        plt.subplot(1, 2, 2)
        plt.imshow(img_final, cmap='gray')
        plt.title('对比度增强后的图片')
        plt.axis('off')
        
        plt.tight_layout()
        plt.show()
    
    return img_final

# ------------------- 示例使用 -------------------
if __name__ == "__main__":
    # 请替换为你的图片路径
    input_image = "/home/gct/LC-CurveModel/data/1-13guangzhou/1314.png"    # 输入图片路径
    output_image = "/home/gct/LC-CurveModel/data/1-13guangzhou/1314_1.png" # 输出图片路径
    
    try:
        # 调用函数增强对比度，可视化对比结果
        enhanced_img = enhance_power_line_contrast(
            image_path=input_image,
            output_path=output_image,
            visualize=True
        )
        print(f"图片对比度增强完成！增强后的图片已保存至: {output_image}")
    except Exception as e:
        print(f"处理出错: {e}")
