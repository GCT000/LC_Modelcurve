from PIL import Image
import numpy as np

def calculate_average_brightness(image_path):
    """
    计算图片的平均亮度（灰度模式下）
    :param image_path: 图片路径
    :return: 平均亮度值（0-255）
    """
    # 打开图片并转换为灰度模式
    with Image.open(image_path).convert('L') as img:
        # 将图片转换为numpy数组以便计算
        img_array = np.array(img)
        # 计算平均亮度
        avg_brightness = np.mean(img_array)
    return avg_brightness

def adjust_brightness_to_match(source_path, target_path, output_path="adjusted_1.png"):
    """
    将源图片的亮度调整为与目标图片相同
    :param source_path: 待调整的图片路径（1.png）
    :param target_path: 目标亮度的图片路径（2.png）
    :param output_path: 调整后图片的保存路径
    """
    try:
        # 计算两张图片的平均亮度
        source_brightness = calculate_average_brightness(source_path)
        target_brightness = calculate_average_brightness(target_path)
        
        print(f"1.png 平均亮度: {source_brightness:.2f}")
        print(f"2.png 平均亮度: {target_brightness:.2f}")
        
        # 计算亮度调整系数
        if source_brightness == 0:
            # 避免除以0的情况
            adjustment_factor = 0
        else:
            adjustment_factor = target_brightness / source_brightness
        
        print(f"亮度调整系数: {adjustment_factor:.4f}")
        
        # 打开源图片（保留彩色通道）
        with Image.open(source_path) as img:
            # 转换为numpy数组进行像素操作
            img_array = np.array(img, dtype=np.float32)
            
            # 对每个通道的像素值应用调整系数
            adjusted_array = img_array * adjustment_factor
            
            # 确保像素值在0-255范围内（防止溢出）
            adjusted_array = np.clip(adjusted_array, 0, 255)
            
            # 转换回uint8类型并保存图片
            adjusted_img = Image.fromarray(adjusted_array.astype(np.uint8))
            adjusted_img.save(output_path)
            
            # 验证调整后的亮度
            final_brightness = calculate_average_brightness(output_path)
            print(f"调整后1.png的平均亮度: {final_brightness:.2f}")
            print(f"图片已保存至: {output_path}")
            
    except FileNotFoundError:
        print("错误：找不到指定的图片文件，请检查文件路径是否正确")
    except Exception as e:
        print(f"发生错误：{e}")

# 主程序执行
if __name__ == "__main__":
    # 输入图片路径（这里默认使用1.png和2.png，你可以根据需要修改）
    source_image = "/home/gct/LC-CurveModel/data/1-13guangzhou/1314_1.png"
    target_image = "/home/gct/LC-CurveModel/data/1-13guangzhou/1224_1.png"
    
    # 调用函数调整亮度
    adjust_brightness_to_match(source_image, target_image)
