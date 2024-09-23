import cv2
import sys

# 初始化变量
points = []
file_name = 'points1.txt'

def on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        # 添加选定的点并显示
        points.append((x, y))
        cv2.circle(image, (x, y), 3, (0, 0, 255), -1)
        cv2.imshow('image', image)

def save_points_to_file():
    with open(file_name, 'w') as f:
        for point in points:
            f.write(f"{point[0]}, {point[1]}\n")
    print(f"Points saved to {file_name}")

# 载入图像并缩小尺寸
image_path = sys.argv[1]
image = cv2.imread(image_path)
image = cv2.resize(image, (image.shape[1], image.shape[0]))

cv2.imshow('image', image)
cv2.setMouseCallback('image', on_mouse)

while True:
    key = cv2.waitKey(1) & 0xFF
    if key == ord('s'):  # 按下 's' 键保存点
        save_points_to_file()
        break
    elif key == 27:  # 按下 'ESC' 键退出
        break

cv2.destroyAllWindows()
