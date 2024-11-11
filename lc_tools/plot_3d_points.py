import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 读取txt文件
file_path = '../simulation/data/points.txt'
points = []
with open(file_path, 'r') as file:
    for line in file:
        x, y, z = map(float, line.strip().split())
        points.append((x, y, z))

# 可视化点云
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
for point in points:
    ax.scatter(point[0], point[1], point[2], c='b', marker='o', s=5)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# 设置坐标范围
ax.set_xlim([-5, 205])
ax.set_ylim([-50, 50])
ax.set_zlim([0, 100])


plt.show()
