import numpy as np
import matplotlib.pyplot as plt
import os
import yaml
from PIL import Image
import numpy as np

# YAML 文件路径
yaml_path = r"E:\大学作业\nader_project\robot\Differential-Wheeled-Robot-with-Mapping-and-Autonomous-Navigation-Capabilities\mbot_navigation\maps\gmapping_save.yaml"

# 读取 YAML 文件
with open(yaml_path, 'r') as file:
    yaml_data = yaml.safe_load(file)

# 获取相对图像路径并转换为绝对路径
image_path = os.path.join(os.path.dirname(yaml_path), yaml_data['image'])

# 加载图像
try:
    with Image.open(image_path) as img:
        image = np.array(img)
        print("Image loaded successfully")
except Exception as e:
    print(f"Failed to load image: {e}")

def generate_grid_map(image):
    # 创建与图像大小相同的栅格地图
    grid_map = np.zeros_like(image, dtype=np.int8)
    
    # 将像素值转换为栅格地图的值
    grid_map[image == 0] = 1  # 障碍物
    grid_map[image == 255] = 0  # 空闲区域
    grid_map[image == 205] = -1  # 未知区域
    
    return grid_map

# 生成栅格地图
grid_map = generate_grid_map(image)
# 打印或进一步处理栅格地图
print(grid_map)
def downsample_grid_map(grid_map, robot_diameter, resolution):
    # 计算重采样因子（新栅格单元格大小相对于原栅格单元格大小）
    factor = int(robot_diameter / resolution)
    
    # 获取原始地图的尺寸
    height, width = grid_map.shape
    
    # 计算新的地图尺寸
    new_height = height // factor
    new_width = width // factor
    
    # 创建新的栅格地图
    new_grid_map = np.zeros((new_height, new_width), dtype=np.int8)
    
    # 对每个区块进行重采样
    for i in range(new_height):
        for j in range(new_width):
            # 获取原地图中相应的区块
            block = grid_map[i*factor:(i+1)*factor, j*factor:(j+1)*factor]
            
            # 统计每个类别的数量
            num_obstacle = np.sum(block == 1)
            num_free = np.sum(block == 0)
            num_unknown = np.sum(block == -1)
            total_cells = block.size
            # 对每个区块进行重采样

            # 获取原地图中相应的区块
            block = grid_map[i*factor:(i+1)*factor, j*factor:(j+1)*factor]
            
            if num_obstacle > 0:
                new_grid_map[i, j] = 1
            # 如果区块中所有单元格都是空闲区域，则标记为空闲区域
            elif num_free == total_cells:
                new_grid_map[i, j] = 0
            # 如果未知区域超过70%，则标记为未知区域，否则视为自由区域
            elif num_unknown / total_cells > 0.9:
                new_grid_map[i, j] = -1
            else:
                new_grid_map[i, j] = 0
    
    return new_grid_map

# 示例使用
robot_diameter = 0.4  # 机器人的直径
resolution = 0.05  # 原始地图的分辨率

# 生成降采样后的栅格地图
grid_map = downsample_grid_map(grid_map, robot_diameter, resolution)

class CoveragePathWithObstacleAvoidance:
    def __init__(self, grid_map):
        self.grid_map = grid_map
        self.visited = np.zeros_like(grid_map)  # 用于跟踪已访问的单元格
        self.path = []
        self.directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # 顺时针方向：右，下，左，上

    def is_valid(self, x, y):
        # 检查是否在地图范围内且该单元格可访问
        return 0 <= x < self.grid_map.shape[0] and 0 <= y < self.grid_map.shape[1] and self.grid_map[x, y] == 0

    def visit(self, x, y):
        self.visited[x, y] = 1
        self.path.append((x, y))

    def spiral_coverage(self, start_x, start_y):
        stack = [(start_x, start_y)]
        while stack:
            x, y = stack.pop()
            if not self.visited[x, y]:
                self.visit(x, y)
                for direction in self.directions:
                    new_x, new_y = x + direction[0], y + direction[1]
                    if self.is_valid(new_x, new_y) and not self.visited[new_x, new_y]:
                        stack.append((new_x, new_y))

    def get_path(self):
        return self.path


coverage = CoveragePathWithObstacleAvoidance(grid_map)
coverage.spiral_coverage(32, 68)
path = coverage.get_path()

# 显示地图和路径
plt.imshow(grid_map, cmap='gray_r', origin='lower')

# 将路径绘制在地图上
x_coords = [point[0] for point in path]
y_coords = [point[1] for point in path]

plt.plot(y_coords, x_coords, marker='o', color='blue', linestyle='-', linewidth=2, markersize=5)

# 显示路径点的顺序
# for i, (x, y) in enumerate(path):
#     plt.text(y, x, str(i+1), fontsize=12, ha='right')

# 设置轴的标签和标题
plt.xlabel("X Coordinate")
plt.ylabel("Y Coordinate")
plt.title("Robot Coverage Path with Obstacle Avoidance")

# 显示网格
plt.grid(True)

# 显示绘制的图像
plt.show()
