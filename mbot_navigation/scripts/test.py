import numpy as np
import matplotlib.pyplot as plt

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

# # 创建一个简单的10x10的空闲地图，1表示障碍物
# grid_map = np.zeros((10, 10))

# # 添加一些障碍物
# grid_map[3:7, 4] = 1
# grid_map[7, 2:5] = 1
def generate_random_grid(size=20, obstacle_probability=0.2):
    """
    生成一个随机的grid，其中1表示障碍物，0表示空闲区域。
    
    :param size: grid的大小，默认为100x100
    :param obstacle_probability: 每个单元格为障碍物的概率，默认为0.2（20%）
    :return: 随机生成的grid_map
    """
    grid_map = np.random.choice([0, 1], size=(size, size), p=[1-obstacle_probability, obstacle_probability])
    return grid_map

# 生成一个100x100的随机grid，障碍物概率为20%
grid_map = generate_random_grid()
coverage = CoveragePathWithObstacleAvoidance(grid_map)
coverage.spiral_coverage(0, 0)
path = coverage.get_path()

# 显示地图和路径
plt.imshow(grid_map, cmap='gray_r', origin='lower')

# 将路径绘制在地图上
x_coords = [point[0] for point in path]
y_coords = [point[1] for point in path]

plt.plot(y_coords, x_coords, marker='o', color='blue', linestyle='-', linewidth=2, markersize=5)

# 显示路径点的顺序
for i, (x, y) in enumerate(path):
    plt.text(y, x, str(i+1), fontsize=12, ha='right')

# 设置轴的标签和标题
plt.xlabel("X Coordinate")
plt.ylabel("Y Coordinate")
plt.title("Robot Coverage Path with Obstacle Avoidance")

# 显示网格
plt.grid(True)

# 显示绘制的图像
plt.show()
