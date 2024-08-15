import numpy as np
import matplotlib.pyplot as plt





class SpiralSTC:
    def __init__(self, grid_map, robot_size):
        self.grid_map = grid_map
        self.robot_size = robot_size
        self.visited = np.zeros_like(grid_map)  # 用于跟踪已访问的单元格
        self.path = []

    def generate_tree(self):
        # 生成初始的生成树，选择起点
        start_x, start_y = 0, 0  # 假设从(0,0)开始
        self.visit(start_x, start_y)
        self.spiral_out(start_x, start_y)
    
    def visit(self, x, y):
        if self.is_valid(x, y):
            self.visited[x, y] = 1
            self.path.append((x, y))
    
    def is_valid(self, x, y):
        # 检查是否在地图范围内且该单元格可访问
        return 0 <= x < self.grid_map.shape[0] and 0 <= y < self.grid_map.shape[1] and self.grid_map[x, y] == 0
    


    def spiral_out(self, x, y):
        # 实现螺旋形路径生成逻辑，遍历每一个邻近的单元格
        directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]  # 向右、向上、向左、向下
        d = 0  # 起始方向

        while True:
            a =0
            new_x, new_y = x + directions[d][0], y + directions[d][1]
            if self.is_valid(new_x, new_y) and not self.visited[new_x, new_y]:
                self.visit(new_x, new_y)
                x, y = new_x, new_y
                # d = (d + 1) % 4  # 顺时针改变方向
            else:
                d = (d + 1) % 4  # 改变方向，寻找新的方向
                a += 1
                if a == 4:
                    break  # 如果回到了起始方向，说明当前区域已覆盖完毕

    def get_path(self):
        return self.path
# 假设grid_map是一个numpy数组，表示已知的地图，其中0表示空闲区域，1表示障碍物
grid_map = np.zeros((10, 10))  # 创建一个简单的10x10的空闲地图
robot_size = 0.4  # 假设机器人直径为0.4米

spiral_stc = SpiralSTC(grid_map, robot_size)
spiral_stc.generate_tree()
path = spiral_stc.get_path()
print(path)

# 如果有地图背景（如从pgm文件读取的地图），可以加载并作为背景显示
# 假设地图大小为 (10, 10)，你可以根据实际地图尺寸进行调整
map_image = np.ones((10, 10))  # 假设一个全白色背景（空闲区域）

plt.imshow(map_image, cmap='gray', origin='lower')

# 将路径绘制在地图上
x_coords = [point[0] for point in path]
y_coords = [point[1] for point in path]

plt.plot(x_coords, y_coords, marker='o', color='blue', linestyle='-', linewidth=2, markersize=5)

# 显示路径点的顺序
for i, (x, y) in enumerate(path):
    plt.text(x, y, str(i+1), fontsize=12, ha='right')

# 设置轴的标签和标题
plt.xlabel("X Coordinate")
plt.ylabel("Y Coordinate")
plt.title("Robot Path")

# 显示网格
plt.grid(True)

# 显示绘制的图像
plt.show()

""" class BSA(SpiralSTC):
    def backtrack(self):
        # 实现回溯逻辑，当遇到障碍物或边界时回溯
        while self.path:
            x, y = self.path.pop()
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                new_x, new_y = x + dx, y + dy
                if self.is_valid(new_x, new_y) and not self.visited[new_x, new_y]:
                    self.spiral_out(new_x, new_y)
                    return

    def generate_tree(self):
        start_x, start_y = 0, 0
        self.visit(start_x, start_y)
        self.spiral_out(start_x, start_y)
        self.backtrack()

bsa = BSA(grid_map, robot_size)
bsa.generate_tree()
path = bsa.get_path()
print(path) """



