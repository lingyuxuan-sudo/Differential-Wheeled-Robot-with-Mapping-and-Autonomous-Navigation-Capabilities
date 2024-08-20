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
            # 如果未知区域超过90%，则标记为未知区域，否则视为自由区域
            elif num_unknown / total_cells > 0.9:
                new_grid_map[i, j] = -1
            else:
                new_grid_map[i, j] = 0
    
    return new_grid_map

# 示例使用
robot_diameter = 0.4  # 机器人的直径
resolution = 0.05  # 原始地图的分辨率

# 生成降采样后的栅格地图
downsampled_grid_map = downsample_grid_map(grid_map, robot_diameter, resolution)

# 打印或进一步处理降采样后的栅格地图
print(downsampled_grid_map)




# 假设 grid_map 是生成的栅格地图
def save_grid_map_as_image(grid_map, output_path):
    # 将栅格地图的值映射到 0-255 之间，用于保存为灰度图像
    # 例如：0 -> 255 (空闲), 1 -> 0 (障碍物), -1 -> 127 (未知)
    img_data = np.zeros_like(grid_map, dtype=np.uint8)
    img_data[grid_map == 0] = 255   # 空闲区域
    img_data[grid_map == 1] = 0     # 障碍物
    img_data[grid_map == -1] = 127  # 未知区域

    # 将 NumPy 数组转换为图像对象
    img = Image.fromarray(img_data)
    
    # 保存图像
    img.save(output_path)
    print(f"Grid map saved as {output_path}")

# 使用示例
save_grid_map_as_image(grid_map, "mbot_navigation\scripts\output_grid_map.png")
save_grid_map_as_image(downsampled_grid_map, "mbot_navigation\scripts\downsampled_grid_map.png")
