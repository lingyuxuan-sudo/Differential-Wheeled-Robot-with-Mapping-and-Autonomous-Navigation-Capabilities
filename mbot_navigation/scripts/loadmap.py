import cv2
import numpy as np
import yaml
from PIL import Image
def load_map(yaml_file):
    with open(yaml_file, 'r') as file:
        map_info = yaml.safe_load(file)
    
    # 读取图像文件
    image = cv2.imread(map_info['image'], cv2.IMREAD_GRAYSCALE)
    
    if image is None:
        raise FileNotFoundError(f"Failed to load image at {map_info['image']}")
    
    resolution = map_info['resolution']
    origin = map_info['origin']
    
    return image, resolution, origin

def generate_grid_map(image):
    grid_map = np.zeros_like(image, dtype=np.int8)
    
    # 设置栅格地图的值：障碍物、空闲区域、未知区域
    grid_map[image == 0] = 1  # 障碍物
    grid_map[image == 255] = 0  # 空闲区域
    grid_map[image == 205] = -1  # 未知区域
    
    return grid_map

# 加载地图
image, resolution, origin = load_map('d:/lyx/knowledge/ros_senior-main/mbot_navigation/maps/gmapping_save.yaml')

# 生成栅格地图
grid_map = generate_grid_map(image)

# 打印栅格地图，或者你可以将其保存为文件
print(grid_map)
