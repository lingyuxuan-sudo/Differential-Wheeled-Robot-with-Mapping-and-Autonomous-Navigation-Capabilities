from PIL import Image
import numpy as np

image_path = "d:/lyx/knowledge/ros_senior-main/mbot_navigation/maps/gmapping_save.pgm"

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
