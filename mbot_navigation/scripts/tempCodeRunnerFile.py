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
