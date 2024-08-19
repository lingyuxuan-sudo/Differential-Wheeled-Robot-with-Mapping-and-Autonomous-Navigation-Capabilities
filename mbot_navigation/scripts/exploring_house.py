#!/usr/bin/env python3

import rospy
import actionlib
from actionlib_msgs.msg import *  
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  
import numpy as np
import matplotlib.pyplot as plt
import os
import yaml
from PIL import Image
import numpy as np
from actionlib_msgs.msg import GoalStatus

def convert_grid_to_world(i, j, resolution, origin):
    x = i * resolution + origin[0] + 0.2
    y = j * resolution + origin[1] + 0.2 
    return x, y

def getpath():
  # YAML 文件路径
  # yaml_path = r"mbot_navigation/maps/gmapping_save.yaml"
  yaml_path = "/home/yuxuan/catkin_ws/src/mbot_navigation/maps/gmapping_save.yaml"


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
              
              # 如果区块中有障碍物（值为1），则新单元格标记为障碍物
              if np.any(block == 1):
                  new_grid_map[i, j] = 1
              # 如果区块中所有单元格都是空闲区域，则标记为空闲区域
              elif np.all(block == 0):
                  new_grid_map[i, j] = 0
              # 否则，标记为未知区域
              else:
                  new_grid_map[i, j] = -1
      
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
          height, width = grid_map.shape
          origin =(-(width*0.4)/2,-(height*0.4)/2)
          return self.path, origin


  coverage = CoveragePathWithObstacleAvoidance(grid_map)
  height, width = grid_map.shape
  coverage.spiral_coverage(50,50)
  return coverage.get_path()   


def navigate_to_goals(grid_path, resolution, origin):
    # 初始化目标列表
    target_list = []
    
    # 转换每个网格点到世界坐标并生成目标点列表
    for i, j in grid_path:
        x, y = convert_grid_to_world(i, j, resolution, origin)
        target_list.append(Pose(Point(x, y, 0.000), Quaternion(0.000, 0.000, 0.645, 0.764)))

    # # 初始化 move_base 的 Action 客户端
    # move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    # move_base_client.wait_for_server()

    # 遍历所有目标点
    for i, target in enumerate(target_list):
        start_time = rospy.Time.now()  
        
        goal = MoveBaseGoal()
        goal.target_pose.pose = target
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        
        rospy.loginfo("Going to goal {0}: {1}".format(i, str(target)))
        
        move_base_client.send_goal(goal)
        
        finished_within_time = move_base_client.wait_for_result(rospy.Duration(300))
        
        if not finished_within_time:
            move_base_client.cancel_goal()
            rospy.loginfo("Time out. Failed to reach goal {0}, moving to the next goal.".format(i))
            continue  # 跳到下一个目标点
        else:
            running_time = (rospy.Time.now() - start_time).to_sec()
            if move_base_client.get_state() == GoalStatus.SUCCEEDED:
                rospy.loginfo("Reached goal {0} successfully, runtime: {1} sec".format(i, running_time))
            else:
                rospy.loginfo("Failed to reach goal {0}, moving to the next goal.".format(i))
                continue  # 跳到下一个目标点



def main():
  rospy.init_node("move_test", anonymous=True)
  
  move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
  
  rospy.loginfo("Waiting for move_base action server...")  
  
  while move_base_client.wait_for_server(rospy.Duration(5.0)) == 0:
    rospy.loginfo("connected to move base server")
  
  # target_list = []
  # target_list.append(Pose(Point(6.543, 4.779, 0.000), Quaternion(0.000, 0.000, 0.645, 0.764)))
  # target_list.append(Pose(Point(5.543, -4.779, 0.000), Quaternion(0.000, 0.000, 0.645, 0.764)))
  # target_list.append(Pose(Point(-5.543, 4.779, 0.000), Quaternion(0.000, 0.000, 0.645, 0.764)))
  # target_list.append(Pose(Point(-5.543, -4.779, 0.000), Quaternion(0.000, 0.000, 0.645, 0.764)))
#   grid_path = [(50, 50), (52, 50), (52, 48), (50, 48)]
  grid_path,origin = getpath()
  target_list = []
  resolution = 0.4
  
  # for i, j in grid_path:
  #       x, y = convert_grid_to_world(i, j, resolution, origin)
  #       target_list.append(Pose(Point(x, y, 0.000), Quaternion(0.000, 0.000, 0.645, 0.764)))


  navigate_to_goals(grid_path, resolution, origin)
  # for i, target in enumerate(target_list):
  #   start_time = rospy.Time.now()  
    
  #   goal = MoveBaseGoal()
  #   goal.target_pose.pose = target
  #   goal.target_pose.header.frame_id = 'map'
  #   goal.target_pose.header.stamp = rospy.Time.now()
    
  #   rospy.loginfo("going to {0} goal, {1}".format(i, str(target)))
    
  #   move_base_client.send_goal(goal)
    
  #   finished_within_time = move_base_client.wait_for_result(rospy.Duration(300))
    
  #   if not finished_within_time:
  #     move_base_client.cancel_goal()
  #     rospy.loginfo("time out, failed to goal")
  #   else:
  #     running_time = (rospy.Time.now() - start_time).to_sec()
  #     if move_base_client.get_state() == GoalStatus.SUCCEEDED:
  #       rospy.loginfo("go to {0} goal succeeded, run time: {1} sec".format(i, running_time))
  #     else:
  #       rospy.loginfo("goal failed")
    

if __name__ == "__main__":
  main()
