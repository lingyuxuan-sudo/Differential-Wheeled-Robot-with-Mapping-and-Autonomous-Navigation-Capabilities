import numpy as np
import networkx as nx

def build_graph_from_map(occupancy_grid, robot_diameter, resolution):
    grid_size = int(robot_diameter / resolution)
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    data = np.array(occupancy_grid.data).reshape((height, width))
    
    G = nx.Graph()

    # 遍历整个地图，生成节点
    for i in range(0, height, grid_size):
        for j in range(0, width, grid_size):
            cell_value = data[i:i+grid_size, j:j+grid_size]
            # 如果这个区域内没有障碍物，添加一个节点
            if np.all(cell_value == 0):  # 0 表示空闲
                node_id = (i//grid_size, j//grid_size)
                G.add_node(node_id, pos=(j + grid_size//2, i + grid_size//2))
                
                # 检查与相邻区域的连通性
                if i >= grid_size:
                    if np.all(data[i-grid_size:i, j:j+grid_size] == 0):
                        G.add_edge((i//grid_size, j//grid_size), ((i-grid_size)//grid_size, j//grid_size))
                if j >= grid_size:
                    if np.all(data[i:i+grid_size, j-grid_size:j] == 0):
                        G.add_edge((i//grid_size, j//grid_size), (i//grid_size, (j-grid_size)//grid_size))

    return G

# 示例使用：
# occupancy_grid 是通过SLAM获得的 OccupancyGrid 消息
# robot_diameter 是机器人的直径 (0.4 m)
# resolution 是地图的分辨率 (例如 0.05 m/cell)

graph = build_graph_from_map(occupancy_grid, 0.4, 0.05)

# 打印生成的图的节点和边
print("Nodes: ", graph.nodes)
print("Edges: ", graph.edges)
