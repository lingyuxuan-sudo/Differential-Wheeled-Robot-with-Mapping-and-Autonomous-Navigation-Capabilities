import numpy as np
import networkx as nx
import random
import matplotlib.pyplot as plt

# 示例网格 (0 表示可通行, 1 表示障碍物)
grid = np.array([
    [0, 0, 0, 1, 0],
    [1, 0, 1, 1, 0],
    [0, 0, 0, 0, 0],
    [1, 1, 1, 0, 1],
    [0, 0, 0, 0, 0]
])

def generate_random_grid(size=100, obstacle_probability=0.2):
    grid_map = np.random.choice([0, 1], size=(size, size), p=[1-obstacle_probability, obstacle_probability])
    return grid_map

def extract_graph_from_grid(grid):
    G = nx.Graph()
    rows, cols = grid.shape

    for i in range(rows):
        for j in range(cols):
            if grid[i, j] == 0:
                G.add_node((i, j))

    for i in range(rows):
        for j in range(cols):
            if grid[i, j] == 0:
                for ni, nj in [(i-1, j), (i+1, j), (i, j-1), (i, j+1)]:
                    if 0 <= ni < rows and 0 <= nj < cols and grid[ni, nj] == 0:
                        G.add_edge((i, j), (ni, nj), weight=1)

    return G

def calculate_distance_matrix_from_graph(G):
    nodes = list(G.nodes)
    n = len(nodes)
    distance_matrix = np.full((n, n), np.inf)

    for i in range(n):
        for j in range(i, n):
            if i == j:
                distance_matrix[i][j] = 0
            else:
                try:
                    distance_matrix[i][j] = nx.dijkstra_path_length(G, nodes[i], nodes[j])
                    distance_matrix[j][i] = distance_matrix[i][j]
                except nx.NetworkXNoPath:
                    distance_matrix[i][j] = np.inf

    return nodes, distance_matrix

def genetic_algorithm(distance_matrix, population_size, elite_size, mutation_rate, generations):
    population = initial_population(population_size, len(distance_matrix))
    for _ in range(generations):
        population = next_generation(population, distance_matrix, elite_size, mutation_rate)
    best_route = min(population, key=lambda route: calculate_fitness(route, distance_matrix))
    return best_route, calculate_fitness(best_route, distance_matrix)

def initial_population(population_size, num_points):
    return [random.sample(range(num_points), num_points) for _ in range(population_size)]

def calculate_fitness(route, distance_matrix):
    total_distance = sum(distance_matrix[route[i], route[i + 1]] for i in range(len(route) - 1))
    total_distance += distance_matrix[route[-1], route[0]]
    return total_distance

def rank_routes(population, distance_matrix):
    return sorted(population, key=lambda route: calculate_fitness(route, distance_matrix))

def selection(population, elite_size):
    return population[:elite_size]

def breed(parent1, parent2):
    child = [None] * len(parent1)
    start, end = sorted(random.sample(range(len(parent1)), 2))
    child[start:end] = parent1[start:end]
    ptr = 0
    for gene in parent2:
        if gene not in child:
            while child[ptr] is not None:
                ptr += 1
            child[ptr] = gene
    return child

def mutate(route, mutation_rate):
    for i in range(len(route)):
        if random.random() < mutation_rate:
            swap_with = random.randint(0, len(route) - 1)
            route[i], route[swap_with] = route[swap_with], route[i]
    return route

def next_generation(population, distance_matrix, elite_size, mutation_rate):
    population = rank_routes(population, distance_matrix)
    elite = selection(population, elite_size)
    offspring = [breed(random.choice(elite), random.choice(elite)) for _ in range(len(population) - elite_size)]
    offspring = [mutate(child, mutation_rate) for child in offspring]
    return elite + offspring

def validate_path(route, nodes, grid):
    for i in range(len(route) - 1):
        node_a = nodes[route[i]]
        node_b = nodes[route[i+1]]
        if not nx.has_path(G, node_a, node_b):
            return False
    return True

grid_map = generate_random_grid(size=15, obstacle_probability=0.2)
G = extract_graph_from_grid(grid_map)
nodes, distance_matrix = calculate_distance_matrix_from_graph(G)

pop_size = 100
elite_size = 20
mutation_rate = 0.01
generations = 500
best_route, best_distance = genetic_algorithm(distance_matrix, pop_size, elite_size, mutation_rate, generations)

# 确保路径有效
while not validate_path(best_route, nodes, grid_map):
    best_route, best_distance = genetic_algorithm(distance_matrix, pop_size, elite_size, mutation_rate, generations)

best_route_nodes = [nodes[i] for i in best_route]
print("最佳路径节点坐标:", best_route_nodes)
print("最短距离:", best_distance)

plt.imshow(grid_map, cmap='gray_r', origin='lower')
x_coords = [point[1] for point in best_route_nodes]
y_coords = [point[0] for point in best_route_nodes]
plt.plot(x_coords, y_coords, marker='o', color='blue', linestyle='-', linewidth=2, markersize=5)
plt.xlabel("X Coordinate")
plt.ylabel("Y Coordinate")
plt.title("Optimized TSP-based Coverage Path with Obstacle Avoidance")
plt.grid(True)
plt.show()
