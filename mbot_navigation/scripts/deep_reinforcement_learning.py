import numpy as np
import tensorflow as tf
from tensorflow.keras import models, layers, optimizers
import random
from collections import deque
import matplotlib.pyplot as plt


# 定义全局地图
global_map = np.array([
    [0, 0, 0, 1, 0],
    [1, 0, 1, 1, 0],
    [0, 0, 0, 0, 0],
    [1, 1, 1, 0, 1],
    [0, 0, 0, 0, 0]
])

# 定义环境类
class CoverageEnv:
    def __init__(self, grid):
        self.grid = grid
        self.size = grid.shape
        self.reset()
    
    def reset(self):
        self.position = (0, 0)
        self.visited = np.zeros_like(self.grid)
        self.visited[self.position] = 1
        return self._get_state()
    
    def _get_state(self):
        return np.concatenate([self.grid.flatten(), self.visited.flatten()])
    
    def step(self, action):
        x, y = self.position
        new_x, new_y = x, y
        
        if action == 0:   # 上
            new_x = max(x - 1, 0)
        elif action == 1: # 下
            new_x = min(x + 1, self.size[0] - 1)
        elif action == 2: # 左
            new_y = max(y - 1, 0)
        elif action == 3: # 右
            new_y = min(y + 1, self.size[1] - 1)
        
        reward = -0.5  # 默认惩罚
        
        if new_x == x and new_y == y:
            reward = -5  # 撞到边界的惩罚
        elif self.grid[new_x, new_y] == 1:
            reward = -5  # 撞到障碍物的惩罚
        elif self.grid[new_x, new_y] == 0 and self.visited[new_x, new_y] == 0:
            reward = 3  # 覆盖新区域的奖励
        
        self.position = (new_x, new_y)
        self.visited[new_x, new_y] = 1
        done = np.all(self.visited | self.grid == 1)
        
        return self._get_state(), reward, done

    
    def valid_actions(self):
        actions = []
        x, y = self.position
        if x > 0 and self.grid[x-1, y] == 0:  # 上
            actions.append(0)
        if x < self.size[0] - 1 and self.grid[x+1, y] == 0:  # 下
            actions.append(1)
        if y > 0 and self.grid[x, y-1] == 0:  # 左
            actions.append(2)
        if y < self.size[1] - 1 and self.grid[x, y+1] == 0:  # 右
            actions.append(3)
        return actions

# DQN Agent
class DQNAgent:
    def __init__(self, state_size, action_size):
        self.state_size = state_size
        self.action_size = action_size
        self.memory = deque(maxlen=2000)
        self.gamma = 0.95    # 折扣因子
        self.epsilon = 1.0   # 探索率
        self.epsilon_min = 0.1
        self.epsilon_decay = 0.95
        self.learning_rate = 0.01
        self.model = self._build_model()
    
    def _build_model(self):
        model = models.Sequential()
        model.add(layers.Dense(64, input_dim=self.state_size, activation='relu'))  # 第一隐藏层
        model.add(layers.Dense(64, activation='relu'))  # 第二隐藏层
        model.add(layers.Dense(64, activation='relu'))  # 第三隐藏层
        model.add(layers.Dense(32, activation='relu'))  # 第四隐藏层
        model.add(layers.Dense(self.action_size, activation='linear'))  # 输出层
        model.compile(loss='mse', optimizer=optimizers.Adam(learning_rate=self.learning_rate))
        return model

    
    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))
    
    def act(self, state):
        if np.random.rand() <= self.epsilon:
            return random.randrange(self.action_size)
        act_values = self.model.predict(state)
        return np.argmax(act_values[0])
    
    def replay(self, batch_size):
        minibatch = random.sample(self.memory, batch_size)
        for state, action, reward, next_state, done in minibatch:
            target = reward
            if not done:
                target = (reward + self.gamma * np.amax(self.model.predict(next_state)[0]))
            target_f = self.model.predict(state)
            target_f[0][action] = target
            self.model.fit(state, target_f, epochs=1, verbose=0)
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay
    
    def load(self, name):
        self.model.load_weights(name)
    
    def save(self, name):
        self.model.save_weights(name)

# 训练DQN模型并绘制路径
if __name__ == "__main__":
    env = CoverageEnv(global_map)
    state_size = env.reset().shape[0]
    action_size = 4  # 上下左右四个动作
    agent = DQNAgent(state_size, action_size)
    
    episodes = 50  # 最大训练回合数
    batch_size = 32
    max_steps = 30  # 每回合最大步数
    early_stop_threshold = 10  # 早停条件：在100个回合内没有显著进展
    best_score = -float('inf')
    stagnation_counter = 0

    for e in range(episodes):
        state = env.reset()
        state = np.reshape(state, [1, state_size])
        for time in range(max_steps):
            action = agent.act(state)
            next_state, reward, done = env.step(action)
            next_state = np.reshape(next_state, [1, state_size])
            agent.remember(state, action, reward, next_state, done)
            state = next_state
            if done:
                print(f"Episode: {e}/{episodes}, Score: {time}, Epsilon: {agent.epsilon:.2}")
                break
            if len(agent.memory) > batch_size:
                agent.replay(batch_size)

        # 检查是否有显著进展，用于早停
        if time > best_score:
            best_score = time
            stagnation_counter = 0
        else:
            stagnation_counter += 1
        
        if stagnation_counter >= early_stop_threshold:
            print("Early stopping triggered.")
            break

    # 评估模型并绘制路径
    state = env.reset()
    state = np.reshape(state, [1, state_size])
    path = [env.position]
    for _ in range(max_steps):
        action = agent.act(state)
        next_state, reward, done = env.step(action)
        path.append(env.position)
        state = np.reshape(next_state, [1, state_size])
        if done:
            break
    
    # 绘制地图和路径
    plt.imshow(global_map, cmap='gray_r', origin='lower')
    x_coords = [p[1] for p in path]
    y_coords = [p[0] for p in path]
    plt.plot(x_coords, y_coords, marker='o', color='blue', linestyle='-', linewidth=2, markersize=5)
    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.title("DQN Optimal Coverage Path")
    plt.grid(True)
    plt.show()
