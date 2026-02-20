import gymnasium as gym
from gymnasium import spaces
import numpy as np
from stable_baselines3 import PPO


#生成一个用于测试driver.py的模型

class Env(gym.Env):
    def __init__(self):
        super().__init__()
        # 眼睛：24个雷达数据 (0 到 10米)
        self.observation_space = spaces.Box(low=0.0 ,high=10.0, shape=(24,) ,dtype=np.float32)
        # 腿：2个动作 (油门和方向盘，-1 到 1)
        self.action_space = spaces.Box(low=-1.0 ,high=1.0, shape=(2,) ,dtype=np.float32)
    def step(self, action):
        return self.observation_space.sample(),0.0,False,False,{}
    
    def reset(self,  seed = None):
        return self.observation_space.sample(), {}

env = Env()
model = PPO("MlpPolicy", env ,verbose=1)
model.learn(total_timesteps=10)

model.save("test_model")
print("模型保存为test_model.zip")
        