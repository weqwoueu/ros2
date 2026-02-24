import gymnasium as gym
from gymnasium import spaces
import numpy as np
from stable_baselines3 import PPO

class simplelidarenv(gym.Env):
    def __init__(self):
        super().__init__()
        #感觉器官
        #24个雷达数据范围从0到10
        self.observation_space = spaces.Box(low= 0.0 , high= 10.0 ,shape=(24,),dtype=np.float32)
        #运动神经
        #线速度和角速度范围从-1到1
        self.action_space = spaces.Box(low= -1.0,high=1.0,shape=(2,),dtype=np.float32)

        #清空场地以及步数归零
        self.current_scan = np.full(24,10.0)# 初始视野很开阔，全是 10 米
        self.step_count = 0


    #500步后的重生函数#
    # 新版的 Gym 强制要求 reset 必须接受 seed 和 options 两个默认参数。就算你代码里不用，也得写在括号里，否则底层调用时会报 TypeError（参数数量不匹配）。
    def reset(self , seed = None, options = None):
        super().reset(seed=seed)
        self.step_count = 0
        #随机生成考场地形 
        #生成 24 个介于 2.0 到 10.0 之间的随机小数
        self.current_scan = np.random.uniform(2.0,10.0,24)

            #随机将24个墙里面的一个突然放到0.5米上
            #让它一出生就必须学会猛打方向盘逃生
        wall_idx = np.random.randint(0,24)
        self.current_scan[wall_idx]= 0.5 
            #返回生成的24个墙
        return self.current_scan.astype(np.float32),{}#.astype(np.float32)强行转化为与observation_space同格式#一个空字典 {}。甲方规定的预留接口，不能省。
    

        
    def step(self , action):
        self.step_count += 1
            #线速度和角速度
        v = action[0]
        w = action[1]           

            #定义物理规则（非常暴力前进就有墙靠近，转弯就有最近的墙远离）
        self.current_scan -= (v*0.1)#有v时所有雷达点都减小
        min_idx = np.argmin(self.current_scan)#找出最近的墙
        if abs(w)>= 0.2:
            self.current_scan[min_idx]+=abs(w)*0.2#远离最近的墙
            
            #加点噪音和限幅
        self.current_scan += np.random.normal(0,0.05,24)
        self.current_scan += np.clip(self.current_scan,0.0,10.0)

            #奖励函数
        min_dis = np.min(self.current_scan)

        reward = 0.0
        terminated = False

        reward += v*1.0#跑得快加分

        reward -= abs(w)*0.05#转弯多扣分

        if min_dis <= 0.5:#撞墙似了扣50
            reward -= 200
            terminated = True
        elif min_dis <= 1.0:#太近了也扣分
            reward -= (1.0/min_dis)*18.0
            
        truncated = self.step_count >= 500

        return self.current_scan.astype(np.float32),float(reward),terminated,truncated,{} #观测 (Array), 奖励 (Float), 正常结束 (Bool), 超时结束 (Bool), 附加信息 (Dict)
       
    
#开始训练
if __name__ == '__main__':
    env = simplelidarenv()
    print('开始训练')
    model = PPO("MlpPolicy",env,verbose=1,learning_rate=0.0003)# verbose=1 打印日志

    model.learn(total_timesteps=200000)
    model.save("/home/liu/learn/ros2/models/test_model.zip")
    print("训练完成保存为/home/liu/learn/ros2/models/test_model.zip")

        

            





            








