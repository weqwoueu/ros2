import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import torch
from stable_baselines3 import PPO

class aidriver(Node):
    def __init__(self,):
        super().__init__('rl_driver')

        #加载训练好的模型
        modle_path = '/home/liu/learn/ros2/models/rl_driver_model.zip' #modle路径

        try:
            self.model = PPO.load(modle_path)
            print('已找到modle_path')
        except:
            print('未找到modle_path')
            self.model = None

        self.publisher = self.create_publisher(Twist,'/cmd_vel',10)
        self.subscription = self.create_subscription(LaserScan,'/scen',self.callback,10)
    def callback(self,msg):
        if self.model == None:
            return
        
        #数据处理长360列表->长24张量
        #ranges = [l if l<=10.0 else 10.0 for l in msg.ranges]#转inf 为 10.0
        ranges_np =np.array(msg.ranges)
        ranges_np = np.nan_to_num(ranges_np ,posinf=10.0)#np写法意思同上#转inf 为 10.0

        step = len(ranges_np)//24
        observation = []
        for i in range(0 , len(ranges_np), step):
            sector = ranges_np[i:i+step]
            if len(sector) > 0:
                observation.append(np.min(sector))
            else:
                observation.append(10.0)
        observation = np.array(observation[:24])#防止obser爆格曲前24个

        action ,_states = self.model.predict(observation , deterministic=True)# deterministic=True: 考试模式，不要随机探索，选概率最大的动作
        cmd = Twist()#创建空白指令
        









