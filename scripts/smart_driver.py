import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class smartdriver(Node):
    def __init__(self):
        super().__init__('smart_driver',)

        #订阅传感器数据
        self.subscriptions = self.create_subscription(
            LaserScan,#数据类型
            '/scan',#数据来源
            self.scan_callback,#一有数据来就触发这个函数
            10#缓冲区：如果处理不过来，最多暂存10帧数据
            )
        
        self.publishers = self.create_publisher(
            Twist,#数据类型
            '/cmd_vel',#发布到哪里
            10#缓冲区
            )
        print('节点实例创建成功')
    def scan_callback(self ,msg):
        



