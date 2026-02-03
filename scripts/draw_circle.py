import rclpy
from rclpy.node import Node
# 导入乌龟能听懂的消息格式 (Twist: 线速度+角速度)
from geometry_msgs.msg import Twist

class Circledrawer(Node):
    def __init__(self):
        super().__init__('circle_drawer_node')
        # 2. 创建发布者 (Publisher)
        # 含义：我要对着 '/turtle1/cmd_vel' 这个大喇叭喊话
        # 消息类型是 Twist，队列长度 10
        self.publisher_ = self.create_publisher(Twist,'/turtle1/cmd_vel',10,)
        #swdad          dsadsdas