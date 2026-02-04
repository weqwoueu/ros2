import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math


class smartturtle(Node):
    def __init__(self):
        super().__init__('smart_turtle_node')
        # 1. 创建发布者 (嘴巴)：用来发运动指令
        self.publisher_ = self.create_publisher(Twist,'/turtle1/cmd_vel',10)
        self.last_th = None
        
        #创建订阅者
        #订阅/turtle1/pose这个topic
        # 频率：一收到消息，马上触发 self.pose_callback 函数待会要写的（处理逻辑）
        #10 (缓冲区/队列大小)
        #Pose数据类型，'/turtle1/pose'从哪里订阅，self.pose_callback意思是把这个函数的地址（指针）交给 ROS 系统
        self.subscription = self.create_subscription(Pose,'/turtle1/pose',self.pose_callback,10)

    # 这是回调函数，相当于单片机的“中断服务程序” (ISR)
    # 每次乌龟位置更新（大约 60Hz），这个函数就会被调用一次
    # 此处的msg是从/turtle1/pose上订阅的pose数据包含
    def pose_callback(self,msg):

        current_x= msg.x
        current_y= msg.y
        current_th = msg.theta
        p=0.5

        #创建Twist类实例cmd用于塞给self.publisher
        cmd = Twist()
        # 边界判断：turtlesim 的屏幕宽度大概是 11.08
        dis_x  = abs(min(current_x,abs(current_x - 11.08)))
        dis_y  = abs(min(current_y,abs(current_y - 11.08)))
        clo_dis = min(dis_x,dis_y)
        pi = math.pi

        if 1.2> dis_x>1.0 or 1.2> dis_y >1.0:
             if (abs(dis_x-dis_y))<=0.1:
                     if abs(current_th - 0)<=0.1 or abs(current_th - 1.57)<=0.1 or abs(current_th - 3.14)<=0.1 or abs(current_th - -1.57)<=0.1:
                         cmd.angular.z = 1.0
                         cmd.linear.x = 0.0 
                     elif not (abs(current_th - 0)<=0.1 or abs(current_th - 1.57)<=0.1 or abs(current_th - 3.14)<=0.1 or abs(current_th - -1.57)<=0.1):
                         cmd.angular.z = 1.0
                         cmd.linear.x = 0.0
                     else:
                         cmd.linear.x = 1.0
                         cmd.angular.z = 0.0 
             else:
                 if not (abs(current_th - 0)<=0.1 or abs(current_th - 1.57)<=0.1 or abs(current_th - 3.14)<=0.1 or abs(current_th - -1.57)<=0.1):
                     cmd.angular.z = 1.0
                     cmd.linear.x = 0.0
                 else:
                     cmd.linear.x = 1.0
                     cmd.angular.z = 0.0 
        else:
             cmd.linear.x = 1.0*clo_dis*p
             cmd.angular.z = 0.0 
        


        self.publisher_.publish(cmd)
                 
       
"""else:
            if abs(current_th - 0)<0.1 or abs(current_th - 1.57)<0.1 or abs(current_th - 3.14)<0.1 or abs(current_th - -1.57)<0.1 and 1.2< dis_x and 1.2< dis_y :
                if current_x>=current_y:
                    cmd.linear.x = 1.0*dis_x*p
                    cmd.angular.z = 0.0 
                else:
                    cmd.linear.x = 1.0*dis_y*p
                    cmd.angular.z = 0.0 
            elif abs(current_th - 0)<0.1 or abs(current_th - 1.57)<0.1 or abs(current_th - 3.14)<0.1 or abs(current_th - -1.57)<0.1 and (1.2< dis_x or 1.2< dis_y) :
                cmd.linear.x =1.0 
                cmd.angular.z = 0.0
            else:
                cmd.linear.x =0.0 
                cmd.angular.z = 2.0 
"""

   
        

def main(args=None):
    rclpy.init(args=args)
    node = smartturtle()
    rclpy.spin(node) # 死循环等待消息
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




