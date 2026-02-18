import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class smartdriver(Node):
    def __init__(self):
        super().__init__('smart_driver')
        #订阅传感器数据
        self.subscription = self.create_subscription(
            LaserScan,#数据类型
            '/scan',#数据来源
            self.scan_callback,#一有数据来就触发这个函数
            10#缓冲区：如果处理不过来，最多暂存10帧数据
            )
        
        self.publisher_ = self.create_publisher(
            Twist,#数据类型
            '/cmd_vel',#发布到哪里
            10#缓冲区
            )
        print('节点实例创建成功')
    def scan_callback(self ,msg):
     #转弯逻辑部分
        yuan = [msg.ranges]
        range = [l if l <=10.0 else 10.0 for l in msg.ranges]#清洗数据，msg.ranges是长360的数组测量范围是0.1到10详见urdf
        front = range[150:210]
        left = range[210:270]
        right = range[90:150]

        min_dis = min(min(front),min(left),min(right))
        print(min_dis)

        warn_dis = 1.0
        cmd = Twist()#创建新tw指令，实例化Twist

        if min_dis <= warn_dis:
            print('进入危险距离')
            if abs(min_dis - min(front)) <=  0.1:
                cmd.linear.x = 0.0
                print('前方有障碍物需要转弯')
                if min(left)>min(right):
                    print('左转')
                    cmd.angular.z = 0.5
                else:
                    print('右转')
                    cmd.angular.z = -0.5
            else:
                 cmd.linear.x = 0.5
                 cmd.angular.z = 0.0
        else:
            print('无障碍物直走')
            cmd.linear.x = 0.5
            cmd.angular.z = 0.0
            #执行
        self.publisher_.publish(cmd)



def main(args=None):
    rclpy.init(args=args)
    node = smartdriver()## 实例化
    #启动节点
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        blank_cmd = Twist()
        node.publisher_.publish(blank_cmd)#安全逻辑，当键盘插入时发布全0指令防止车乱爬
    finally:
        node.destroy_node()# 5. 资源回收
        rclpy.shutdown()
# 6. 关闭通信


if __name__ == '__main__':
    main()

    


        



