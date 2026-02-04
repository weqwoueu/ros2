import rclpy
from rclpy.node import Node
# 导入乌龟能听懂的消息格式 (Twist: 线速度+角速度)
from geometry_msgs.msg import Twist

#定义节点Circledrawer

class Circledrawer(Node):
    def __init__(self):
        super().__init__('circle_drawer_node')#节点名称'circle_drawer_node'
        # 创建发布者 (Publisher)
        # 含义：我要对着 '/turtle1/cmd_vel' 这个大喇叭喊话
        # 消息类型是 Twist，队列长度 10
        self.publisher_ = self.create_publisher(Twist,'/turtle1/cmd_vel',10,)
        
        #设置定时器（Timer）
        #定时周期为0.5秒
        time_period = 0.5
        self.timer = self.create_timer(time_period , self.timer_callback)

        print("节点启动！准备画圆...")

        #定时器周期调用timer_callback
    def timer_callback(self):
        #产生Twist类实例msg
        msg = Twist()

        msg.linear.x = 10.0 #(m/s)默认 X 轴指向车头正前方
        msg.angular.z = 5.0 #(rad/s)z指向屏幕外
             
        #发布信息
        self.publisher_.publish(msg)
        print(f"已发布信息，线速度{msg.linear.x}   角速度{msg.angular.z}")
def main(args=None):
    #初始化ros2通信库只有执行了这句，你的程序才有资格去连接 ROS 2 的通信网络。如果不写，后面的节点根本创建不出来。
    rclpy.init(args=args)

    #创建节点实例node
    node = Circledrawer()

    #让节点“旋转”起来,进入死循环值班，0.5s 到了 -> 跑去执行 timer_callback -> 发指令 -> 回来继续等，node.timer定时器只是个闹钟不负责调用，如果闹钟响了：它就负责去调用 timer_callback 函数。如果没事：它就挂起等待，不占用过多 CPU
    rclpy.spin(node)

    #按了Ctrl+C 强行终止程序，spin 循环会被打破，代码才会走到这里。
    #注销节点
    #destroy_node：把这个节点注销掉，告诉 ROS 网络“我下线了，别再给我发消息了”
    node.destroy_node()

    #shutdown：关闭 ROS 2 库，释放内存，拉下总闸。
    rclpy.shutdown()

# 安全锁
#如果我直接运行这个脚本 (python3 draw_circle.py)，那就执行 main()。
#如果我是被别的脚本导入的 (import draw_circle)，那就不要执行 main()。
#这防止了你只想复用这个类时，它却自动启动了程序。
if __name__ == '__main__':
    main()















