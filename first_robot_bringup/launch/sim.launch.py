import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 确定 URDF 文件的绝对路径
    # 注意：这里我们暂时硬编码路径，因为你还没有把 urdf 打包安装
    # 如果你的用户名不是 liu，请修改下面的路径！
    urdf_file = os.path.expanduser('~/learn/ros2/robot_description/urdf/first_robot.urdf')

    # 读取 URDF 内容
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 2. 配置 Gazebo 仿真环境
    # 引用官方的 empty.sdf 启动文件
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 3. 机器人状态发布节点 (Robot State Publisher)
    # 它的作用是把 URDF 发给 Gazebo 和 Rviz
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 4. 在 Gazebo 中生成机器人 (Spawn)
    # 它会从 topic 中读取机器人描述，然后生出来
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-z', '0.5'],
        output='screen'
    )

    # 5. 通信桥接 (Bridge)
    # 连接 ROS 的 /cmd_vel 和 Gazebo 的 /cmd_vel
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',#速度指令ros--->gazebo
                   
                   #激光雷达数据gazebo--->ros
                   #格式：/scan @ros类型 @ gazebo类型
                   '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',

                   #tf坐标变换（gazebo——>ros）
                   '/model/first_robot/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'
                   ],
        output='screen',
        remappings=[('/model/first_robot/tf',  '/tf')] 
        )

    # 6. 键盘控制 (Teleop)
    # 为了方便，让他单独弹出一个新终端窗口运行
    # prefix="gnome-terminal --" 意思是：打开一个新的黑色终端窗口来跑这个命令
    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        prefix='gnome-terminal --', 
        output='screen'
    )

    tf_patch = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar', 'first_robot/base_link/lidar'],
        output='screen'
    )

    # 返回描述，告诉 ROS 2 要启动哪些东西
    return LaunchDescription([
        gazebo,
        rsp,
        spawn,
        bridge,
        #teleop,
        tf_patch,
    ])