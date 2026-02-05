# ros2
My ROS 2 Learning Journey
本项目基于 **ROS 2 (jazzy)** 框架，实现了一个针对差速移动机器人（Differential Drive Robot）的**反应式导航系统**。
项目不依赖高精地图（SLAM），而是利用传感器反馈（Pose/Odom）进行实时的**闭环控制**。核心算法包含**比例速度控制（P-Control）**、**航向自动校正**以及**死角/死锁逃逸策略（Corner Recovery）**。<img width="550" height="599" alt="Screenshot from 2026-02-05 14-53-07" src="https://github.com/user-attachments/assets/4fa1d908-1fa2-4cb8-affc-7dcffc6ed746" />
