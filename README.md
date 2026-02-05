# 🐢 ROS 2 Autonomous Wall-Following & Corner-Escape System
> **基于 ROS 2 的移动机器人自主贴墙与角落逃逸算法实现**
> <img width="550" height="599" alt="Screenshot from 2026-02-05 14-53-07" src="https://github.com/user-attachments/assets/86eeecb0-4521-468c-9387-d9c623743416" />


## 📖 项目背景 (Background)
在移动机器人导航中，**“贴墙跟随 (Wall Following)”** 是最基础也是最具挑战性的行为之一。尤其是在死角（Corner）区域，简单的规则控制往往会导致机器人陷入局部极值（Local Minima）或原地震荡。

本项目基于 **ROS 2** 框架，设计了一套包含**P控制（比例控制）**与**状态机（State Machine）**的闭环控制算法，成功解决了机器人在受限空间内的自主导航与死锁逃逸问题。

## 🛠 技术栈 (Tech Stack)
*   **Framework**: ROS 2 (Jazzy/Humble)
*   **Language**: Python 3 (`rclpy`)
*   **Algorithm**: Closed-loop Kinematic Control, State Machine
*   **Simulation**: Turtlesim

## 🚀 核心算法逻辑 (Core Algorithms)

本项目并未采用黑盒的强化学习，而是设计了一套**可解释性强、鲁棒性高**的规则系统。核心逻辑如下：

### 1. 虚拟感知层 (Virtual Perception)
通过订阅 `/turtle1/pose` 话题，实时解算机器人相对于环境边界（四面墙壁）的**欧氏距离**。
```python
# 实时计算最近障碍物距离 (Lidar-like Logic)
dis_x = abs(min(current_x, abs(current_x - 11.09)))
dis_y = abs(min(current_y, abs(current_y - 11.09)))![Uploading Screenshot from 2026-02-05 14-53-07.png…]()

clo_dis = min(dis_x, dis_y)
```

### 2. 速度规划 (Velocity Profiling)
为了防止机器人高速撞墙，引入了 **P-Controller (比例控制器)**。
*   **逻辑**：线速度与最近障碍物距离成正比 (`v = Kp * distance`)。
*   **效果**：当机器人远离墙壁时全速行驶，接近墙壁时自动平滑减速，实现“软着陆”。

### 3. 角落死锁逃逸机制 (Corner Deadlock Escape) —— **项目亮点** ✨
在矩形角落（同时靠近 X 轴和 Y 轴墙壁）是传统算法最容易失效的区域。本项目设计了特殊的逃逸逻辑：

*   **状态检测**：当 `abs(dis_x - dis_y) <= 0.1` 且处于边界区域时，判定为进入“角落死区”。
*   **姿态解算**：检查当前航向角是否对齐（0, 90, 180, 270度）。
*   **逃逸策略**：
    *   若姿态不对齐：优先原地旋转 (`angular.z = 1.0`)。
    *   **若姿态已对齐但仍被困（死锁）**：强制输出混合速度矢量 (`linear.x = 0.5`, `angular.z = 0.5`)。这会产生一个切向运动，强制机器人破坏当前的稳定平衡态，从而“滑”出死角。

## 📝 代码实现细节 (Code Logic)

整个控制逻辑被封装在 `pose_callback` 中，形成一个实时闭环：

1.  **安全区 (Safe Zone)**:
    *   判定：距离墙壁 > 2.0m。
    *   动作：执行 P 控制直行，快速接近边界。

2.  **单边墙壁修正 (Wall Alignment)**:
    *   判定：只靠近一面墙。
    *   动作：检测角度误差。如果未平行于墙面，执行原地转向；如果已平行，沿墙巡航。

3.  **角落处理 (Corner Handling)**:
    *   判定：同时靠近两面墙 (差值 < 0.1)。
    *   动作：**触发自动修正逻辑**。无论当前是否对齐，强制施加角速度和线速度，完成 90 度转弯机动。

## 📸 结果分析 (Result Analysis)
从运行截图可以看出：
1.  **轨迹平滑**：直线路段没有明显的震荡，证明 P 控制参数 (`p=0.5`) 选择得当。
2.  **转弯精准**：在四个顶点处，机器人均完成了标准的 90 度转向，没有发生碰撞或卡死。
3.  **内旋收敛**：随着路径演化，机器人自动向中心收敛，证明了算法在连续空间内的稳定性。

## 🏃 如何运行 (Usage)

1.  启动仿真环境：
    ```bash
    ros2 run turtlesim turtlesim_node
    ```
2.  启动控制节点：
    ```bash
    python3 smart_turtle.py
    ```

---
