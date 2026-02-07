本项目是一个基于 ROS2 Humble 和 PX4 的无人机仿真与导航系统。集成了micro-xrce-dds、Small-Point-LIO 里程计以及 EGO-Planner 路径规划，实现了从感知到规划再到控制的全流程闭环。
🛠 1. 环境依赖 (Prerequisites)

在运行本项目前，请确保您的开发环境已配置以下核心组件：
组件	版本	说明
操作系统	Ubuntu 22.04 LTS	推荐系统版本
ROS2	Humble Hawksbill	机器人操作系统
仿真引擎	Gazebo Harmonic	配合 PX4 使用的仿真环境
通信中间件	Micro-XRCE-DDS	用于 PX4 与 ROS2 的高速数据交换
📂 2. 核心模块说明 (Modules)

项目工作空间包含以下关键功能包：

    gz_ros_plugin: 桥接层，负责将 Gazebo 中的传感器原始数据转换为 ROS2 标准话题。

    livox_ros_driver2: Livox 激光雷达的消息类型定义及驱动支持。

    px4_msgs: 包含 PX4 固件定义的标准 uORB 消息对应的 ROS2 消息格式。

    small_point_lio: 轻量化激光惯性里程计，为规划器提供高精度的里程计数据和点云。

    visual_odom_bridge: 转换插件，将 small_point_lio 的位姿转化为 PX4 可识别的外部定位数据。

    ego_ros2: 核心规划器，基于 EGO-Planner 算法根据目标点生成平滑的避障路径。

    px4_ctrl: 控制节点，订阅规划器命令并将其封装为 Offboard 指令发送给飞控。

    key.py: 交互脚本，通过键盘终端快速发送起飞、模式切换等控制指令。

🚀 3. 运行指南 (Execution Guide)

请按照以下顺序在不同的终端窗口中启动节点：
第一步：启动 SLAM 与里程计

解析雷达数据并建立实时里程计。
Bash

ros2 launch small_point_lio small_point_lio.launch.py

第二步：启动定位转接 (Odom Bridge)

将视觉/激光里程计反馈给 PX4 飞控，实现外部定位。
Bash

ros2 run visual_odom_bridge visual_odom_bridge

第三步：启动规划器与可视化

启动 EGO-Planner 核心算法及 Rviz2 监控界面。
Bash

# 启动规划节点
ros2 launch ego_planner single_uav_gazebo.launch.py

# 启动 Rviz 可视化
ros2 launch ego_planner rviz.launch.py

第四步：启动控制与交互

启动控制逻辑并使用键盘脚本接管无人机。
Bash

# 1. 启动控制节点
ros2 run px4_ctrl offboard_control

# 2. 运行键盘控制脚本
python3 key.py

🎮 4. 指令交互 (Command Table)

在 key.py 运行的终端中，可以使用以下快捷键：
按键	功能描述
t	Takeoff: 自动解锁、起飞并进入悬停状态
o	Offboard: 切换至 Offboard 模式，开始执行规划器路径
