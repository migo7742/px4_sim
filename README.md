1.先决条件
ubuntu22.04
ros2 humble
gazebo harmonic
micro-xrce-dds
2.文件夹说明
gz_ros_plugin:将gazebo中传感器数据转接成ros2话题
livox_ros_driver2：livox雷达消息类型定义
px4_msgs：px4消息类型定义
px4_ctrl：无人机控制节点，订阅规划器发布的命令，转发给飞控
small_point_lio：为规划器提供里程计和点云数据
visual_odom_bridge：将small_point_lio发布的odom转发给飞控实现外部视觉定位
ego_ros2:规划器，根据目标点规划出路径
key.py:命令发送脚本
3.启动slam
ros2 launch small_point_lio small_point_lio.launch.py
4.启动odom转接
ros2 run visual_odom_bridge visual_odom_bridge
5.启动规划器
ros2 launch ego_planner single_uav_gazebo.launch.py
6.启动rviz
ros2 launch ego_planner rviz.launch.py
7.启动控制节点
ros2 run px4_ctrl offboard_control
8.启动命令发送脚本
python3 key.py
输入t是起飞并悬停，输入o是进入offboard模式
