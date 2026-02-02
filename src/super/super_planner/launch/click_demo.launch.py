import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 这里虽然获取了路径，但在下面传参时不使用它
    pkg_share = get_package_share_directory('super_planner')
    rviz_config_path = os.path.join(get_package_share_directory('super_planner'), 'rviz', '1.rviz')
    # 2. 【修改点】不要构造绝对路径，只定义文件名
    # default_config_path = os.path.join(pkg_share, 'config', 'click_smooth_ros2.yaml') 
    config_filename = 'click_smooth_ros2.yaml' 

    # 3. 声明参数 (保持不变)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true') 
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # 4. 定义节点
    SUPER_NODE = Node(
        package='super_planner',
        executable='fsm_node',
        output='screen',
        parameters=[{
            # 【修改点】只传文件名，让 C++ 代码自己去拼前面的路径
            'config_name': config_filename, 
            'use_sim_time': use_sim_time
        }]
    )
    
    rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    # 使用 -d 替代 --display-config，这是最稳妥的写法
    arguments=['-d', rviz_config_path]
)
         
    
    world2map_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_map_broadcaster",
        output="screen",
        arguments=[
            "0", "0", "0",              # x y z
            "0", "0", "0",              # roll pitch yaw
            "world", "odom"
        ]
    )

    ld = LaunchDescription()
    ld.add_action(world2map_static_tf)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(SUPER_NODE)
    ld.add_action(rviz_node)

    return ld
