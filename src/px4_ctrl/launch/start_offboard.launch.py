import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # ---------------------------------------------
    # 这里必须修改为你实际的包名
    # 你的 package.xml 里写的是 px4_ctrl
    # ---------------------------------------------
    package_name = 'px4_ctrl' 
    
    # 获取 yaml 文件路径
    config_file_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'geometric_controller.yaml'
    )

    return LaunchDescription([
        Node(
            package=package_name,
            executable='offboard_control_node', 
            name='offboard_geometric_controller',
            output='screen',
            parameters=[config_file_path],
            # 如果你的规划器话题不是 /planning/pos_cmd，可以在这里修改
            # remappings=[
            #    ('/planning/pos_cmd', '/your_planner_topic') 
            # ]
        )
    ])
