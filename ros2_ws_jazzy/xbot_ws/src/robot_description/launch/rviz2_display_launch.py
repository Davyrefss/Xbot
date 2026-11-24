"""
one key bringup rviz2
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    desc_share = get_package_share_directory('robot_description')
    use_sim_time = LaunchConfiguration('use_sim_time')

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(desc_share, 'rviz', 'robot.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='仿真中必须为 true, 否则 RViz 会报 message filter dropping message'),
        rviz2,
    ])
