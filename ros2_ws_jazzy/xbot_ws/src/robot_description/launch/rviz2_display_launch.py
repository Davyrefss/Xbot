"""Display the XBot model in RViz."""

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
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the clock published on /clock'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(desc_share, 'rviz', 'robot.rviz')),
        rviz2,
    ])
