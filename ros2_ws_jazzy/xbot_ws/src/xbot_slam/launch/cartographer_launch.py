"""
    only bringup cartographer

"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node 

def generate_launch_description():
    slam_share = get_package_share_directory('xbot_slam')
    cartographer_share = get_package_share_directory('cartographer_ros')

    # 文件path
    params_file = os.path.join(slam_share, 'config', 'cartographer_params_online.lua')

    # carto 节点
    carto_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-configuration_directory', os.path.join(slam_share, 'config'),
                   '-configuration_basename', 'cartographer_params_online.lua'],
    )
    # occupancy_grid_node 节点
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'resolution': 0.05
        }],
    )

    return LaunchDescription([
        carto_node,
        occupancy_grid_node
    ])



