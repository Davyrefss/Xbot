"""
建图一键bringup--

usage:
    ros2 launch xbot_slam sim_mapping_launch.py

save map:
    ros2 run nav2_map_server map_saver_cli -f src/xbot_nav/maps/xbot_rooms
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim_share = get_package_share_directory('xbot_sim')
    slam_share = get_package_share_directory('xbot_slam')

    simworld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, 'launch', 'simworld_bringup_launch.py')),
        launch_arguments={'rviz_config': LaunchConfiguration('rviz_config')}.items(),
    )

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'slam_toolbox_launch.py')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config', default_value=os.path.join(slam_share, 'rviz', 'slam.rviz')),
        simworld,
        mapping,
    ])
