"""
    only bringup slam_toolbox

"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    slam_share = get_package_share_directory('xbot_slam')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')

    # 文件path
    params_file = os.path.join(slam_share, 'config', 'mapper_params_online_async.yaml')
    launch_file = os.path.join(slam_toolbox_share, 'launch', 'online_async_launch.py')

    # 启动配置的替换
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            'slam_params_file': params_file,
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items(), 
    )

    return LaunchDescription([
        slam_toolbox
    ])


