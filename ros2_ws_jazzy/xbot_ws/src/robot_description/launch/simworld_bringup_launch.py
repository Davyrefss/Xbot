"""Compatibility entry point for the Gazebo launch in xbot_sim."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    sim_share = Path(get_package_share_directory('xbot_sim'))
    sim_launch = sim_share / 'launch' / 'simworld_bringup_launch.py'

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(sim_launch))),
    ])
