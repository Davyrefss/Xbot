import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg = "robot_description"
    gaze_cntl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg), "launch", "gazebo_control_launch.py")
        )
    )

    return LaunchDescription([
        gaze_cntl,
        Node(
            package='xbot_control',
            executable='teleop_keyboard',
            output='screen'
        )
    ])