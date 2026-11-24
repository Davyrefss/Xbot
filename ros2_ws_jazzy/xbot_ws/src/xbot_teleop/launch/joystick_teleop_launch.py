"""
    /joy -> teleop_twist_joy -> /cmd_vel -> Gazebo diff-drive
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    joy_dev = LaunchConfiguration('joy_dev')
    joy_config = LaunchConfiguration('joy_config')
    scale_linear = LaunchConfiguration('scale_linear')
    scale_angular = LaunchConfiguration('scale_angular')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device': joy_dev,
            'autorepeat_rate': 20.0,  # 重新发布速率
        }],
        output='screen',
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[{
            'joy_config': joy_config,
            'require_enable_button': False,
            'axis_linear.x': 1,
            'axis_angular.yaw': 0,
            'scale_linear': scale_linear,
            'scale_angular': scale_angular,
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        DeclareLaunchArgument('joy_config', default_value='xbox'),
        DeclareLaunchArgument('scale_linear', default_value='0.5'),
        DeclareLaunchArgument('scale_angular', default_value='1.5'),
        # launch
        joy_node,
        teleop_node
    ])



