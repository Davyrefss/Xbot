"""Launch the Gazebo world, XBot model, and ROS bridge."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import IfElseSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():
    sim_share = Path(get_package_share_directory('xbot_sim'))
    desc_share = Path(get_package_share_directory('robot_description'))
    gz_share = Path(get_package_share_directory('ros_gz_sim'))
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')

    arguments = [
        DeclareLaunchArgument(
            'world', default_value=str(sim_share / 'worlds' / 'xbot_rooms.world'),
            description='Gazebo world file'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run Gazebo server with headless sensor rendering'),
        DeclareLaunchArgument(
            'extra_gz_args', default_value='',
            description='Additional arguments for gz sim'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('robot_name', default_value='xbot'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('z_pose', default_value='0.08'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'software_gl', default_value=IfElseSubstitution(headless, 'false', 'true'),
            description='Force Mesa software rendering (defaults to false with headless EGL)'),
        DeclareLaunchArgument(
            'qt_xcb', default_value='true',
            description='Force Qt to use the X11/xcb backend'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config', default_value=str(desc_share / 'rviz' / 'robot.rviz')),
    ]

    environment = []
    if 'GZ_IP' not in os.environ:
        environment.append(SetEnvironmentVariable('GZ_IP', '127.0.0.1'))

    environment.extend([
        # Gazebo resolves model://robot_description relative to this share directory.
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(desc_share.parent)),
        SetEnvironmentVariable(
            'LIBGL_ALWAYS_SOFTWARE', '1',
            condition=IfCondition(LaunchConfiguration('software_gl'))),
        SetEnvironmentVariable(
            'QT_QPA_PLATFORM', 'xcb',
            condition=IfCondition(LaunchConfiguration('qt_xcb'))),
    ])

    headless_args = IfElseSubstitution(headless, '-s --headless-rendering ', '')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gz_share / 'launch' / 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [
                '-r ', headless_args, LaunchConfiguration('extra_gz_args'),
                ' ', LaunchConfiguration('world'),
            ],
        }.items(),
    )

    model = xacro.process_file(str(sim_share / 'urdf' / 'xbot_sim.urdf.xacro')).toxml()
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(model, value_type=str),
            'use_sim_time': use_sim_time,
        }],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', LaunchConfiguration('robot_name'),
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x_pose'),
            '-y', LaunchConfiguration('y_pose'),
            '-z', LaunchConfiguration('z_pose'),
            '-Y', LaunchConfiguration('yaw'),
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': str(sim_share / 'config' / 'bridge.yaml')}],
        output='screen',
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(desc_share / 'launch' / 'rviz2_display_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz_config': LaunchConfiguration('rviz_config'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription(arguments + environment + [
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        rviz,
    ])
