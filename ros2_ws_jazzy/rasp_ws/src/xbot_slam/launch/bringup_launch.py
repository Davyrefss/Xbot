"""
    one key bring up hardware and mapping
"""

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    lidar_share = get_package_share_directory('ydlidar_ros2_driver')
    slam_share = get_package_share_directory('xbot_slam')
    desc_share = get_package_share_directory('robot_description')

    # 声明+引用
    slave_port_declare = DeclareLaunchArgument(
        'slave_port',
        default_value='/dev/ttyUSB0',
        description='Serial port to which the slave device is connected'
    )
    slave_port = LaunchConfiguration('slave_port')
    lidar_port_declare = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB1',
        description='Serial port to which the lidar is connected'
    )
    lidar_port = LaunchConfiguration('lidar_port')


    # 读取机器人描述
    with open(os.path.join(desc_share, 'urdf', 'robot.urdf'), 'r') as infp:
        robot_desc = infp.read()

    # RSP节点
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False
        }]
    )

    # 下位控制节点--串口
    slave_ctrl = Node(
        package='xbot_serial',
        executable='serial_node',
        name='xbot_serial_node',
        output='screen',
        parameters=[{
            'port': slave_port,
        }]
    )

    # mapping launch
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_share, 'launch', 'mapping_launch.py')),
    )

    # lidar launch
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(lidar_share, 'launch', 'x3_launch.py')),
        launch_arguments={
            'port': lidar_port
        }.items(),
    )

    return LaunchDescription([
        # 声明参数
        slave_port_declare,
        lidar_port_declare,
        # 启动节点
        slave_ctrl,
        robot_state_publisher,
        mapping_launch,
        lidar_launch
    ])



