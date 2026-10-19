#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 函数名固定
def generate_launch_description():
    # Get the path to the package's share directory
    share_dir = get_package_share_directory('ydlidar_ros2_driver')

    # 创建两个参数的引用，launch在运行时根据DeclareLaunchArgument的定义来解析参数
    params_file = LaunchConfiguration('params_file')
    port = LaunchConfiguration('port')

    # DeclareLaunchArgument用于声明launch文件中可以传入的参数，参数名为params_file和port
    '''
    ros2 launch ydlidar_ros2_driver x3_launch.py params_file:=./src/ydliadr_ros2_driver/params/ydlidar_x3.yaml port:=/dev/ttyUSB0
    '''
    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'params', 'ydlidar_x3.yaml'),
        description='YAML file with parameters for the YDLIDAR driver'
    )
    port_declare = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port to which the YDLIDAR is connected'
    )

    driver_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        parameters=[params_file, {'port': port}]
    )

    # 返回LaunchDescription对象，包含了声明的参数和节点
    return LaunchDescription([
        params_declare,
        port_declare,
        driver_node
    ])

    
