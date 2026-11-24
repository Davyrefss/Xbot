import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg = 'robot_description'
    pkg_share = get_package_share_directory(pkg)

    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    rviz_path = os.path.join(pkg_share, 'rviz', 'robot.rviz')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()
    '''
        robot_state_publisher节点用于发布机器人状态信息
        joint_state_publisher_gui节点用于提供一个GUI界面来控制关节状态
        rviz2节点用于可视化机器人模型和状态;
        发布一个robot_description话题,
        joint_state_publisher_gui节点会订阅这个话题来获取机器人模型信息,
        手动控制关节状态将信息发布到joint_states话题,
        同时robot_state_publisher节点会订阅joint_states话题来获取关节状态信息,
        并将机器人模型和状态信息发布到tf话题中
        riviz2节点会订阅robot_description话题来获取机器人模型信息,并显示在rviz中
    '''
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_path],
        ),
    ])
