import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    slam_share = get_package_share_directory('xbot_slam')
    desc_share = get_package_share_directory('robot_description')

    with open(os.path.join(desc_share, 'urdf', 'robot.urdf'), 'r') as f:
        robot_description = f.read()

    params_file = os.path.join(slam_share, 'config', 'mapper_params_online_async.yaml')
    rviz_config = os.path.join(slam_share, 'rviz', 'slam.rviz')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[params_file],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        robot_state_publisher,
        slam_toolbox,
        rviz,
    ])