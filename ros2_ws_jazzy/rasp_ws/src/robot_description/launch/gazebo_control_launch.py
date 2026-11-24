import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = 'robot_description'
    pkg_share = get_package_share_directory(pkg)

    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    world_path = os.path.join(pkg_share, 'worlds', 'robot.world')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'generated_robot',
                   '-topic', 'robot_description',
                   '-z', '0.5'],
        output='screen',
    )

    # 桥接 ROS /cmd_vel 到 gz /model/generated_robot/cmd_vel
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/generated_robot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        remappings=[('/model/generated_robot/cmd_vel', '/cmd_vel')],
        output='screen',
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn,
        bridge,
    ])
