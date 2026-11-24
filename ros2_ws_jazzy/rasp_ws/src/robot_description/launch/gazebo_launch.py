# 解决在vmware中由于虚拟显卡的渲染导致gazebo显示异常
# export LIBGL_ALWAYS_SOFTWARE=1; export QT_QPA_PLATFORM=xcb
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg = "robot_description"
    # 拼接urdf和world的路径
    urdf_path = os.path.join(get_package_share_directory(pkg), "urdf", "robot.urdf")
    world_path = os.path.join(get_package_share_directory(pkg), "worlds", "robot.world")

    with open(urdf_path, "r") as f:
        robot_description = f.read()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r {world_path}"}.items(),
    )

    return LaunchDescription([
        gazebo,
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=["-name", "generated_robot", "-file", urdf_path, '-z', '1.0'],
            output="screen",
        )
    ])