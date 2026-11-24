"""Gazebo (gz-sim 8 / Harmonic) 仿真底盘 bringup

只负责“仿真世界 + 机器人 + ROS/Gazebo 桥”，不启动建图或导航:

    Gazebo 世界  ──/scan──┐
                 ──/odom──┤ ros_gz_bridge ──> ROS 2
                 ──/tf─────┤   (clock/scan/odom/tf/joint_states 进, cmd_vel 出)
                 ──/clock──┘
    robot_state_publisher: 发布 base_footprint->base_link/wheel/laser 的 TF
    ros_gz_sim create:     把 URDF 从 /robot_description 话题生成到 Gazebo 里

用法:
    ros2 launch robot_description sim_bringup_launch.py
    ros2 launch robot_description sim_bringup_launch.py headless:=true
    ros2 launch robot_description sim_bringup_launch.py world:=/path/to/xxx.world

注意:
    1. 仿真时所有节点都要 use_sim_time:=true, 否则 TF/传感器时间戳和 /clock 对不上,
       RViz 会报 "message filter dropping message ... earlier than transform"
    2. Gazebo 里的 URDF 网格用的是 model://robot_description/meshes/...,
       所以必须把包的 share 父目录加进 GZ_SIM_RESOURCE_PATH, 否则模型没有外观/碰撞体,
       机器人会“掉进地板”并且无法运动
    3. /cmd_vel 的桥接方向是 ROS->Gazebo (] 表示 ROS 到 GZ), 其余是 GZ->ROS ([)
    4. gpu_lidar 必须有 OpenGL: 本机建议先 export LIBGL_ALWAYS_SOFTWARE=1 (VMware/无独显),
       headless 模式下也会借用当前 DISPLAY 做 GLX 软件渲染; 完全没有显示器时
       加 extra_gz_args:="--headless-rendering" 走 EGL。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    desc_share = get_package_share_directory('robot_description')
    gz_share = get_package_share_directory('ros_gz_sim')

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    extra_gz_args = LaunchConfiguration('extra_gz_args')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw = LaunchConfiguration('yaw')

    urdf_path = os.path.join(desc_share, 'urdf', 'robot.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # 用 gz sim 官方 launch，只替换 gz_args。
    # GUI 模式: 服务器 + 客户端一起起。
    # headless: 只起服务器。注意雷达(gpu_lidar)仍然需要 OpenGL 上下文，
    #   没有 GUI 时 Gazebo 会借用当前 DISPLAY 的 GLX 做软件渲染，所以
    #   headless 也需要 DISPLAY(本机桌面/SSH -X 都可以)。
    #   真·无显示器的服务器再加 extra_gz_args:="--headless-rendering" 走 EGL。
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r ', extra_gz_args, ' ', world]}.items(),
        condition=UnlessCondition(headless),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r -s ', extra_gz_args, ' ', world]
        }.items(),
        condition=IfCondition(headless),
    )

    # URDF 里的 mesh 是 package://robot_description/meshes/xxx.stl，
    # sdformat 会把它翻译成 model://robot_description/meshes/xxx.stl，
    # 需要让 Gazebo 在 share 目录下能找到 robot_description 这个“模型目录”。
    set_gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.dirname(desc_share),
    )

    # 说明: 本项目的雷达是 gpu_lidar，需要 GPU/软件 OpenGL。在 VMware 里如果
    # 报 "eglInitialize failed / OpenGL 3.3 is not supported"，先把
    #   export LIBGL_ALWAYS_SOFTWARE=1
    # 再启动 (GUI 与 headless 都适用)。

    # gz transport 默认走组播(239.255.0.1)。虚拟机/多网卡/CI 里偶尔会报
    # "Exception sending a multicast message: Network is unreachable",
    # 导致 /scan /odom 突然全断。单机仿真强制走回环最稳:
    gz_env_actions = []
    if 'GZ_IP' not in os.environ:
        gz_env_actions.append(SetEnvironmentVariable('GZ_IP', '127.0.0.1'))

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw,
        ],
        output='screen',
    )

    # 话题方向: [ = Gazebo -> ROS，] = ROS -> Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # 轮速里程计(会打滑), 单独给出来方便和真值对比; 不参与 TF
            '/odom_wheel@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
    )

    return LaunchDescription(gz_env_actions + [
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(desc_share, 'worlds', 'xbot_rooms.world'),
            description='Gazebo 世界文件'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='true=只起 Gazebo 服务器(无 GUI)，适用于 SSH/CI'),
        DeclareLaunchArgument(
            'extra_gz_args', default_value='',
            description='追加给 gz sim 的参数, 例如 "--headless-rendering"'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='仿真中必须为 true'),
        DeclareLaunchArgument('robot_name', default_value='xbot',
                              description='Gazebo 里的模型名'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('z_pose', default_value='0.05',
                              description='base_footprint 的初始高度, 抬起一点让轮子落地'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        set_gz_resource_path,
        gazebo_gui,
        gazebo_headless,
        robot_state_publisher,
        spawn_robot,
        bridge,
    ])
