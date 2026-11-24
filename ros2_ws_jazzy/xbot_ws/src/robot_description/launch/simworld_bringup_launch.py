"""
simworld一键bringup--

只负责“仿真世界 + 机器人 + ROS/Gazebo 桥”，不启动建图或导航:

    Gazebo 世界   ──/scan──┐
                 ──/odom──┤ 
                ───/tf────┤ 
                ───/clock─┤ --> ros_gz_bridge ──> ROS 2
                ─/cmd_vel─┤
             ─/odom_wheel─┤  
           ─/joint_states─┘
    robot_state_publisher: 发布 base_footprint->base_link/wheel/laser 的 TF
    ros_gz_sim create:     把 URDF 从 /robot_description 话题生成到 Gazebo 里

usage:
    ros2 launch robot_description simworld_bringup_launch.py

    # 无头运行(不开 Gazebo GUI 和 RViz, 适合 CI/远程验证):
    ros2 launch robot_description simworld_bringup_launch.py \
        use_rviz:=false extra_gz_args:='-s '

注意:
    1. 仿真时所有节点都要 use_sim_time:=true, 否则 TF/传感器时间戳和 /clock 对不上,
       RViz 会报 "message filter dropping message ... earlier than transform"
    2. Gazebo 里的 URDF 网格用的是 model://robot_description/meshes/...,
       所以必须把包的 share 父目录加进 GZ_SIM_RESOURCE_PATH, 否则模型没有外观/碰撞体,
       机器人会“掉进地板”并且无法运动
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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    desc_share = get_package_share_directory('robot_description')
    gz_share = get_package_share_directory('ros_gz_sim')

    # Gazebo 仿真参数 + 引用
    world_declare = DeclareLaunchArgument(
        'world',
        default_value = os.path.join(desc_share, 'worlds', 'xbot_rooms.world'),
        description = 'sim world file path'
    )
    world = LaunchConfiguration('world')
    extra_gz_args_declare = DeclareLaunchArgument(
        'extra_gz_args',
        default_value = '',
        description = '追加给 gz sim 的参数, 例如 "--headless-rendering"'
    )
    extra_gz_args = LaunchConfiguration('extra_gz_args')
    use_sim_time_declare = DeclareLaunchArgument(
        'use_sim_time',
        default_value = 'true',
        description = 'must be true in simulation'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name_declare = DeclareLaunchArgument(
        'robot_name',
        default_value = 'xbot',
        description = 'robot name in Gazebo'
    )
    robot_name = LaunchConfiguration('robot_name')
    x_pose_declare = DeclareLaunchArgument(
        'x_pose',
        default_value = '0.0',
        description = 'x pose of robot'
    )
    x_pose = LaunchConfiguration('x_pose')
    y_pose_declare = DeclareLaunchArgument(
        'y_pose',
        default_value = '0.0',
        description = 'y pose of robot'
    )
    y_pose = LaunchConfiguration('y_pose')
    z_pose_declare = DeclareLaunchArgument(
        'z_pose',
        default_value = '0.08',     
        description = 'base_footprint initial height, lift a bit to let wheels touch the ground'
    )
    z_pose = LaunchConfiguration('z_pose')
    yaw_declare = DeclareLaunchArgument(
        'yaw',
        default_value = '0.0',
        description = 'yaw angle of robot'
    )
    yaw = LaunchConfiguration('yaw')

    # force rendering
    sf_gl_declare = DeclareLaunchArgument(
        'software_gl',
        default_value='true',
        description='Force Mesa software rendering (LIBGL_ALWAYS_SOFTWARE=1)',
    )     
    software_gl = LaunchConfiguration('software_gl')
    qt_xcb_declare = DeclareLaunchArgument(
        'qt_xcb',
        default_value='true',
        description='Force Qt to use the X11/xcb backend (QT_QPA_PLATFORM=xcb)',
    )
    qt_xcb = LaunchConfiguration('qt_xcb')
    use_rviz_declare = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='true=同时启动 RViz 显示机器人模型 (无头运行设 false)',
    )
    use_rviz = LaunchConfiguration('use_rviz')

    with open(os.path.join(desc_share, 'urdf', 'robot.urdf'), 'r') as f:
        robot_description = f.read()

    # gz transport 默认走组播(239.255.0.1)。虚拟机/多网卡/CI 里偶尔会报
    # "Exception sending a multicast message: Network is unreachable",
    gz_env_actions = []
    if 'GZ_IP' not in os.environ:
        gz_env_actions.append(SetEnvironmentVariable('GZ_IP', '127.0.0.1'))

    # URDF 里的 mesh 是 package://robot_description/meshes/xxx.stl，
    # sdformat 会把它翻译成 model://robot_description/meshes/xxx.stl，
    # 需要让 Gazebo 在 share 目录下能找到 robot_description 这个“模型目录”。
    set_gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.dirname(desc_share),
    )    

    # -->解决在vmware中由于虚拟显卡的渲染导致gazebo显示异常
    set_software_gl = SetEnvironmentVariable(
        'LIBGL_ALWAYS_SOFTWARE', '1',
        condition=IfCondition(software_gl),
    )
    set_qt_xcb = SetEnvironmentVariable(
        'QT_QPA_PLATFORM', 'xcb',
        condition=IfCondition(qt_xcb),
    )

    # 用 gz_sim官方launch启动GUI，只替换 gz_args
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r ', extra_gz_args, ' ', world]}.items(),
    )

    # node
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

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        # @ 前面是话题名；
        # @ 后面是“ROS 类型 + 方向符 + Gazebo 类型”；
        # [ 表示 Gazebo → ROS
        # ] 表示 ROS → Gazebo
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

    rviz2_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_share, 'launch', 'rviz2_display_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(gz_env_actions + [
        # Gazebo 仿真参数
        world_declare,
        extra_gz_args_declare,
        use_sim_time_declare,
        robot_name_declare,
        x_pose_declare,
        y_pose_declare,
        z_pose_declare,
        yaw_declare,
        sf_gl_declare,
        qt_xcb_declare,
        use_rviz_declare,
        # launch actions
        set_gz_resource_path,
        set_software_gl,
        set_qt_xcb,
        gazebo_gui,
        robot_state_publisher,
        spawn_robot,
        bridge,
        rviz2_gui,
    ])
