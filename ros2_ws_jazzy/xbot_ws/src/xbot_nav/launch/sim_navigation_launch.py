"""仿真导航: Gazebo + 仿真底盘 + Nav2 (AMCL + 已保存地图) + RViz

用法:
    # 1) 先按 sim_mapping_launch.py 建好图并保存到 src/xbot_nav/maps/xbot_rooms.(yaml|pgm)
    # 2) 再启动导航
    ros2 launch xbot_nav sim_navigation_launch.py

    # 无显示器:
    ros2 launch xbot_nav sim_navigation_launch.py headless:=true use_rviz:=false

    # 换地图 / 换世界:
    ros2 launch xbot_nav sim_navigation_launch.py \
        map:=/home/davyrefss/XBot/xbot_ws/src/xbot_nav/maps/xxx.yaml \
        world:=/home/davyrefss/XBot/xbot_ws/src/robot_description/worlds/xbot_rooms.world

启动后:
    RViz 里用 "2D Pose Estimate" 校正初始位姿(默认已按 (0,0) 初始化),
    再用 "Nav2 Goal" / "2D Goal Pose" 点一个目标点即可。
    命令行也可以直接发动作:
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
      "{pose: {header: {frame_id: map}, pose: {position: {x: 3.0, y: 0.0}, orientation: {w: 1.0}}}}"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory('xbot_nav')
    desc_share = get_package_share_directory('robot_description')
    nav2_share = get_package_share_directory('nav2_bringup')

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    rviz_config = LaunchConfiguration('rviz_config')

    # 1. Gazebo 世界 + 机器人 + /scan /odom /tf /clock 桥接
    #    headless=true 时给 gz sim 加 -s (纯服务端无 GUI)
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_share, 'launch', 'simworld_bringup_launch.py')),
        launch_arguments={
            'world': world,
            'use_sim_time': use_sim_time,
            'use_rviz': use_rviz,
            'extra_gz_args': PythonExpression(
                ["'-s ' if '", headless, "' == 'true' else ''"]),
        }.items(),
    )

    # 2. Nav2: map_server + amcl + 规划/控制/行为树 (slam=false 表示用已有地图)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'namespace': '',
            'use_namespace': 'false',
            'slam': 'False',
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': 'false',
        }.items(),
    )

    # 3. RViz: 直接用 nav2 自带的默认视图(含 map/costmap/路径/导航按钮)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(desc_share, 'worlds', 'xbot_rooms.world'),
            description='Gazebo 世界文件'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='true=不开 Gazebo GUI'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='true=打开 RViz'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='仿真中必须为 true'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(nav_share, 'maps', 'xbot_rooms.yaml'),
            description='地图 yaml 路径(先用建图流程生成)'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(nav_share, 'config', 'nav2_params.yaml'),
            description='Nav2 参数文件'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='自动完成 lifecycle configure/activate'),
        DeclareLaunchArgument('use_composition', default_value='True',
                              description='true=所有 nav2 节点跑在一个容器进程里, 省 CPU'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(nav2_share, 'rviz', 'nav2_default_view.rviz'),
            description='RViz 配置文件'),
        sim,
        nav2,
        rviz,
    ])
