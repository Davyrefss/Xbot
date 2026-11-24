include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,                -- 把全局地图构建器配置挂到 options 上
  trajectory_builder = TRAJECTORY_BUILDER,  -- 把轨迹构建器配置挂到 options 上
  sensor_bridge = SENSOR_BRIDGE,            -- 把传感器桥配置挂到 options 上
  -- 下面是一些常用的参数配置
  map_frame = "map",                        -- 地图坐标系的名称
  tracking_frame = "base_footprint",        -- 机器人底盘坐标系的名称
  published_frame = "odom",                 -- 发布的坐标系的名称
  odom_frame = "odom",                      -- 里程计坐标系的名称    
  provide_odom_frame = false,               -- 是否提供里程计坐标系    
  publish_frame_projected_to_2d = true,     -- 是否将发布的坐标系投影到2D平面上
  use_pose_extrapolator = true,             -- 是否使用姿态外推器
  -- 输入传感器数据
  use_odometry = true,                      -- 是否使用里程计数据
  use_nav_sat = false,                      -- 是否使用导航卫星数据
  use_landmarks = false,                    -- 是否使用地标数据
  num_laser_scans = 1,                      -- 激光扫描数量
  num_multi_echo_laser_scans = 0,           -- 多回波激光扫描数量
  num_subdivisions_per_laser_scan = 1,      -- 每个激光扫描的子划分数量
  num_point_clouds = 0,                     -- 点云数量  
  -- 超时和发布频率
  lookup_transform_timeout_sec = 0.2,       -- 查找变换的超时时间
  submap_publish_period_sec = 0.3,          -- 子地图发布周期
  pose_publish_period_sec = 5e-3,           -- 姿态发布周期
  trajectory_publish_period_sec = 30e-3,    -- 轨迹发布周期
  -- 采样比率
  rangefinder_sampling_ratio = 1.,          -- 测距仪采样比率
  odometry_sampling_ratio = 1.,             -- 里程计采样比率   
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true    -- 使用2D轨迹构建器

TRAJECTORY_BUILDER_2D.min_range = 0.12                              -- 小于 0.12m 的激光点直接丢掉
TRAJECTORY_BUILDER_2D.max_range = 8.0                               -- 大于 8.0m 的激光点直接丢掉   
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0                 -- 激光点缺失时，假设激光点在 5.0m 处
TRAJECTORY_BUILDER_2D.use_imu_data = false                          -- 不使用 IMU 数据
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true   -- 使用在线相关扫描匹配算法
-- 子图参数
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90                   -- 每个子地图包含 90 个激光点云数据
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05     -- 子地图分辨率为 0.05m
-- 位姿图参数
POSE_GRAPH.optimize_every_n_nodes = 90                              -- 每 90 个节点进行一次优化
POSE_GRAPH.constraint_builder.min_score = 0.55                      -- 约束构建器的最小分数为 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6   -- 全局定位的最小分数为 0.6

return options