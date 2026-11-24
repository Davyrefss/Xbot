#!/usr/bin/env bash
# 一键清理 Gazebo / 仿真相关进程
# 典型用途: 反复 launch 后残留多个 gz server, 同时发 /tf, 导致 Nav2 "Failed to make progress"
set +e
pkill -9 -f "gz sim"
pkill -9 -f gz_tools_vendor
pkill -9 -f parameter_bridge
pkill -9 -f async_slam_toolbox
pkill -9 -f component_container
pkill -9 -f "ros2 launch"
pkill -9 -f robot_state_publisher
pkill -9 -f sim2d_node
sleep 1
echo "remaining: gz=$(pgrep -c -x gz 2>/dev/null || echo 0) ruby=$(pgrep -c -x ruby 2>/dev/null || echo 0)"
