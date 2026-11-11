#!/usr/bin/env bash
# ==============================================================================
# Stitched Painting Robot & LiDAR Wall Obstacle Detection Simulation Launcher
# ==============================================================================

set -e

# 1. Kill any lingering simulator and ROS 2 nodes
killall -9 gzserver gzclient rviz2 2>/dev/null || true
pkill -9 -f "corner_bringup|painting_robot|lidar_wall_detection|wall_motion|wall_detection|state_machine|paint_executor|actuator_bridge|paint_planner|paint_bot" 2>/dev/null || true
sleep 1.0

# 2. Source ROS 2 Humble & Downloads workspace
source /opt/ros/humble/setup.bash
source /home/zeo/Downloads/install/setup.bash

# 3. Export Gazebo model path
export GAZEBO_MODEL_PATH="/home/zeo/Downloads/src:/home/zeo/Downloads/install/paint_bot_description/share:${GAZEBO_MODEL_PATH}"

echo "================================================================="
echo " Starting Stitched Simulation (Robot + FSM + Vertical LiDAR)"
echo "================================================================="
ros2 launch painting_robot_bringup corner_bringup.launch.py "$@"
