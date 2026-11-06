#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('lidar_wall_detection')
    pkg_desc = get_package_share_directory('paint_bot_description')

    # Config files
    horizontal_params = os.path.join(pkg_dir, 'config', 'wall_detection_params.yaml')
    box_filter_params = os.path.join(pkg_dir, 'config', 'wall_box_filter.yaml')
    obstacle_params   = os.path.join(pkg_dir, 'config', 'obstacle_detector.yaml')
    cutout_params     = os.path.join(pkg_dir, 'config', 'cutout_detector.yaml')
    accumulator_params = os.path.join(pkg_dir, 'config', 'wall_3d_accumulator.yaml')
    rviz_config_path  = os.path.join(pkg_desc, 'rviz', 'wall_obstacle_simulation.rviz')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_horizontal = LaunchConfiguration('enable_horizontal')
    enable_vertical   = LaunchConfiguration('enable_vertical')
    enable_3d_cloud   = LaunchConfiguration('enable_3d_cloud')
    rviz              = LaunchConfiguration('rviz')

    # Declare Arguments
    declare_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_horizontal_cmd = DeclareLaunchArgument(
        'enable_horizontal', default_value='true',
        description='Launch horizontal 2D wall detection (RANSAC/PCA)'
    )
    declare_vertical_cmd = DeclareLaunchArgument(
        'enable_vertical', default_value='true',
        description='Launch vertical wall obstacle & cutout detection'
    )
    declare_3d_cloud_cmd = DeclareLaunchArgument(
        'enable_3d_cloud', default_value='true',
        description='Launch vertical scan-to-cloud conversion and 3D accumulator'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch RViz visualization'
    )

    # 1. Horizontal Wall Detection Node (RANSAC / PCA / EMA)
    wall_detection_node = Node(
        package='lidar_wall_detection',
        executable='wall_detection_node',
        name='wall_detection_node',
        output='screen',
        parameters=[
            horizontal_params,
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(enable_horizontal)
    )

    # 2. Vertical LaserScan Box Filter (crops scan along wall slice)
    box_filter_node = Node(
        package='lidar_wall_detection',
        executable='laser_scan_box_filter',
        name='laser_scan_box_filter',
        output='screen',
        parameters=[
            box_filter_params,
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(enable_vertical)
    )

    # 3. Obstacle Detector Node (protrusions / sockets / pipes)
    obstacle_detector_node = Node(
        package='lidar_wall_detection',
        executable='obstacle_detector',
        name='obstacle_detector',
        output='screen',
        parameters=[
            obstacle_params,
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(enable_vertical)
    )

    # 4. Cutout & Window Detector Node (windows / doors / openings)
    cutout_detector_node = Node(
        package='lidar_wall_detection',
        executable='cutout_window_detector',
        name='cutout_window_detector',
        output='screen',
        parameters=[
            cutout_params,
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(enable_vertical)
    )

    # 5. Wall Scan to PointCloud Node (converts /wall_scan -> /wall_scan_cloud)
    scan_to_cloud_node = Node(
        package='lidar_wall_detection',
        executable='wall_scan_to_pointcloud_node',
        name='wall_scan_to_pointcloud',
        output='screen',
        parameters=[{
            'scan_topic': '/wall_scan',
            'cloud_topic': '/wall_scan_cloud',
            'target_frame': 'laser',
            'publish_z': 0.0,
            'use_sim_time': use_sim_time,
        }],
        condition=IfCondition(enable_3d_cloud)
    )

    # 6. 3D Cloud Accumulator Node (accumulates wall cloud in odom frame)
    cloud_accumulator_node = Node(
        package='lidar_wall_detection',
        executable='cloud_accumulator_node',
        name='cloud_accumulator_wall_scan',
        output='screen',
        parameters=[
            accumulator_params,
            {
                'fixed_frame': 'odom',
                'lidar_frame': 'laser',
                'input_cloud_topic': '/wall_scan_cloud',
                'use_sim_time': use_sim_time,
            }
        ],
        condition=IfCondition(enable_3d_cloud)
    )

    # RViz visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz)
    )

    return LaunchDescription([
        declare_sim_time_cmd,
        declare_horizontal_cmd,
        declare_vertical_cmd,
        declare_3d_cloud_cmd,
        declare_rviz_cmd,

        wall_detection_node,
        box_filter_node,
        obstacle_detector_node,
        cutout_detector_node,
        scan_to_cloud_node,
        cloud_accumulator_node,
        rviz_node
    ])
