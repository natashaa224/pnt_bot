#!/usr/bin/env python3
"""
robot_bringup.launch.py — Unified End-to-End Bringup for Painting Robot.

Operates in two modes via a single toggle:
  1. SIMULATION (is_sim:=true):
     - Launches Gazebo Classic with wall_obstacle_world.world
     - Spawns robot with virtual horizontal & vertical LiDARs and diff-drive plugins
     - Launches Actuator Bridge in 'sim' backend
     - Runs full perception, 3D obstacle/void accumulation, FSM, and painting pipeline
     - Launches RViz visualization

  2. REAL ROBOT AT SITE (is_sim:=false):
     - Uses real-time system clock (use_sim_time:=false)
     - Launches physical RPLidar hardware drivers (/scan and /wall_scan)
     - Launches Portenta MCU TCP bridge (pmc_io_map, IMU, tower lights)
     - Launches KVMS BMS battery monitoring
     - Configures Actuator Bridge with 'sbc' backend (Leadshine RS485 + Feetech Wrist + Solenoid GPIO)
     - Runs full perception, 3D obstacle/void accumulation, FSM, and painting pipeline
     - Launches Gamepad teleop for teach-homing and manual override
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ── Package Directories ───────────────────────────────────────────────────
    pkg_bringup     = get_package_share_directory('painting_robot_bringup')
    pkg_description = get_package_share_directory('paint_bot_description')
    pkg_painting    = get_package_share_directory('painting_robot_painting')
    pkg_lidar       = get_package_share_directory('lidar_wall_detection')
    pkg_gazebo_ros  = get_package_share_directory('gazebo_ros')

    # ── Configuration Files ───────────────────────────────────────────────────
    urdf_file             = os.path.join(pkg_description, 'urdf', 'paint_bot_urdf.xacro')
    default_world         = os.path.join(pkg_description, 'worlds', 'wall_obstacle_world.world')
    gazebo_params_file    = os.path.join(pkg_bringup, 'config', 'gazebo_params.yaml')

    model_paths = [
        os.path.dirname(pkg_description),
        os.environ.get('GAZEBO_MODEL_PATH', '')
    ]
    gazebo_model_path_env = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH', ':'.join([p for p in model_paths if p]))
    common_params         = os.path.join(pkg_bringup, 'config', 'common_alignment_params.yaml')
    painting_params       = os.path.join(pkg_bringup, 'config', 'actuator_bridge_params.yaml')
    h_detection_params    = os.path.join(pkg_bringup, 'config', 'wall_detection_params.yaml')
    wall_motion_params    = os.path.join(pkg_bringup, 'config', 'wall_motion_params.yaml')
    env_params            = os.path.join(pkg_bringup, 'config', 'env_monitoring_params.yaml')

    box_filter_params     = os.path.join(pkg_lidar, 'config', 'wall_box_filter.yaml')
    obstacle_params       = os.path.join(pkg_lidar, 'config', 'obstacle_detector.yaml')
    cutout_params         = os.path.join(pkg_lidar, 'config', 'cutout_detector.yaml')
    accumulator_params     = os.path.join(pkg_lidar, 'config', 'wall_3d_accumulator.yaml')

    rviz_config_sim       = os.path.join(pkg_description, 'rviz', 'paint_bot.rviz')

    # ── Launch Arguments ──────────────────────────────────────────────────────
    is_sim_arg = DeclareLaunchArgument(
        'is_sim', default_value='true',
        description='Set to true for Gazebo simulation, false for physical robot at site'
    )
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo without GUI (gzserver only) when in sim mode'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2 for monitoring'
    )
    enable_vertical_arg = DeclareLaunchArgument(
        'enable_vertical', default_value='true',
        description='Enable vertical LiDAR wall obstacle and cutout detection pipeline'
    )
    enable_fsm_arg = DeclareLaunchArgument(
        'enable_fsm', default_value='true',
        description='Launch high-level state machine autonomy node'
    )
    world_arg = DeclareLaunchArgument(
        'world', default_value=default_world,
        description='Gazebo world file (sim mode only)'
    )
    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='-1.20')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='0.15')
    spawn_z_arg = DeclareLaunchArgument('spawn_z', default_value='0.10')
    spawn_yaw_arg = DeclareLaunchArgument('spawn_yaw', default_value='0.0')

    # Values
    is_sim          = LaunchConfiguration('is_sim')
    headless        = LaunchConfiguration('headless')
    rviz_flag       = LaunchConfiguration('rviz')
    enable_vertical = LaunchConfiguration('enable_vertical')
    enable_fsm      = LaunchConfiguration('enable_fsm')
    world_file      = LaunchConfiguration('world')
    spawn_x         = LaunchConfiguration('spawn_x')
    spawn_y         = LaunchConfiguration('spawn_y')
    spawn_z         = LaunchConfiguration('spawn_z')
    spawn_yaw       = LaunchConfiguration('spawn_yaw')

    use_sim_time = is_sim

    # ── 1. Robot State Publisher (Common) ─────────────────────────────────────
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' sim:=classic attach_gantry:=true attach_sensors:=false']),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }]
    )

    # ── 2. Simulation Environment (is_sim == true) ────────────────────────────
    gazebo_headed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': world_file,
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
        }.items(),
        condition=IfCondition(PythonExpression(["'", is_sim, "' == 'true' and '", headless, "' == 'false'"]))
    )

    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ]),
        launch_arguments={
            'world': world_file,
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
        }.items(),
        condition=IfCondition(PythonExpression(["'", is_sim, "' == 'true' and '", headless, "' == 'true'"]))
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_painting_robot',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'painting_robot',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw,
            '-timeout', '60.0'
        ],
        condition=IfCondition(is_sim)
    )

    # ── 3. Real Robot Hardware Drivers (is_sim == false) ───────────────────────
    # Primary horizontal LiDAR
    rplidar_horiz = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_horiz_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/rplidar_horiz',
            'serial_baudrate': 460800,
            'frame_id': 'lidar_link',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }],
        condition=UnlessCondition(is_sim)
    )

    # Vertical scanning LiDAR
    rplidar_vert = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_vert_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/rplidar_vert',
            'serial_baudrate': 460800,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }],
        remappings=[('scan', '/wall_scan')],
        condition=UnlessCondition(is_sim)
    )

    # Portenta PMC TCP Bridge (IO / IMU / Tower lights)
    portenta_bridge = Node(
        package='painting_robot_mcu',
        executable='portenta_bridge_node',
        name='portenta_bridge_node',
        output='screen',
        condition=UnlessCondition(is_sim)
    )

    # Battery Management System (RS485)
    bms_node = Node(
        package='painting_robot_mcu',
        executable='bms_node',
        name='bms_node',
        output='screen',
        condition=UnlessCondition(is_sim)
    )

    # Gamepad Joystick Teleop
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'device_path': '/dev/input/js0', 'autorepeat_rate': 20.0, 'deadzone': 0.05}],
        condition=UnlessCondition(is_sim)
    )

    joy_teleop_node = Node(
        package='painting_robot_hmi',
        executable='joy_teleop_node',
        name='joy_teleop_node',
        output='screen',
        condition=UnlessCondition(is_sim)
    )

    # ── 4. Horizontal LiDAR Perception & Navigation ───────────────────────────
    wall_detection_node = Node(
        package='lidar_wall_detection',
        executable='wall_detection_node',
        name='wall_detection_node',
        output='screen',
        parameters=[
            h_detection_params,
            common_params,
            {'use_sim_time': use_sim_time}
        ]
    )

    wall_motion_node = Node(
        package='painting_robot_navigation',
        executable='wall_motion_node',
        name='wall_motion_node',
        output='screen',
        parameters=[
            wall_motion_params,
            common_params,
            {'use_sim_time': use_sim_time}
        ]
    )

    env_monitoring_node = Node(
        package='painting_robot_perception',
        executable='env_monitoring_v2',
        name='env_monitoring_v2',
        output='screen',
        parameters=[
            common_params,
            env_params,
            {
                'use_sim_time': use_sim_time,
                'enable_internal': True,
                'enable_external': True,
                'enable_bumpout':  True,
                'fake_paint':      False,
            }
        ]
    )

    # ── 5. Vertical LiDAR Wall Surface & Obstacle Stack ───────────────────────
    box_filter_node = Node(
        package='lidar_wall_detection',
        executable='laser_scan_box_filter',
        name='laser_scan_box_filter',
        output='screen',
        parameters=[box_filter_params, {'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_vertical)
    )

    obstacle_detector_node = Node(
        package='lidar_wall_detection',
        executable='obstacle_detector',
        name='obstacle_detector',
        output='screen',
        parameters=[obstacle_params, {'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_vertical)
    )

    cutout_detector_node = Node(
        package='lidar_wall_detection',
        executable='cutout_window_detector',
        name='cutout_window_detector',
        output='screen',
        parameters=[cutout_params, {'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_vertical)
    )

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
        condition=IfCondition(enable_vertical)
    )

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
        condition=IfCondition(enable_vertical)
    )

    wall_depth_profile_node = Node(
        package='lidar_wall_detection',
        executable='wall_depth_profile_node',
        name='wall_depth_profile_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_vertical)
    )

    # ── 6. Safety Authority ───────────────────────────────────────────────────
    safety_monitor_node = Node(
        package='painting_robot_mcu',
        executable='safety_monitor_node',
        name='safety_monitor_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'bypass': PythonExpression(["'", is_sim, "' == 'true'"]),
        }]
    )

    # ── 7. Painting Subsystem ─────────────────────────────────────────────────
    paint_planner = Node(
        package='painting_robot_painting',
        executable='paint_planner_node',
        name='paint_planner_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'strip_z_min': 0.0,
            'strip_z_max': 1.2
        }]
    )

    paint_executor = Node(
        package='painting_robot_painting',
        executable='paint_executor_node',
        name='paint_executor_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    actuator_bridge_sim = Node(
        package='painting_robot_painting',
        executable='actuator_bridge_node',
        name='actuator_bridge_node',
        output='screen',
        parameters=[
            painting_params,
            {'use_sim_time': True, 'hardware_backend': 'sim', 'enable_odom': False}
        ],
        condition=IfCondition(is_sim)
    )

    actuator_bridge_real = Node(
        package='painting_robot_painting',
        executable='actuator_bridge_node',
        name='actuator_bridge_node',
        output='screen',
        parameters=[
            painting_params,
            {'use_sim_time': False, 'hardware_backend': 'sbc'}
        ],
        condition=UnlessCondition(is_sim)
    )

    # ── 8. High-Level Autonomy FSM ────────────────────────────────────────────
    fsm_node = Node(
        package='painting_robot_fsm',
        executable='state_machine_node',
        name='state_machine_node',
        output='screen',
        parameters=[
            common_params,
            {
                'use_sim_time': use_sim_time,
                'skip_homing_gate': PythonExpression(["'", is_sim, "' == 'true'"]),
                'auto_start': PythonExpression(["'", is_sim, "' == 'true'"]),
            },
        ],
        condition=IfCondition(enable_fsm)
    )

    # ── 9. RViz2 Visualization ────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_sim],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz_flag)
    )

    return LaunchDescription([
        gazebo_model_path_env,
        is_sim_arg,
        headless_arg,
        rviz_arg,
        enable_vertical_arg,
        enable_fsm_arg,
        world_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        spawn_yaw_arg,

        robot_state_publisher,

        # Sim environment
        gazebo_headed,
        gazebo_headless,
        spawn_entity,

        # Real hardware drivers
        rplidar_horiz,
        rplidar_vert,
        portenta_bridge,
        bms_node,
        joy_node,
        joy_teleop_node,

        # Core navigation & perception
        wall_detection_node,
        wall_motion_node,
        env_monitoring_node,

        # Vertical LiDAR surface & obstacle detection
        box_filter_node,
        obstacle_detector_node,
        cutout_detector_node,
        scan_to_cloud_node,
        cloud_accumulator_node,
        wall_depth_profile_node,

        # Safety & Actuation
        safety_monitor_node,
        paint_planner,
        paint_executor,
        actuator_bridge_sim,
        actuator_bridge_real,

        # High-level FSM
        fsm_node,

        # Visualization
        rviz_node,
    ])
