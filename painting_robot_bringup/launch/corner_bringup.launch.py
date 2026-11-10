
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():

    # ── Package paths ─────────────────────────────────────────────────────────
    pkg_bringup  = get_package_share_directory('painting_robot_bringup')
    pkg_painting = get_package_share_directory('painting_robot_painting')

    painting_params  = os.path.join(pkg_painting, 'config', 'actuator_bridge_params.yaml')
    gazebo_params_file = os.path.join(pkg_bringup, 'config', 'gazebo_params.yaml')
    world_file = LaunchConfiguration('world')
    pkg_description = get_package_share_directory('paint_bot_description')
    default_world = os.path.join(pkg_description, 'worlds', 'wall_obstacle_world.world')
    urdf_file = os.path.join(pkg_description, 'urdf', 'paint_bot_urdf.xacro')

    model_paths = [
        os.path.dirname(pkg_description),
        os.environ.get('GAZEBO_MODEL_PATH', '')
    ]
    gazebo_model_path_env = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH', ':'.join([p for p in model_paths if p]))

    # ── Args ──────────────────────────────────────────────────────────────────
    wall_detection_pkg_arg = DeclareLaunchArgument(
        'wall_detection_pkg',
        default_value='lidar_wall_detection',
        description='Package providing wall_detection_node (lidar_wall_detection or painting_robot_navigation)'
    )
    wall_detection_pkg = LaunchConfiguration('wall_detection_pkg')

    fake_paint_arg = DeclareLaunchArgument(
        'fake_paint',
        default_value='false',
        description='Enable fake paint actuator in env_monitoring_v2'
    )
    fake_paint = LaunchConfiguration('fake_paint')

    enable_internal_arg = DeclareLaunchArgument(
        'enable_internal',
        default_value='false',
        description='Enable internal corner detection in env_monitoring_v2'
    )
    enable_internal = LaunchConfiguration('enable_internal')

    enable_external_arg = DeclareLaunchArgument(
        'enable_external',
        default_value='false',
        description='Enable external corner detection in env_monitoring_v2'
    )
    enable_external = LaunchConfiguration('enable_external')

    enable_bumpout_arg = DeclareLaunchArgument(
        'enable_bumpout',
        default_value='false',
        description='Enable bumpout detection in env_monitoring_v2'
    )
    enable_bumpout = LaunchConfiguration('enable_bumpout')

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo without GUI (gzserver only)'
    )
    headless = LaunchConfiguration('headless')

    sim = LaunchConfiguration("sim")  # classic / ignition
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )
    rviz_flag = LaunchConfiguration('rviz')
    rviz_config_file = os.path.join(pkg_description, 'rviz', 'paint_bot.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(rviz_flag)
    )

    moveit_rviz_flag = LaunchConfiguration('moveit_rviz')

    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

    attach_gantry = LaunchConfiguration("attach_gantry")
    attach_sensors = LaunchConfiguration("attach_sensors")






    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "use_sim_time": True,
            "robot_description": ParameterValue(
                Command(['xacro ', urdf_file, ' sim:=classic attach_gantry:=true attach_sensors:=false']),
                value_type=str
            )
        }],
        output="screen"
    )

    # ── Gazebo headed / headless ──────────────────────────────────────────────
    gazebo_headed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': world_file,
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
        }.items(),
        condition=IfCondition(PythonExpression(["'", headless, "' == 'false'"]))
    )

    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gzserver.launch.py')
        ]),
        launch_arguments={
            'world': world_file,
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
        }.items(),
        condition=IfCondition(headless)
    )

    # ── Spawn robot ───────────────────────────────────────────────────────────


    # # for external 
    # spawn_entity = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=[
    #         '-topic', 'robot_description',
    #         '-entity', 'painting_robot',
    #         '-x', '3.68',
    #         # '-y', '-1.0', #vertex aligned to corner
    #         '-y', '-0.75',
    #         '-z', '0.1',
    #         '-Y', '-1.57'
    #     ],
    #     output='screen'
    # )

    # spawn_entity = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=[
    #         '-topic', 'robot_description',
    #         '-entity', 'painting_robot',
    #         '-x', '3.80',
    #         '-y', '2.0',
    #         '-z', '0.1',
    #         '-Y', '-3.1415'
    #     ],
    #     output='screen'
    # )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'painting_robot',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw
        ],
        output='screen'
    )

    # ── Controllers ───────────────────────────────────────────────────────────
    # diff_drive_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['paint_bot'],
    # )
    # joint_broad_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_state_broadcaster'],
    # )
    # axis_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['paint_axis_controller'],
    #     output='screen'
    # )

    # ── EKF (odometry provided directly by Gazebo diff_drive plugin) ───────────
    # ekf_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     remappings=[('odometry/filtered', '/paint_bot/odom_filtered')],
    #     parameters=[
    #         os.path.join(pkg_bringup, 'config', 'ekf.yaml'),
    #         {'use_sim_time': True}
    #     ]
    # )

    safety_monitor_node = Node(
        package='painting_robot_mcu',
        executable='safety_monitor_node',
        name='safety_monitor_node',
        output='screen',
        parameters=[{'use_sim_time': True, 'bypass': True}],
    )

    # ── Navigation ────────────────────────────────────────────────────────────
    common_alignment_params = os.path.join(pkg_bringup, 'config', 'common_alignment_params.yaml')

    wall_detection_node = Node(
        package=wall_detection_pkg,
        executable='wall_detection_node',
        name='wall_detection_node',
        output='screen',
        parameters=[
            os.path.join(pkg_bringup, 'config', 'wall_detection_params.yaml'),
            common_alignment_params,
            {'use_sim_time': True}
        ]
    )

    wall_motion_node = Node(
        package='painting_robot_navigation',
        executable='wall_motion_node',
        name='wall_motion_node',
        output='screen',
        parameters=[
            os.path.join(pkg_bringup, 'config', 'wall_motion_params.yaml'),
            common_alignment_params,
            {'use_sim_time': True}
        ]
    )

    # ── FSM ───────────────────────────────────────────────────────────────────


    fsm_node = Node(
        package='painting_robot_fsm',
        executable='state_machine_node',
        name='state_machine_node',
        output='screen',
        parameters=[
            common_alignment_params,
            {'use_sim_time': True, 'skip_homing_gate': True, 'auto_start': True},
        ],
    )

    # ── Perception ────────────────────────────────────────────────────────────
    env_monitoring_node = Node(
        package='painting_robot_perception',
        executable='env_monitoring_v2',
        name='env_monitoring_v2',
        output='screen',
        parameters=[
            common_alignment_params,           # single source: lidar_offset, target_distance
            os.path.join(pkg_bringup, 'config', 'env_monitoring_params.yaml'),
            {
                'use_sim_time': True,
                'enable_internal':  enable_internal,
                'enable_external':  enable_external,
                'enable_bumpout':   enable_bumpout,
                'fake_paint':       fake_paint,
            }
        ]
    )

    # ── Painting ─────────────────────────────────────────────────────────────
    paint_planner = Node(
        package='painting_robot_painting',
        executable='paint_planner_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'strip_z_min':  0.0,
            'strip_z_max':  1.2
        }]
    )

    paint_executor = Node(
        package='painting_robot_painting',
        executable='paint_executor_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    actuator_bridge = Node(
        package='painting_robot_painting',
        executable='actuator_bridge_node',
        output='screen',
        parameters=[painting_params, {'use_sim_time': True,'hardware_backend': 'sim','enable_odom': False}]
    )


    # ── Vertical LiDAR Wall & Obstacle Detection ──────────────────────────────
    enable_vertical_arg = DeclareLaunchArgument(
        'enable_vertical_lidar',
        default_value='true',
        description='Launch vertical LiDAR wall slice, obstacle, and cutout detection'
    )
    enable_vertical = LaunchConfiguration('enable_vertical_lidar')

    pkg_lidar = get_package_share_directory('lidar_wall_detection')
    box_filter_params = os.path.join(pkg_lidar, 'config', 'wall_box_filter.yaml')
    obstacle_params   = os.path.join(pkg_lidar, 'config', 'obstacle_detector.yaml')
    cutout_params     = os.path.join(pkg_lidar, 'config', 'cutout_detector.yaml')
    accumulator_params = os.path.join(pkg_lidar, 'config', 'wall_3d_accumulator.yaml')

    box_filter_node = Node(
        package='lidar_wall_detection',
        executable='laser_scan_box_filter',
        name='laser_scan_box_filter',
        output='screen',
        parameters=[box_filter_params, {'use_sim_time': True}],
        condition=IfCondition(enable_vertical)
    )

    obstacle_detector_node = Node(
        package='lidar_wall_detection',
        executable='obstacle_detector',
        name='obstacle_detector',
        output='screen',
        parameters=[obstacle_params, {'use_sim_time': True}],
        condition=IfCondition(enable_vertical)
    )

    cutout_detector_node = Node(
        package='lidar_wall_detection',
        executable='cutout_window_detector',
        name='cutout_window_detector',
        output='screen',
        parameters=[cutout_params, {'use_sim_time': True}],
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
            'use_sim_time': True,
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
                'use_sim_time': True,
            }
        ],
        condition=IfCondition(enable_vertical)
    )

    wall_depth_profile_node = Node(
        package='lidar_wall_detection',
        executable='wall_depth_profile_node',
        name='wall_depth_profile_node',
        output='screen',
        parameters=[
            common_alignment_params,
            {'use_sim_time': True}
        ],
        condition=IfCondition(enable_vertical)
    )

    return LaunchDescription([
        gazebo_model_path_env,
        # Args
        headless_arg,
        enable_vertical_arg,
        DeclareLaunchArgument('world', default_value=default_world,
                              description='Path to the Gazebo world file'),
        DeclareLaunchArgument('spawn_x', default_value='-1.20'),
        DeclareLaunchArgument('spawn_y', default_value='0.15'),
        DeclareLaunchArgument('spawn_z', default_value='0.10'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        wall_detection_pkg_arg,
        fake_paint_arg,
        enable_internal_arg,
        enable_external_arg,
        enable_bumpout_arg,
        rviz_arg,

        # Sim
        robot_state_publisher,
        gazebo_headed,
        gazebo_headless,
        spawn_entity,

        # Core
        wall_detection_node,
        wall_motion_node,
        fsm_node,
        env_monitoring_node,
        safety_monitor_node,

        # Vertical LiDAR Surface & Obstacle Detection
        box_filter_node,
        obstacle_detector_node,
        cutout_detector_node,
        scan_to_cloud_node,
        cloud_accumulator_node,
        wall_depth_profile_node,

        # Painting
        paint_planner,
        paint_executor,
        actuator_bridge,

        # Visualization
        rviz_node,
    ])




