import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
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
    world_file = os.path.join(
        get_package_share_directory('painting_robot_description'),
        'worlds', 'wall_world.world'
    )
    
    
    pkg_description = get_package_share_directory('paint_bot_description')
    urdf_file = os.path.join(pkg_description, 'urdf', 'paint_bot_urdf.xacro')

    # ── Args ──────────────────────────────────────────────────────────────────
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo without GUI (gzserver only)'
    )
    headless = LaunchConfiguration('headless')

    sim = LaunchConfiguration("sim")  # classic / ignition
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    rviz_flag = LaunchConfiguration('rviz')
    moveit_rviz_flag = LaunchConfiguration('moveit_rviz')

    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")

    attach_gantry = LaunchConfiguration("attach_gantry")
    attach_sensors = LaunchConfiguration("attach_sensors")




    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "use_sim_time": True,
            "robot_description": ParameterValue(
                Command(['xacro ', urdf_file, ' sim:=classic attach_gantry:=false attach_sensors:=false']),
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


    # ── Spawn robot ───────────────────────────────────────────────────────────

    spawn_entity = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
        '-topic', 'robot_description',
        '-entity', 'painting_robot',
        '-x', '-5.5',
        '-y', '11.52',
        '-z', '0.1',
        '-Y', '3.1416'  # facing front wall (negative Y direction)
    ],
    output='screen'
)

    # ── Controllers ───────────────────────────────────────────────────────────


    diff_drive_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['paint_bot'],
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    axis_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['paint_axis_controller'],
        output='screen'
    )

    # ── EKF ───────────────────────────────────────────────────────────────────
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_bringup, 'config', 'ekf.yaml'),
            {
                'use_sim_time': False,
                'publish_tf': False,
            }
        ],
        remappings=[
            ('/odometry/filtered', '/paint_bot/odom_filtered'),
        ],
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'device_path': '/dev/input/js0'}]
    )

    joy_teleop_node = Node(
        package='painting_robot_hmi',
        executable='joy_teleop_node',
        name='joy_teleop_node',
        output='screen'
    )

    # -------------------------
    common_alignment_params = os.path.join(pkg_bringup, 'config', 'common_alignment_params.yaml')
    
    wall_detection_node = Node(
            package='painting_robot_navigation',
            executable='wall_detection_node',
            name='wall_detection_node',
            output='screen',
            parameters=[
                os.path.join(pkg_bringup, 'config', 'wall_detection_params.yaml'),
                common_alignment_params,
                {'use_sim_time': False}
            ]
    )



    # ── Perception ────────────────────────────────────────────────────────────
    # env_monitoring_node = Node(
    #     package='painting_robot_perception',
    #     executable='env_monitoring_node',
    #     name='env_monitoring_node',
    #     output='screen',
    #     parameters=[
    #         os.path.join(pkg_bringup, 'config', 'env_monitoring_params.yaml'),
    #         {'use_sim_time': True}
    #     ]
    # )
    
    env_monitoring_v2_node = Node(
        package='painting_robot_perception',
        executable='env_monitoring_v2',
        name='env_monitoring_v2',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'internal_trigger': 0.71,      # LiDAR-frame front-wall preempt (m)
            'internal_approach_dist': 1.00,
            'vote_window': 5,
            'vote_threshold': 3,
            'enable_internal': True,
            'enable_external': True,
            'enable_bumpout': True,
            'fake_paint': True,            # no actuator running
        }]
    )
    

    logging_node = Node(
        package='painting_robot_hmi',
        executable='logging_node',
        name='logging_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )


    safety_monitor_node = Node(
        package='painting_robot_mcu',
        executable='safety_monitor_node',
        name='safety_monitor_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
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

    fsm_node = Node(
        package='painting_robot_fsm',
        executable='state_machine_node',
        name='state_machine_node',
        output='screen',
        parameters=[{'use_sim_time': True,'skip_homing_gate': True,}],
    )

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
        parameters=[painting_params, {'use_sim_time': True,'hardware_backend': 'sim'}]
    )


    return LaunchDescription([
        # Args
        headless_arg,
        DeclareLaunchArgument('spawn_x', default_value='1'),
        DeclareLaunchArgument('spawn_y', default_value='12.1'),
        DeclareLaunchArgument('spawn_z', default_value='0.01'),

        # Sim
        robot_state_publisher,
        gazebo_headed,
        spawn_entity,

        # Controllers
        diff_drive_spawner,
        joint_broad_spawner,
        axis_spawner,

        # Core
        ekf_node,
        wall_detection_node,
        wall_motion_node,
        fsm_node,
        env_monitoring_v2_node,
        safety_monitor_node,
        # joy_node,
        # joy_teleop_node,

        # Painting
        paint_planner,
        paint_executor,
        # actuator_bridge,
    ])
