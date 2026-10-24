from launch import LaunchDescription
from launch.actions import (
    RegisterEventHandler, DeclareLaunchArgument,
    TimerAction, IncludeLaunchDescription
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

def generate_launch_description():

    pkg_bringup = get_package_share_directory('painting_robot_bringup')

    use_depth = LaunchConfiguration('use_depth')

    # -------------------------
    # RSP — always on, no tier
    # -------------------------
        # ── Package paths ─────────────────────────────────────────────────────────
    pkg_bringup  = get_package_share_directory('painting_robot_bringup')
    pkg_painting = get_package_share_directory('painting_robot_painting')

    pkg_description = get_package_share_directory('paint_bot_description')
    urdf_file = os.path.join(pkg_description, 'urdf', 'paint_bot_urdf.xacro')

    painting_params  = os.path.join(pkg_bringup, 'config', 'actuator_bridge_params.yaml')

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_file, ' use_ros2_control:=false sim_mode:=false']),
                value_type=str
            )
        }]
    )


    actuator_bridge = Node(
        package='painting_robot_painting',
        executable='actuator_bridge_node',
        output='screen',
        parameters=[
            painting_params,
            {'use_sim_time': False, 'hardware_backend': 'sbc'}  # override backend only
        ]
    )

    portenta_bridge = Node(
            package='painting_robot_mcu',
            executable='portenta_bridge_node',
            name='portenta_bridge_node',
            output='screen',
    )

    rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            # 'serial_port': '/dev/ttyUSB1',
            'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_ba91da1af96ef011aeb3529b1045c30f-if00-port0',

            'serial_baudrate': 460800,
            'frame_id': 'lidar_link',   # check frame id
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }],
    )

    cliff_rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='cliff_rplidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_66495e31006ff011b7f3499b1045c30f-if00-port0',
            'serial_baudrate': 460800,
            'frame_id': 'cliff_laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }],
        remappings=[('scan', '/scan_cliff')],
    )

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
        parameters=[{'device_path': '/dev/input/js0','autorepeat_rate': 20.0, 'deadzone': 0.05}],
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


    # env_monitoring_node = Node(
    #     package='painting_robot_perception',
    #     executable='env_monitoring_node',
    #     name='env_monitoring_node',
    #     output='screen',
    #     parameters=[
    #         os.path.join(pkg_bringup, 'config', 'env_monitoring_params.yaml'),
    #         {'use_sim_time': False}
    #     ]
    # )

    env_monitoring_node = Node(
        package='painting_robot_perception',
        executable='env_monitoring_v2',
        name='env_monitoring_v2',
        output='screen',
        parameters=[
            os.path.join(pkg_bringup, 'config', 'common_params.yaml'),           # single source: lidar_offset, target_distance
            os.path.join(pkg_bringup, 'config', 'env_monitoring_params.yaml'),
            {
                'use_sim_time': False,
                # booleans kept inline for quick on/off from the launch file
                'enable_internal':  True,
                'enable_external':  True,
                'enable_bumpout':   True,
                'fake_paint':       False,
            }
        ]
    )
 
        # ── Perception / safety sensors ───────────────────────────────────────────
    cliff_detection_node = Node(
        package='painting_robot_perception',
        executable='cliff_detection_node',
        name='cliff_detection_node',
        output='screen',
        parameters=[
            os.path.join(pkg_bringup, 'config', 'env_monitoring_params.yaml'),
            {'use_sim_time': False}
        ],
    )

    ultrasound_node = Node(
        package='painting_robot_perception',
        executable='ultrasound_node',
        name='ultrasound_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            # 'port': '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B3D044007-if00',
            'port': '/dev/ttyUSB2',
        }],
    )

    logging_node = Node(
        package='painting_robot_hmi',
        executable='logging_node',
        name='logging_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    bms_node = Node(
        package='painting_robot_mcu',
        executable='bms_node',
        name='bms_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'port': '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG03WRYV-if00-port0',
        }],
    )

    safety_monitor_node = Node(
        package='painting_robot_mcu',
        executable='safety_monitor_node',
        name='safety_monitor_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )


    collision_warn_node = Node(
        package='painting_robot_perception',
        executable='collision_warn_node',
        name='collision_warn_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'warn_distance': 0.5,
            'clear_distance': 0.6,
            'fault_code': 'COLLISION_WARN',
        }],
    )


    wall_motion_node = Node(
                package='painting_robot_navigation',
                executable='wall_motion_node',
                name='wall_motion_node',
                output='screen',
                parameters=[
                    os.path.join(pkg_bringup, 'config', 'wall_motion_params.yaml'),
                    common_alignment_params,
                    {'use_sim_time': False}
                ]
        )



    fsm_node = Node(
        package='painting_robot_fsm',
        executable='state_machine_node',
        name='state_machine_node',
        output='screen',
        parameters=[
            common_alignment_params,
            {'use_sim_time': False, 'skip_homing_gate': False},
        ],
    )

    paint_planner = Node(
        package='painting_robot_painting',
        executable='paint_planner_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'strip_z_min':  0.0,
            'strip_z_max':  1.2
        }]
    )

    paint_executor = Node(
        package='painting_robot_painting',
        executable='paint_executor_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # -------------------------
    # Launch Description
    # -------------------------
    return LaunchDescription([
        DeclareLaunchArgument('use_depth', default_value='false'),

        rsp,
        portenta_bridge,
                # Controllers


        # Core
        # logging_node,
        safety_monitor_node,
        ekf_node,
        rplidar,
        # cliff_rplidar,
        wall_detection_node,
        wall_motion_node,
        fsm_node,
        env_monitoring_node,
        joy_node,
        joy_teleop_node,

        #sensor
        # bms_node,
        # cliff_detection_node,
        # ultrasound_node,
      

        # Painting
        paint_planner,
        paint_executor,
        actuator_bridge,

    ])
