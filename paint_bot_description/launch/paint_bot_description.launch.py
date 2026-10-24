#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # -------------------------
    # Launch Config
    # -------------------------
    sim = LaunchConfiguration("sim")  # classic / ignition
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")


    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")

    # -------------------------
    # Packages
    # -------------------------
    pkg_description = FindPackageShare("painting_truebot_description").find("painting_truebot_description")
    pkg_gazebo = FindPackageShare("gazebo_ros").find("gazebo_ros")
    pkg_painting = FindPackageShare("painting_robot_painting").find("painting_robot_painting")
    painting_params = os.path.join(pkg_painting, "config", "actuator_bridge_params.yaml")

    urdf_file = PathJoinSubstitution([pkg_description, "urdf", "painting_bot_true.urdf.xacro"])


    # -------------------------
    # Robot State Publisher
    # -------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_description": ParameterValue(
                Command([
                    "xacro ",
                    urdf_file,
                    " sim:=",
                    sim
                ]),
                value_type=None
            )
        }],
        output="screen"
    )

    # ============================================================
    # 🔵 CLASSIC GAZEBO
    # ============================================================
    classic_gazebo = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo, "launch", "gzserver.launch.py")
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo, "launch", "gzclient.launch.py")
            ),
            condition=IfCondition(gui)
        ),

        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-topic", "robot_description",
                "-entity", "paint_bot",
                "-x", spawn_x,
                "-y", spawn_y,
                "-z", spawn_z
            ],
            output="screen"
        )
    ],
    condition=IfCondition(PythonExpression(["'", sim, "' == 'classic'"]))
    )

  

    diff_drive_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['paint_bot'],
    )

    # was: joint_broad  →  now: joint_state_broadcaster
    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    axis_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["paint_axis_controller"],
        output="screen"
    )

    

    paint_planner = Node(
        package="painting_robot_painting",
        executable="paint_planner_node",
        parameters=[{"use_sim_time": use_sim_time,
                    "strip_z_min": 0.0,
                    "strip_z_max": 1.2}],
        output="screen"
    )

    paint_executor = Node(
        package="painting_robot_painting",
        executable="paint_executor_node",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen"
    )

    actuator_bridge = Node(
        package="painting_robot_painting",
        executable="actuator_bridge_node",
        parameters=[painting_params, {"use_sim_time": use_sim_time}],
        output="screen"
    )


    return LaunchDescription([

        # Args
        DeclareLaunchArgument("sim", default_value="classic"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true"),

        DeclareLaunchArgument("spawn_x", default_value="0.0"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        DeclareLaunchArgument("spawn_z", default_value="0.15"),


        # Nodes
        robot_state_publisher,
        classic_gazebo,
        joint_broad_spawner,
        diff_drive_spawner,
        axis_spawner,
        paint_planner,
        paint_executor,
        actuator_bridge
    ])
