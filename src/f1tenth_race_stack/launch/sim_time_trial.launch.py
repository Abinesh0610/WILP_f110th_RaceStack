"""
sim_time_trial.launch.py
========================
Launch file for Time Trial racing mode in SIMULATION.

This is identical to `time_trial.launch.py`, but it removes hardware-specific
nodes (like vesc_bridge) so you can run the F1TENTH Gym simulator cleanly.

Usage:
    Terminal 1: ros2 launch f1tenth_gym_ros gym_bridge_launch.py
    Terminal 2: ros2 launch f1tenth_race_stack sim_time_trial.launch.py \\
                    map_path:=/path/to/maps/f1tenth_track.yaml \\
                    racing_line_csv:=/path/to/maps/racing_line.csv
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('f1tenth_race_stack')
    config_dir = os.path.join(pkg_share, 'config')
    rviz_config = os.path.join(pkg_share, 'rviz', 'racing.rviz')
    maps_dir = os.path.join(pkg_share, 'maps')

    map_path_arg = DeclareLaunchArgument(
        'map_path', default_value=os.path.join(maps_dir, 'f1tenth_track.yaml')
    )
    racing_line_csv_arg = DeclareLaunchArgument(
        'racing_line_csv', default_value=os.path.join(maps_dir, 'racing_line.csv')
    )

    # The racing_line.csv is PRE-GENERATED — just load and publish it
    # This is much faster than re-running the full generator on every launch
    racing_line_node = Node(
        package='f1tenth_race_stack',
        executable='racing_line_generator',
        name='racing_line_generator',
        output='screen',
        parameters=[{
            'map_path': '',           # Empty = skip map processing
            'output_csv': LaunchConfiguration('racing_line_csv'),
            'v_max': 8.0,
            'v_min': 1.0,
            'curvature_gain': 2.5,
            'publish_rate_hz': 5.0,  # Re-publish every 0.2s so late subscribers catch it
        }]
    )

    # Pre-load the racing line from CSV and publish it directly
    csv_publisher_node = Node(
        package='f1tenth_race_stack',
        executable='racing_line_publisher',
        name='racing_line_publisher',
        output='screen',
        parameters=[{
            'racing_line_csv': LaunchConfiguration('racing_line_csv'),
        }]
    )

    # Delay pure_pursuit by 5s to ensure racing_line is fully published
    pure_pursuit_node = TimerAction(
        period=5.0,
        actions=[Node(
            package='f1tenth_race_stack',
            executable='pure_pursuit',
            name='pure_pursuit',
            output='screen',
            parameters=[
                os.path.join(config_dir, 'pure_pursuit_params.yaml'),
                # NOTE: vehicle_params.yaml is plain YAML (no ros__parameters key)
                # and cannot be used with --params-file. Vehicle params are
                # already declared inside pure_pursuit_params.yaml.
            ],
            remappings=[
                ('/drive_cmd', '/drive'),
            ]
        )]
    )



    state_machine_node = Node(
        package='f1tenth_race_stack',
        executable='race_state_machine',
        name='race_state_machine',
        output='screen',
        parameters=[{'initial_state': 'TIME_TRIAL'}]
    )

    visualizer_node = Node(
        package='f1tenth_race_stack',
        executable='visualizer',
        name='race_stack_visualizer',
    )

    # Lap counter popup — starts after pure_pursuit is guaranteed running
    lap_counter_node = TimerAction(
        period=6.0,
        actions=[Node(
            package='f1tenth_race_stack',
            executable='lap_counter',
            name='lap_counter',
            output='screen',
            parameters=[{
                'departure_threshold': 2.5,
                'arrival_threshold': 1.5,
                'odom_topic': '/ego_racecar/odom',
            }]
        )]
    )

    # rqt_node = TimerAction(
    #     period=2.0,
    #     actions=[Node(package='rqt_reconfigure', executable='rqt_reconfigure', output='screen')]
    # )


    return LaunchDescription([
        map_path_arg,
        racing_line_csv_arg,

        csv_publisher_node,
        pure_pursuit_node,
        state_machine_node,
        visualizer_node,
        lap_counter_node,

        # rqt_node,
    ])
