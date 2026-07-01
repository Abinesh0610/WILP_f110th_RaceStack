"""
sim_head_to_head.launch.py
==========================
Launch file for Head-to-Head racing mode in SIMULATION.

This is identical to `head_to_head.launch.py`, but it removes hardware-specific
nodes (like vesc_bridge) so you can run the F1TENTH Gym simulator cleanly.

Usage:
    Terminal 1: ros2 launch f1tenth_gym_ros gym_bridge_launch.py
    Terminal 2: ros2 launch f1tenth_race_stack sim_head_to_head.launch.py \\
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

    racing_line_node = Node(
        package='f1tenth_race_stack',
        executable='racing_line_generator',
        name='racing_line_generator',
        parameters=[{
            'map_path': LaunchConfiguration('map_path'),
            'output_csv': LaunchConfiguration('racing_line_csv'),
            'v_max': 6.0,
            'v_min': 1.0,
            'curvature_gain': 3.0,
        }]
    )

    mppi_node = Node(
        package='f1tenth_race_stack',
        executable='mppi_controller',
        name='mppi_controller',
        output='screen',
        parameters=[
            os.path.join(config_dir, 'mppi_params.yaml'),
            os.path.join(config_dir, 'vehicle_params.yaml'),
        ],
        remappings=[
            ('/drive_cmd', '/drive')
        ]
    )



    state_machine_node = Node(
        package='f1tenth_race_stack',
        executable='race_state_machine',
        name='race_state_machine',
        parameters=[{'initial_state': 'HEAD_TO_HEAD'}]
    )

    visualizer_node = Node(
        package='f1tenth_race_stack',
        executable='visualizer',
        name='race_stack_visualizer',
    )

    rqt_node = TimerAction(
        period=2.0,
        actions=[Node(package='rqt_reconfigure', executable='rqt_reconfigure', output='screen')]
    )


    return LaunchDescription([
        map_path_arg,
        racing_line_csv_arg,

        racing_line_node,
        mppi_node,
        state_machine_node,
        visualizer_node,

        rqt_node,
    ])
