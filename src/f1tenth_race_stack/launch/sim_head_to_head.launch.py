"""
sim_head_to_head.launch.py
==========================
Launch file for Head-to-Head racing mode in SIMULATION.

Uses the MPPI controller which simulates 1000 rollouts per step to find
the optimal trajectory — automatically avoids opponent cars detected via LiDAR.

Usage:
    Terminal 1: ros2 launch f1tenth_gym_ros gym_bridge_launch.py
    Terminal 2: ros2 launch f1tenth_race_stack sim_head_to_head.launch.py \\
                    map_path:=/path/to/maps/levine_track.yaml \\
                    racing_line_csv:=/path/to/maps/levine_racing_line.csv
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
    maps_dir   = os.path.join(pkg_share, 'maps')

    map_path_arg = DeclareLaunchArgument(
        'map_path', default_value=os.path.join(maps_dir, 'levine_track.yaml')
    )
    racing_line_csv_arg = DeclareLaunchArgument(
        'racing_line_csv', default_value=os.path.join(maps_dir, 'levine_racing_line.csv')
    )

    # Load pre-computed CSV and publish /racing_line at 5 Hz
    # (much faster than re-generating from the map on every launch)
    csv_publisher_node = Node(
        package='f1tenth_race_stack',
        executable='racing_line_publisher',
        name='racing_line_publisher',
        output='screen',
        parameters=[{
            'racing_line_csv': LaunchConfiguration('racing_line_csv'),
        }]
    )

    # MPPI controller — delayed 5 s so racing line is published first
    # NOTE: vehicle_params.yaml is NOT passed here (plain YAML, not ROS params format)
    mppi_node = TimerAction(
        period=5.0,
        actions=[Node(
            package='f1tenth_race_stack',
            executable='mppi_controller',
            name='mppi_controller',
            output='screen',
            parameters=[
                os.path.join(config_dir, 'mppi_params.yaml'),
            ],
            remappings=[
                ('/drive_cmd', '/drive')
            ]
        )]
    )

    state_machine_node = Node(
        package='f1tenth_race_stack',
        executable='race_state_machine',
        name='race_state_machine',
        output='screen',
        parameters=[{'initial_state': 'HEAD_TO_HEAD'}]
    )

    visualizer_node = Node(
        package='f1tenth_race_stack',
        executable='visualizer',
        name='race_stack_visualizer',
        output='screen',
    )

    rqt_node = TimerAction(
        period=2.0,
        actions=[Node(package='rqt_reconfigure', executable='rqt_reconfigure', output='screen')]
    )

    return LaunchDescription([
        map_path_arg,
        racing_line_csv_arg,

        csv_publisher_node,
        mppi_node,
        state_machine_node,
        visualizer_node,

        rqt_node,
    ])
