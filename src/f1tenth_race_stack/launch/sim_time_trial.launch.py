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

    slam_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(config_dir, 'slam_params.yaml'),
            {'mode': 'localization', 'map_file_name': LaunchConfiguration('map_path')}
        ]
    )

    racing_line_node = Node(
        package='f1tenth_race_stack',
        executable='racing_line_generator',
        name='racing_line_generator',
        parameters=[{
            'map_path': LaunchConfiguration('map_path'),
            'output_csv': LaunchConfiguration('racing_line_csv'),
            'v_max': 8.0,
            'v_min': 1.0,
            'curvature_gain': 2.5,
        }]
    )

    pure_pursuit_node = Node(
        package='f1tenth_race_stack',
        executable='pure_pursuit',
        name='pure_pursuit',
        output='screen',
        parameters=[
            os.path.join(config_dir, 'pure_pursuit_params.yaml'),
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
        parameters=[{'initial_state': 'TIME_TRIAL'}]
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

    rviz_node = TimerAction(
        period=1.0,
        actions=[Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config])]
    )

    return LaunchDescription([
        map_path_arg,
        racing_line_csv_arg,
        slam_node,
        racing_line_node,
        pure_pursuit_node,
        state_machine_node,
        visualizer_node,
        rviz_node,
        rqt_node,
    ])
