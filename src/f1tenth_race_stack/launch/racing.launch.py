"""
racing.launch.py
================
Launch file for Mode 3: Full MPPI racing mode.

Nodes launched:
    1. slam_toolbox         — Localization-only mode (loads saved map)
    2. racing_line_generator — Loads racing line CSV and publishes /racing_line
    3. mppi_controller       — MPPI racing controller (20 Hz)
    4. vesc_bridge           — Converts /drive to VESC topics
    5. race_state_machine    — FSM in RACING state
    6. visualizer            — Race stack MarkerArray publisher
    7. rqt_reconfigure       — Live parameter tuning GUI (auto-opened)
    8. rviz2                 — Visualisation with racing overlay

Usage:
    ros2 launch f1tenth_race_stack racing.launch.py \
        map_path:=/path/to/maps/f1tenth_track.yaml \
        racing_line_csv:=/path/to/maps/racing_line.csv

Live tuning during race:
    ros2 param set /mppi_controller mppi.weight_reference_track 15.0
    ros2 param set /mppi_controller mppi.K 1500
    (or use the rqt_reconfigure GUI opened automatically)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for full MPPI racing mode."""

    pkg_share = get_package_share_directory('f1tenth_race_stack')
    config_dir = os.path.join(pkg_share, 'config')
    rviz_config = os.path.join(pkg_share, 'rviz', 'racing.rviz')
    maps_dir = os.path.join(pkg_share, 'maps')

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value=os.path.join(maps_dir, 'f1tenth_track.yaml'),
        description='Absolute path to the SLAM-built .yaml map file'
    )

    racing_line_csv_arg = DeclareLaunchArgument(
        'racing_line_csv',
        default_value=os.path.join(maps_dir, 'racing_line.csv'),
        description='Absolute path to the racing line CSV'
    )

    # ------------------------------------------------------------------
    # SLAM Toolbox — localization-only (loads saved map)
    # ------------------------------------------------------------------
    slam_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(config_dir, 'slam_params.yaml'),
            {
                'mode': 'localization',
                'map_file_name': LaunchConfiguration('map_path'),
            }
        ]
    )

    # ------------------------------------------------------------------
    # Racing Line Generator — loads CSV and publishes at 1 Hz
    # ------------------------------------------------------------------
    racing_line_node = Node(
        package='f1tenth_race_stack',
        executable='racing_line_generator',
        name='racing_line_generator',
        output='screen',
        parameters=[{
            'map_path': LaunchConfiguration('map_path'),
            'output_csv': LaunchConfiguration('racing_line_csv'),
            'v_max': 6.0,
            'v_min': 1.0,
            'curvature_gain': 3.0,
            'smoothing_window': 15,
            'publish_rate_hz': 1.0,
        }]
    )

    # ------------------------------------------------------------------
    # MPPI Controller — racing controller at 20 Hz
    # ------------------------------------------------------------------
    mppi_node = Node(
        package='f1tenth_race_stack',
        executable='mppi_controller',
        name='mppi_controller',
        output='screen',
        parameters=[
            os.path.join(config_dir, 'mppi_params.yaml'),
            os.path.join(config_dir, 'vehicle_params.yaml'),
        ]
    )

    # ------------------------------------------------------------------
    # VESC Bridge
    # ------------------------------------------------------------------
    vesc_bridge_node = Node(
        package='f1tenth_race_stack',
        executable='vesc_bridge',
        name='vesc_bridge',
        output='screen',
        parameters=[os.path.join(config_dir, 'vesc_bridge_params.yaml'),
                    os.path.join(config_dir, 'vehicle_params.yaml')]
    )

    # ------------------------------------------------------------------
    # Race State Machine — start in RACING state
    # ------------------------------------------------------------------
    state_machine_node = Node(
        package='f1tenth_race_stack',
        executable='race_state_machine',
        name='race_state_machine',
        output='screen',
        parameters=[{'initial_state': 'RACING'}]
    )

    # ------------------------------------------------------------------
    # Visualizer
    # ------------------------------------------------------------------
    visualizer_node = Node(
        package='f1tenth_race_stack',
        executable='visualizer',
        name='race_stack_visualizer',
        output='screen'
    )

    # ------------------------------------------------------------------
    # rqt_reconfigure — live parameter tuning GUI (delayed 4s for startup)
    # ------------------------------------------------------------------
    rqt_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='rqt_reconfigure',
                executable='rqt_reconfigure',
                name='rqt_reconfigure',
                output='screen',
            )
        ]
    )

    # ------------------------------------------------------------------
    # RViz2 — delayed by 3 s
    # ------------------------------------------------------------------
    rviz_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
            )
        ]
    )

    return LaunchDescription([
        map_path_arg,
        racing_line_csv_arg,
        slam_node,
        racing_line_node,
        mppi_node,
        vesc_bridge_node,
        state_machine_node,
        visualizer_node,
        rviz_node,
        rqt_node,
    ])
