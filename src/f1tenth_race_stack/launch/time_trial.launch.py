"""
time_trial.launch.py
====================
Launch file for Time Trial racing mode.

Algorithm:  Pure Pursuit (deterministic, jitter-free, optimal lap time)
Safety:     Deadman switch on RadioMaster MT12 button 5 (SF switch)
            Release the button → car stops immediately at 50 Hz.

Nodes launched:
    1. slam_toolbox         — Localization-only (loads saved map)
    2. racing_line_generator — Loads racing line CSV, publishes /racing_line
    3. pure_pursuit          — Time trial controller at 40 Hz → /drive_cmd
    4. deadman_switch        — Gates /drive_cmd → /drive based on button 5
    5. vesc_bridge           — Converts /drive → VESC ERPM + servo topics
    6. race_state_machine    — FSM in TIME_TRIAL state
    7. joy_node              — RadioMaster MT12 joystick reader
    8. visualizer            — RViz2 MarkerArray debugger
    9. rviz2                 — Visualisation overlay
   10. rqt_reconfigure       — Live parameter tuning GUI

Usage:
    ros2 launch f1tenth_race_stack time_trial.launch.py \\
        map_path:=/path/to/maps/f1tenth_track.yaml \\
        racing_line_csv:=/path/to/maps/racing_line.csv

Live tuning during race:
    ros2 param set /pure_pursuit pure_pursuit.speed_scale 0.9
    ros2 param set /pure_pursuit pure_pursuit.lookahead_distance 1.2
    ros2 param set /pure_pursuit pure_pursuit.lookahead_gain 0.3
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for Time Trial racing mode."""

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
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device (RadioMaster MT12)'
    )

    # ------------------------------------------------------------------
    # SLAM Toolbox — localization only
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
    # Racing Line Generator — publishes /racing_line at 1 Hz
    # ------------------------------------------------------------------
    racing_line_node = Node(
        package='f1tenth_race_stack',
        executable='racing_line_generator',
        name='racing_line_generator',
        output='screen',
        parameters=[{
            'map_path': LaunchConfiguration('map_path'),
            'output_csv': LaunchConfiguration('racing_line_csv'),
            'v_max': 8.0,          # Aggressive speed for time trials
            'v_min': 1.0,
            'curvature_gain': 2.5, # Slightly less speed reduction at corners
            'smoothing_window': 15,
            'publish_rate_hz': 2.0,
        }]
    )

    # ------------------------------------------------------------------
    # Pure Pursuit controller — Time Trial algorithm → publishes /drive_cmd
    # ------------------------------------------------------------------
    pure_pursuit_node = Node(
        package='f1tenth_race_stack',
        executable='pure_pursuit',
        name='pure_pursuit',
        output='screen',
        parameters=[
            os.path.join(config_dir, 'pure_pursuit_params.yaml'),
            os.path.join(config_dir, 'vehicle_params.yaml'),
        ]
    )

    # ------------------------------------------------------------------
    # Joy node — RadioMaster MT12
    # ------------------------------------------------------------------
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': LaunchConfiguration('joy_dev'),
            'deadzone': 0.05,
            'autorepeat_rate': 50.0,  # Higher rate for responsive deadman
        }]
    )

    # ------------------------------------------------------------------
    # Deadman Switch — CRITICAL SAFETY NODE
    # Gates /drive_cmd → /drive based on button 5 (SF switch)
    # Manual drive NOT active in TIME_TRIAL — only safety gate
    # ------------------------------------------------------------------
    deadman_node = Node(
        package='f1tenth_race_stack',
        executable='deadman_switch',
        name='deadman_switch',
        output='screen',
        parameters=[os.path.join(config_dir, 'deadman_switch_params.yaml'),
                    os.path.join(config_dir, 'vehicle_params.yaml')]
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
    # Race State Machine — TIME_TRIAL state
    # ------------------------------------------------------------------
    state_machine_node = Node(
        package='f1tenth_race_stack',
        executable='race_state_machine',
        name='race_state_machine',
        output='screen',
        parameters=[{'initial_state': 'TIME_TRIAL'}]
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
    # rqt_reconfigure — live tuning GUI (delayed 4 s)
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
    # RViz2 (delayed 3 s)
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
        joy_dev_arg,
        joy_node,        # Joy must be first — deadman depends on it
        slam_node,
        racing_line_node,
        pure_pursuit_node,
        deadman_node,    # Safety gate always launched before vesc_bridge
        vesc_bridge_node,
        state_machine_node,
        visualizer_node,
        rviz_node,
        rqt_node,
    ])
