"""
head_to_head.launch.py
======================
Launch file for Head-to-Head racing mode.

Algorithm:  MPPI (Model Predictive Path Integral) — dynamic overtaking,
            opponent avoidance through obstacle-aware rollout sampling.
Safety:     Deadman switch on RadioMaster MT12 button 5 (SF switch).
            Release the button → car stops immediately at 50 Hz.
            In HEAD_TO_HEAD mode, the deadman also allows switching to
            MANUAL control instantly via axis inputs on the joystick.

Nodes launched:
    1.  slam_toolbox         — Localization-only (loads saved map)
    2.  racing_line_generator — Loads racing line CSV, publishes /racing_line
    3.  mppi_controller       — Head-to-head controller at 20 Hz → /drive_cmd
    4.  deadman_switch        — Gates /drive_cmd → /drive based on button 5
    5.  vesc_bridge           — Converts /drive → VESC ERPM + servo topics
    6.  race_state_machine    — FSM in HEAD_TO_HEAD state
    7.  joy_node              — RadioMaster MT12 joystick reader
    8.  visualizer            — RViz2 MarkerArray debugger
    9.  rviz2                 — Visualisation overlay
   10.  rqt_reconfigure       — Live parameter tuning GUI (MPPI weights)

Usage:
    ros2 launch f1tenth_race_stack head_to_head.launch.py \\
        map_path:=/path/to/maps/f1tenth_track.yaml \\
        racing_line_csv:=/path/to/maps/racing_line.csv

Live tuning during race:
    # Increase obstacle avoidance aggressiveness
    ros2 param set /mppi_controller mppi.weight_obstacle_penalty 800.0

    # Tighten track following
    ros2 param set /mppi_controller mppi.weight_reference_track 15.0

    # More exploration (better overtaking options)
    ros2 param set /mppi_controller mppi.lambda 0.15

    # Emergency manual override — publish to /race_mode:
    ros2 topic pub /race_mode std_msgs/String "data: MAPPING_MANUAL" --once
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for Head-to-Head racing mode."""

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
            'v_max': 6.0,          # More conservative for H2H (opponent blocking)
            'v_min': 1.0,
            'curvature_gain': 3.0,
            'smoothing_window': 15,
            'publish_rate_hz': 2.0,
        }]
    )

    # ------------------------------------------------------------------
    # MPPI Controller — Head-to-Head algorithm → publishes /drive_cmd
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
            'autorepeat_rate': 50.0,  # High rate for responsive deadman
        }]
    )

    # ------------------------------------------------------------------
    # Deadman Switch — CRITICAL SAFETY NODE
    # In HEAD_TO_HEAD mode:
    #   - Deadman held + autonomous → forwards MPPI /drive_cmd → /drive
    #   - Deadman released          → zero velocity immediately
    #   - Switch FSM to MAPPING_MANUAL → enables joystick manual override
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
    # Race State Machine — HEAD_TO_HEAD state
    # ------------------------------------------------------------------
    state_machine_node = Node(
        package='f1tenth_race_stack',
        executable='race_state_machine',
        name='race_state_machine',
        output='screen',
        parameters=[{'initial_state': 'HEAD_TO_HEAD'}]
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
    # rqt_reconfigure — live MPPI weight tuning (delayed 4 s)
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
        joy_node,        # Joy first — deadman depends on it
        slam_node,
        racing_line_node,
        mppi_node,
        deadman_node,    # Safety gate always before vesc_bridge
        vesc_bridge_node,
        state_machine_node,
        visualizer_node,
        rviz_node,
        rqt_node,
    ])
