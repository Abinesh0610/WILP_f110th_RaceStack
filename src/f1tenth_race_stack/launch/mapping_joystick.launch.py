"""
mapping_joystick.launch.py
==========================
Launch file for Mode 1: Manual joystick-controlled SLAM mapping.

Nodes launched:
    1. slam_toolbox      — Online async mapping, saves map to maps/
    2. joy_node          — RadioMaster MT12 joystick via /dev/input/js0
    3. deadman_switch    — Manual drive from axes; gates on button 5 (SF)
    4. vesc_bridge       — Converts /drive to VESC topics
    5. race_state_machine — FSM in MAPPING_MANUAL state
    6. rviz2             — Visualisation with pre-configured .rviz file
    7. visualizer        — Race stack MarkerArray publisher

Usage:
    ros2 launch f1tenth_race_stack mapping_joystick.launch.py

Safety:
    Hold button 5 (SF switch) on RadioMaster MT12 to enable driving.
    Release it at any time -> car stops immediately at 50 Hz.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for manual joystick mapping mode."""

    pkg_share = get_package_share_directory('f1tenth_race_stack')
    config_dir = os.path.join(pkg_share, 'config')
    rviz_config = os.path.join(pkg_share, 'rviz', 'mapping.rviz')

    # ------------------------------------------------------------------
    # Launch arguments (override from CLI)
    # ------------------------------------------------------------------
    joy_device_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device path (RadioMaster MT12)'
    )

    # ------------------------------------------------------------------
    # SLAM Toolbox - online async mapping
    # ------------------------------------------------------------------
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(config_dir, 'slam_params.yaml'),
            {'mode': 'mapping'}
        ]
    )

    # ------------------------------------------------------------------
    # Joy node - reads RadioMaster MT12 joystick
    # ------------------------------------------------------------------
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': LaunchConfiguration('joy_dev'),
            'deadzone': 0.05,
            'autorepeat_rate': 50.0,
        }]
    )

    # ------------------------------------------------------------------
    # Deadman Switch - handles manual drive from joystick axes.
    # In MAPPING_MANUAL state: reads joy axes -> /drive only when button 5 held.
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
    # Race State Machine - start in MAPPING_MANUAL
    # ------------------------------------------------------------------
    state_machine_node = Node(
        package='f1tenth_race_stack',
        executable='race_state_machine',
        name='race_state_machine',
        output='screen',
        parameters=[{'initial_state': 'MAPPING_MANUAL'}]
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
    # RViz2 - delayed by 2s to allow nodes to start up
    # ------------------------------------------------------------------
    rviz_node = TimerAction(
        period=2.0,
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
        joy_device_arg,
        joy_node,        # Joy first - deadman depends on it
        slam_node,
        deadman_node,    # Safety gate before vesc_bridge
        vesc_bridge_node,
        state_machine_node,
        visualizer_node,
        rviz_node,
    ])
