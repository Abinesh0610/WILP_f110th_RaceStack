"""
mapping_autonomous.launch.py
============================
Launch file for Mode 2: Autonomous Follow-The-Gap SLAM mapping.

Nodes launched:
    1. slam_toolbox      - Online async mapping
    2. joy_node          - RadioMaster MT12 (required for deadman safety)
    3. follow_the_gap    - FTG reactive planner publishes to /drive_cmd
    4. deadman_switch    - Gates /drive_cmd -> /drive on button 5 held
    5. vesc_bridge       - Converts /drive to VESC topics
    6. race_state_machine - FSM in MAPPING_AUTO state
    7. visualizer        - Race stack MarkerArray publisher
    8. rviz2             - Visualisation

Usage:
    ros2 launch f1tenth_race_stack mapping_autonomous.launch.py

Safety:
    Hold button 5 (SF switch) on RadioMaster MT12 to allow FTG to drive.
    Release it at any time -> car stops immediately at 50 Hz.
    WARNING: Ensure the track is clear before launching.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for autonomous FTG mapping mode."""

    pkg_share = get_package_share_directory('f1tenth_race_stack')
    config_dir = os.path.join(pkg_share, 'config')
    rviz_config = os.path.join(pkg_share, 'rviz', 'mapping.rviz')

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    joy_device_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device (RadioMaster MT12) - for deadman safety'
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
    # Joy node - RadioMaster MT12 (required for deadman safety button)
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
    # Deadman Switch - safety gate in MAPPING_AUTO mode.
    # FTG publishes to /drive_cmd; deadman gates it to /drive.
    # Release button 5 at any time to immediately stop the car.
    # ------------------------------------------------------------------
    deadman_node = Node(
        package='f1tenth_race_stack',
        executable='deadman_switch',
        name='deadman_switch',
        output='screen',
        parameters=[os.path.join(config_dir, 'deadman_switch_params.yaml')]
    )

    # ------------------------------------------------------------------
    # Follow-The-Gap - autonomous mapping driver
    # Delayed by 3 s to allow SLAM to initialise first.
    # Publishes to /drive_cmd (gated by deadman_switch).
    # ------------------------------------------------------------------
    ftg_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='f1tenth_race_stack',
                executable='follow_the_gap',
                name='follow_the_gap',
                output='screen',
                parameters=[os.path.join(config_dir, 'ftg_params.yaml')]
            )
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
        parameters=[os.path.join(config_dir, 'vesc_bridge_params.yaml')]
    )

    # ------------------------------------------------------------------
    # Race State Machine - start in MAPPING_AUTO
    # ------------------------------------------------------------------
    state_machine_node = Node(
        package='f1tenth_race_stack',
        executable='race_state_machine',
        name='race_state_machine',
        output='screen',
        parameters=[{'initial_state': 'MAPPING_AUTO'}]
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
    # RViz2 - delayed by 2 s
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
        ftg_node,
        vesc_bridge_node,
        state_machine_node,
        visualizer_node,
        rviz_node,
    ])
