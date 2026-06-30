"""
system_id.launch.py
===================
Launch file for Mode 4: System identification drive.

Purpose:
    Drive the car at a sequence of fixed speeds in a straight line and
    record odometry. Use the output to calibrate:
        - erpm_gain: ERPM per m/s (for vesc_bridge)
        - steering_to_servo_gain and offset (by hand after visual inspection)

Nodes launched:
    1. system_id_driver — Open-loop speed ramp node
    2. vesc_bridge      — Converts /drive to VESC topics

Usage:
    ros2 launch f1tenth_race_stack system_id.launch.py

Prerequisites:
    - Place the car in a long, clear straight (min 10 m)
    - Ensure VESC driver is running
    - Have someone ready to catch / stop the car if needed

Output:
    Console log with commanded vs measured speed table.
    Use these values to compute: erpm_gain = VESC_ERPM / measured_speed_mps
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for system identification mode."""

    pkg_share = get_package_share_directory('f1tenth_race_stack')
    config_dir = os.path.join(pkg_share, 'config')

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    test_speeds_arg = DeclareLaunchArgument(
        'test_speeds',
        default_value='[0.5, 1.0, 1.5, 2.0, 2.5, 3.0]',
        description='List of speeds to test in m/s'
    )

    test_duration_arg = DeclareLaunchArgument(
        'test_duration_s',
        default_value='3.0',
        description='Duration to drive at each test speed [s]'
    )

    startup_delay_arg = DeclareLaunchArgument(
        'startup_delay_s',
        default_value='5.0',
        description='Countdown before system ID starts [s]'
    )

    # ------------------------------------------------------------------
    # System ID Driver
    # ------------------------------------------------------------------
    sysid_node = Node(
        package='f1tenth_race_stack',
        executable='system_id_driver',
        name='system_id_driver',
        output='screen',
        parameters=[{
            'test_speeds': LaunchConfiguration('test_speeds'),
            'test_duration_s': float(LaunchConfiguration('test_duration_s')),
            'startup_delay_s': float(LaunchConfiguration('startup_delay_s')),
        }]
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

    return LaunchDescription([
        test_speeds_arg,
        test_duration_arg,
        startup_delay_arg,
        sysid_node,
        vesc_bridge_node,
    ])
