"""
sim_mapping.launch.py
=====================
Launch file for Autonomous Mapping in SIMULATION.

This bypasses hardware (no vesc_bridge, no deadman switch).
The Follow-The-Gap algorithm will autonomously drive the simulated car
around the track while SLAM Toolbox builds the map.

Usage:
    Terminal 1: ros2 launch f1tenth_gym_ros gym_bridge_launch.py
    Terminal 2: ros2 launch f1tenth_race_stack sim_mapping.launch.py

After 2 laps, open Terminal 3 and save the map:
    ros2 run nav2_map_server map_saver_cli -f ~/ABINESH_Packages/racer_ws/src/f1tenth_race_stack/maps/f1tenth_track
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('f1tenth_race_stack')
    config_dir = os.path.join(pkg_share, 'config')
    rviz_config = os.path.join(pkg_share, 'rviz', 'mapping.rviz')

    # SLAM Toolbox - online async mapping
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

    # Follow-The-Gap - autonomous mapping driver
    # Bypasses deadman switch by remapping /drive_cmd -> /drive
    ftg_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='f1tenth_race_stack',
                executable='follow_the_gap',
                name='follow_the_gap',
                output='screen',
                parameters=[os.path.join(config_dir, 'ftg_params.yaml')],
                remappings=[('/drive_cmd', '/drive')]
            )
        ]
    )

    state_machine_node = Node(
        package='f1tenth_race_stack',
        executable='race_state_machine',
        name='race_state_machine',
        output='screen',
        parameters=[{'initial_state': 'MAPPING_AUTO'}]
    )

    visualizer_node = Node(
        package='f1tenth_race_stack',
        executable='visualizer',
        name='race_stack_visualizer',
        output='screen'
    )



    return LaunchDescription([
        slam_node,
        ftg_node,
        state_machine_node,
        visualizer_node,

    ])
