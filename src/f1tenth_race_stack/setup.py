from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'f1tenth_race_stack'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Register the package with ament index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install config YAML files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Install RViz config files
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
        # Install maps directory (placeholder)
        (os.path.join('share', package_name, 'maps'), []),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'scipy',
        'opencv-python',
        'matplotlib',
    ],
    zip_safe=True,
    maintainer='F1TENTH Racer',
    maintainer_email='racer@f1tenth.org',
    description='F1TENTH autonomous racing stack with MPPI + FTG on ROS 2 Humble',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Mapping module
            'follow_the_gap = f1tenth_race_stack.mapping.follow_the_gap:main',
            # Racing line generator (from map)
            'racing_line_generator = f1tenth_race_stack.racing_line.racing_line_generator:main',
            # Racing line publisher (from pre-computed CSV, used in simulation)
            'racing_line_publisher = f1tenth_race_stack.racing_line.racing_line_publisher:main',
            # MPPI Controller
            'mppi_controller = f1tenth_race_stack.controller.mppi_controller:main',
            # Pure Pursuit Controller
            'pure_pursuit = f1tenth_race_stack.controller.pure_pursuit:main',
            # State machine
            'race_state_machine = f1tenth_race_stack.state_machine.race_state_machine:main',
            # Utilities
            'vesc_bridge = f1tenth_race_stack.utils.vesc_bridge:main',
            'visualizer = f1tenth_race_stack.utils.visualizer:main',
            'deadman_switch = f1tenth_race_stack.utils.deadman_switch:main',
            # System ID
            'system_id_driver = f1tenth_race_stack.utils.system_id_driver:main',
            # Lap Counter (Time Trial popup)
            'lap_counter = f1tenth_race_stack.utils.lap_counter:main',
        ],
    },
)
