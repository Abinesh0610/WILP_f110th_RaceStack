"""
racing_line_publisher.py
========================
Lightweight node that reads a pre-computed racing_line.csv and publishes
it as a nav_msgs/Path on /racing_line at 5 Hz.

This is used in simulation launches where the CSV is already generated,
so we skip the expensive map processing step and start racing immediately.
"""

import csv
import math
import os

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class RacingLinePublisher(Node):
    def __init__(self):
        super().__init__('racing_line_publisher')
        self.declare_parameter('racing_line_csv', '')

        csv_path = self.get_parameter('racing_line_csv').value
        if not csv_path or not os.path.isfile(csv_path):
            self.get_logger().error(
                f'racing_line_csv not found: {csv_path}\n'
                'Set it via: --ros-args -p racing_line_csv:=/path/to/racing_line.csv'
            )
            return

        self._path_msg = self._load_csv(csv_path)
        self.get_logger().info(
            f'Loaded {len(self._path_msg.poses)} waypoints from {csv_path}'
        )

        self._pub = self.create_publisher(Path, '/racing_line', 10)
        # Publish at 5 Hz so any late-starting subscriber (e.g. pure_pursuit) catches it
        self.create_timer(0.2, self._publish)

    def _load_csv(self, path: str) -> Path:
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        with open(path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = float(row['x'])
                pose.pose.position.y = float(row['y'])
                # Encode speed in z so pure_pursuit can read it
                pose.pose.position.z = float(row['target_speed'])
                heading = float(row['heading'])
                half_yaw = heading / 2.0
                pose.pose.orientation.z = math.sin(half_yaw)
                pose.pose.orientation.w = math.cos(half_yaw)
                path_msg.poses.append(pose)
        return path_msg

    def _publish(self):
        if self._path_msg is None:
            return
        self._path_msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RacingLinePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
