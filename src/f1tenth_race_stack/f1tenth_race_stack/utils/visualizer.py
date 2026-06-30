"""
visualizer.py
=============
RViz2 MarkerArray publisher for debugging the F1TENTH race stack.

Purpose:
    Publishes a suite of RViz2 markers to help engineers visualise the
    internal state of the race stack during mapping and racing:
        - Racing line as a LINE_STRIP marker
        - Current car velocity as an ARROW marker
        - MPPI rollout "best trajectory" as a LINE_STRIP marker
        - Obstacle bubble as a SPHERE marker (closest LiDAR point)
        - Current FSM state as a TEXT_VIEW_FACING marker

Subscribes:
    /racing_line               (nav_msgs/Path)
    /ego_racecar/odom          (nav_msgs/Odometry)
    /race_stack/current_state  (std_msgs/String)
    /scan                      (sensor_msgs/LaserScan)

Publishes:
    /race_stack/markers (visualization_msgs/MarkerArray)
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray


class RaceStackVisualizer(Node):
    """
    Aggregates race stack data and publishes a rich MarkerArray to RViz2.
    """

    # Marker IDs — each marker type gets a unique integer ID
    MARKER_RACING_LINE = 0
    MARKER_VELOCITY_ARROW = 1
    MARKER_STATE_TEXT = 2
    MARKER_OBSTACLE_BUBBLE = 3

    def __init__(self) -> None:
        """Initialise all subscribers and the marker publisher."""
        super().__init__('race_stack_visualizer')

        # ------------------------------------------------------------------
        # Internal state cache
        # ------------------------------------------------------------------
        self._racing_line: Optional[Path] = None
        self._odom: Optional[Odometry] = None
        self._state_str: str = 'IDLE'
        self._scan: Optional[LaserScan] = None

        # ------------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------------
        self.create_subscription(Path, '/racing_line', self._rl_cb, 10)
        self.create_subscription(Odometry, '/ego_racecar/odom', self._odom_cb, 10)
        self.create_subscription(String, '/race_stack/current_state', self._state_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)

        # ------------------------------------------------------------------
        # Publisher
        # ------------------------------------------------------------------
        self._marker_pub = self.create_publisher(
            MarkerArray, '/race_stack/markers', 10)

        # ------------------------------------------------------------------
        # Publish at 10 Hz
        # ------------------------------------------------------------------
        self.create_timer(0.1, self._publish_markers)

        self.get_logger().info('RaceStackVisualizer started.')

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------

    def _rl_cb(self, msg: Path) -> None:
        """Cache racing line path."""
        self._racing_line = msg

    def _odom_cb(self, msg: Odometry) -> None:
        """Cache odometry."""
        self._odom = msg

    def _state_cb(self, msg: String) -> None:
        """Cache current FSM state string."""
        self._state_str = msg.data

    def _scan_cb(self, msg: LaserScan) -> None:
        """Cache latest scan."""
        self._scan = msg

    # ------------------------------------------------------------------
    # Marker construction helpers
    # ------------------------------------------------------------------

    def _base_marker(self, marker_id: int, frame: str = 'map') -> Marker:
        """
        Create a Marker with common fields pre-filled.

        Parameters
        ----------
        marker_id : int
            Unique marker ID for RViz2.
        frame : str
            TF frame the marker is published in.
        """
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = frame
        m.ns = 'race_stack'
        m.id = marker_id
        m.action = Marker.ADD
        return m

    def _racing_line_marker(self) -> Optional[Marker]:
        """Build a LINE_STRIP marker for the racing line (magenta)."""
        if self._racing_line is None or len(self._racing_line.poses) < 2:
            return None

        m = self._base_marker(self.MARKER_RACING_LINE)
        m.type = Marker.LINE_STRIP
        m.scale.x = 0.03  # Line width [m]
        m.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.8)  # Magenta

        for pose in self._racing_line.poses:
            pt = Point()
            pt.x = pose.pose.position.x
            pt.y = pose.pose.position.y
            pt.z = 0.02  # Slightly above ground
            m.points.append(pt)

        return m

    def _velocity_arrow_marker(self) -> Optional[Marker]:
        """Build an ARROW marker showing the car's current velocity vector (cyan)."""
        if self._odom is None:
            return None

        m = self._base_marker(self.MARKER_VELOCITY_ARROW, frame='ego_racecar/odom')
        m.type = Marker.ARROW
        m.pose = self._odom.pose.pose

        speed: float = self._odom.twist.twist.linear.x
        m.scale.x = max(0.05, abs(speed) * 0.1)  # Arrow length proportional to speed
        m.scale.y = 0.05
        m.scale.z = 0.05
        m.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.9)  # Cyan

        return m

    def _state_text_marker(self) -> Marker:
        """Build a TEXT_VIEW_FACING marker showing the current FSM state (yellow)."""
        m = self._base_marker(self.MARKER_STATE_TEXT, frame='map')
        m.type = Marker.TEXT_VIEW_FACING
        m.scale.z = 0.4  # Text height [m]
        m.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # Yellow
        m.text = f'STATE: {self._state_str}'

        # Position text above the car if odom is available
        if self._odom is not None:
            m.pose.position.x = self._odom.pose.pose.position.x
            m.pose.position.y = self._odom.pose.pose.position.y
            m.pose.position.z = 0.6  # 60 cm above ground
        else:
            m.pose.position.z = 0.5

        m.pose.orientation.w = 1.0
        return m

    def _obstacle_bubble_marker(self) -> Optional[Marker]:
        """
        Build a SPHERE marker at the closest LiDAR obstacle point (red).
        Positioned in the base_link frame.
        """
        if self._scan is None:
            return None

        ranges = np.array(self._scan.ranges)
        valid = np.isfinite(ranges) & (ranges > 0.05)
        if not np.any(valid):
            return None

        min_idx = int(np.argmin(np.where(valid, ranges, np.inf)))
        min_dist = float(ranges[min_idx])
        min_angle = (self._scan.angle_min
                     + min_idx * self._scan.angle_increment)

        m = self._base_marker(self.MARKER_OBSTACLE_BUBBLE, frame='base_link')
        m.type = Marker.SPHERE
        m.pose.position.x = min_dist * math.cos(min_angle)
        m.pose.position.y = min_dist * math.sin(min_angle)
        m.pose.position.z = 0.1
        m.pose.orientation.w = 1.0
        m.scale.x = 0.2
        m.scale.y = 0.2
        m.scale.z = 0.2
        m.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.7)  # Red

        return m

    # ------------------------------------------------------------------
    # Main publish callback
    # ------------------------------------------------------------------

    def _publish_markers(self) -> None:
        """Assemble all markers into a MarkerArray and publish."""
        marker_array = MarkerArray()

        # Collect all markers (filter out None values)
        candidates = [
            self._racing_line_marker(),
            self._velocity_arrow_marker(),
            self._state_text_marker(),
            self._obstacle_bubble_marker(),
        ]
        marker_array.markers = [m for m in candidates if m is not None]

        if marker_array.markers:
            self._marker_pub.publish(marker_array)


def main(args=None) -> None:
    """Entry point for the race_stack_visualizer ROS 2 node."""
    rclpy.init(args=args)
    node = RaceStackVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('RaceStackVisualizer shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
