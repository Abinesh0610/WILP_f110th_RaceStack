"""
follow_the_gap.py
=================
Autonomous mapping driver using the Follow-The-Gap (FTG) algorithm.

Purpose:
    Drive the F1TENTH car safely around an unknown track while SLAM Toolbox
    runs in the background to build a map. FTG is chosen for mapping because
    it is reactive, requires no prior map, and operates well at low-to-medium
    speeds.

Algorithm Summary:
    1. Receive LaserScan, replace invalid readings with max_lidar_range.
    2. Find the closest obstacle point and zero out a safety bubble around it.
    3. Find the largest contiguous non-zero gap in the modified scan.
    4. Within that gap, choose the point with the maximum range as the
       steering target.
    5. Convert the target index to a steering angle and compute a speed that
       decreases proportionally with steering magnitude.
    6. Publish AckermannDriveStamped to /drive.

All parameters are live-tunable via:
    ros2 param set /follow_the_gap ftg.<param_name> <value>
"""

from __future__ import annotations

import math
from typing import List, Tuple

import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan


class FollowTheGapNode(Node):
    """
    ROS 2 node implementing the Follow-The-Gap reactive planner.

    Subscribes:
        /scan (sensor_msgs/LaserScan)

    Publishes:
        /drive (ackermann_msgs/AckermannDriveStamped)

    All parameters are declared via declare_parameter and can be changed
    live using ros2 param set or rqt_reconfigure.
    """

    def __init__(self) -> None:
        """Initialise the node, declare parameters, set up pub/sub."""
        super().__init__('follow_the_gap')

        # ------------------------------------------------------------------
        # Declare all tunable parameters (sourced from ftg_params.yaml)
        # ------------------------------------------------------------------
        self.declare_parameter('ftg.max_speed', 3.0)
        self.declare_parameter('ftg.min_speed', 0.5)
        self.declare_parameter('ftg.bubble_radius', 0.40)
        self.declare_parameter('ftg.max_lidar_range', 10.0)
        self.declare_parameter('ftg.gap_selection', 'largest')
        self.declare_parameter('ftg.steering_gain', 1.0)

        # ------------------------------------------------------------------
        # Vehicle geometry (from vehicle_params.yaml for self-containment)
        # ------------------------------------------------------------------
        self.declare_parameter('vehicle.max_steering_angle', 0.43)

        # Load initial parameter values into instance variables
        self._load_params()

        # ------------------------------------------------------------------
        # Live parameter callback — updates internal state on any param change
        # ------------------------------------------------------------------
        self.add_on_set_parameters_callback(self._param_callback)

        # ------------------------------------------------------------------
        # Subscriber: LiDAR scan
        # ------------------------------------------------------------------
        self._scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self._scan_callback,
            10
        )

        # ------------------------------------------------------------------
        # Publisher: drive commands — gated by deadman_switch → /drive
        # ------------------------------------------------------------------
        self._drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive_cmd',
            10
        )

        self.get_logger().info(
            f'FollowTheGap node started | '
            f'max_speed={self.max_speed:.2f} m/s | '
            f'bubble_radius={self.bubble_radius:.3f} m | '
            f'gap_selection={self.gap_selection}'
        )

    # ------------------------------------------------------------------
    # Parameter management
    # ------------------------------------------------------------------

    def _load_params(self) -> None:
        """Read all parameters from the ROS 2 parameter server into attributes."""
        self.max_speed: float = self.get_parameter('ftg.max_speed').value
        self.min_speed: float = self.get_parameter('ftg.min_speed').value
        self.bubble_radius: float = self.get_parameter('ftg.bubble_radius').value
        self.max_lidar_range: float = self.get_parameter('ftg.max_lidar_range').value
        self.gap_selection: str = self.get_parameter('ftg.gap_selection').value
        self.steering_gain: float = self.get_parameter('ftg.steering_gain').value
        self.max_steering_angle: float = self.get_parameter('vehicle.max_steering_angle').value

    def _param_callback(self, params: List[Parameter]) -> SetParametersResult:
        """
        Called by ROS 2 whenever ros2 param set updates any parameter.
        Immediately re-loads all FTG parameters so the next scan callback
        uses the new values — no node restart required.
        """
        self._load_params()
        self.get_logger().info('FTG parameters updated live.')
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Core algorithm
    # ------------------------------------------------------------------

    def _scan_callback(self, msg: LaserScan) -> None:
        """
        Main callback: receives LaserScan, runs FTG, publishes AckermannDrive.

        Parameters
        ----------
        msg : LaserScan
            Incoming LiDAR scan from Hokuyo UST-10LX.
        """
        ranges = np.array(msg.ranges, dtype=np.float32)
        angle_min: float = msg.angle_min
        angle_increment: float = msg.angle_increment

        # --- Step 1: Preprocess scan ---
        # Replace NaN / Inf / out-of-range values with max_lidar_range so the
        # gap-finding algorithm treats them as free space.
        ranges = np.where(
            np.isfinite(ranges) & (ranges > 0.0),
            ranges,
            self.max_lidar_range
        )
        # Hard-clip everything to [0, max_lidar_range]
        ranges = np.clip(ranges, 0.0, self.max_lidar_range)

        # --- Step 2: Safety bubble ---
        # Find the closest obstacle to the car
        closest_idx: int = int(np.argmin(ranges))
        closest_dist: float = float(ranges[closest_idx])

        # Compute the angular half-width of the bubble in array indices
        if closest_dist > 1e-4:
            # arc = arcsin(bubble_radius / distance) gives half-angle in rad
            half_angle_rad: float = math.asin(
                min(self.bubble_radius / closest_dist, 1.0)
            )
            # Convert angle to number of scan indices
            bubble_half_width: int = max(1, int(half_angle_rad / angle_increment))
        else:
            # Car is essentially touching obstacle — zero everything
            bubble_half_width = len(ranges) // 2

        # Zero out the bubble around the closest point
        bubble_start: int = max(0, closest_idx - bubble_half_width)
        bubble_end: int = min(len(ranges) - 1, closest_idx + bubble_half_width)
        ranges[bubble_start:bubble_end + 1] = 0.0

        # --- Step 3: Gap finding ---
        # A "gap" is a contiguous sequence of non-zero range values.
        start_idx, end_idx = self._find_best_gap(ranges)

        # --- Step 4: Best point selection ---
        # Within the selected gap, pick the index with the highest range
        # (points furthest away from obstacles are safest to drive towards).
        gap_ranges = ranges[start_idx:end_idx + 1]
        best_idx_in_gap: int = int(np.argmax(gap_ranges))
        best_idx: int = start_idx + best_idx_in_gap

        # --- Step 5: Steering angle computation ---
        # Convert array index to physical angle relative to car forward direction
        best_angle_rad: float = angle_min + (best_idx * angle_increment)
        # Apply steering gain and clamp to vehicle limits
        steer: float = float(
            np.clip(
                self.steering_gain * best_angle_rad,
                -self.max_steering_angle,
                self.max_steering_angle
            )
        )

        # --- Step 6: Speed scaling ---
        # Reduce speed proportionally to steering magnitude.
        # At zero steer → max_speed; at pi/4 steer → min_speed.
        steer_fraction: float = abs(steer) / (math.pi / 4.0)
        speed_fraction: float = max(self.min_speed / self.max_speed,
                                    1.0 - steer_fraction)
        speed: float = self.max_speed * speed_fraction

        # --- Publish drive command ---
        self._publish_drive(speed, steer)

    def _find_best_gap(self, ranges: np.ndarray) -> Tuple[int, int]:
        """
        Find the best gap in the range array based on the gap_selection strategy.

        A gap is a contiguous sequence of non-zero (unobstructed) range values.

        Parameters
        ----------
        ranges : np.ndarray
            1D array of LiDAR ranges (zeros = obstructed by bubble).

        Returns
        -------
        Tuple[int, int]
            (start_index, end_index) of the selected gap (inclusive).
        """
        # Build a boolean mask: True where ranges > 0 (free space)
        free_mask: np.ndarray = ranges > 0.0

        # Find contiguous groups of True values using diff
        # Pad with False on both ends to detect edges
        padded = np.concatenate([[False], free_mask, [False]])
        diff = np.diff(padded.astype(int))
        starts: np.ndarray = np.where(diff == 1)[0]   # Rising edges → gap starts
        ends: np.ndarray = np.where(diff == -1)[0] - 1  # Falling edges → gap ends

        if len(starts) == 0:
            # Fallback: if all ranges zeroed, use full array center
            n = len(ranges)
            return n // 4, 3 * n // 4

        if self.gap_selection == 'largest':
            # Choose the longest gap (most unobstructed region)
            lengths = ends - starts + 1
            best = int(np.argmax(lengths))
        else:
            # 'furthest': choose the gap with the highest average range
            averages = [float(np.mean(ranges[s:e + 1])) for s, e in zip(starts, ends)]
            best = int(np.argmax(averages))

        return int(starts[best]), int(ends[best])

    def _publish_drive(self, speed: float, steering_angle: float) -> None:
        """
        Publish an AckermannDriveStamped message to the /drive topic.

        Parameters
        ----------
        speed : float
            Desired speed in m/s.
        steering_angle : float
            Desired steering angle in radians (positive = left).
        """
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = speed
        msg.drive.steering_angle = steering_angle
        self._drive_pub.publish(msg)


def main(args=None) -> None:
    """Entry point for the follow_the_gap ROS 2 node."""
    rclpy.init(args=args)
    node = FollowTheGapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('FollowTheGap node shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
