"""
pure_pursuit.py
===============
Pure Pursuit controller for F1TENTH Time Trial racing.

Purpose:
    Follow the pre-computed optimal racing line as cleanly and fast as
    possible by implementing the classic Pure Pursuit geometric tracking
    algorithm. This is the gold standard for time trials because it is
    deterministic, computationally trivial (runs at 100+ Hz on an Intel i7),
    and produces extremely smooth, jitter-free control commands.

Algorithm:
    1. Find the current car position from odometry.
    2. Search the racing line for the nearest waypoint.
    3. Look ahead by `lookahead_distance` metres along the racing line to
       find the target point (the "carrot").
    4. Calculate the steering angle required to drive an arc from the
       current position to the target point using the bicycle model:
           curvature κ = 2 * lateral_error / lookahead_distance²
           steering δ  = arctan(wheelbase * κ)
    5. Set the target speed from the racing line's pre-computed speed
       profile at the nearest waypoint.
    6. Apply a lookahead gain so the lookahead distance scales with speed
       (faster = look further ahead for stability).

All parameters are live-tunable via:
    ros2 param set /pure_pursuit pure_pursuit.<param_name> <value>

Note:
    Pure Pursuit publishes to /drive_cmd (not /drive directly).
    The deadman_switch node gates /drive_cmd → /drive based on the
    joystick deadman button state.
"""

from __future__ import annotations

import math
from typing import List, Optional

import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path


class PurePursuitController(Node):
    """
    ROS 2 node implementing the Pure Pursuit geometric path tracker.

    Subscribes:
        /ego_racecar/odom  (nav_msgs/Odometry)   — current car state
        /racing_line       (nav_msgs/Path)        — reference racing line

    Publishes:
        /drive_cmd         (ackermann_msgs/AckermannDriveStamped)

    All parameters live-tunable via ros2 param set.
    """

    def __init__(self) -> None:
        """Initialise Pure Pursuit controller with parameters and ROS interfaces."""
        super().__init__('pure_pursuit')

        # ------------------------------------------------------------------
        # Declare all tunable parameters
        # ------------------------------------------------------------------
        # Lookahead distance: how far ahead on the path to aim for
        self.declare_parameter('pure_pursuit.lookahead_distance', 1.0)  # [m]

        # Lookahead gain: scales lookahead with speed (adaptive lookahead)
        # lookahead = lookahead_distance + lookahead_gain * current_speed
        self.declare_parameter('pure_pursuit.lookahead_gain', 0.2)

        # Speed scaling: multiply racing line target speed by this factor
        # Set < 1.0 for conservative laps; 1.0 for full speed
        self.declare_parameter('pure_pursuit.speed_scale', 1.0)

        # Minimum and maximum speed clamps
        self.declare_parameter('pure_pursuit.min_speed', 0.5)  # [m/s]
        self.declare_parameter('pure_pursuit.max_speed', 8.0)  # [m/s]

        # Vehicle parameters
        self.declare_parameter('vehicle.wheelbase', 0.324)
        self.declare_parameter('vehicle.max_steering_angle', 0.43)

        # Control loop frequency [Hz]
        self.declare_parameter('pure_pursuit.control_frequency_hz', 40.0)

        self._load_params()
        self.add_on_set_parameters_callback(self._param_callback)

        # ------------------------------------------------------------------
        # Internal state
        # ------------------------------------------------------------------
        self._state: Optional[np.ndarray] = None       # [x, y, yaw, speed]
        self._racing_line: Optional[np.ndarray] = None  # (N, 4) [x,y,heading,speed]

        # ------------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------------
        self.create_subscription(
            Odometry, '/ego_racecar/odom', self._odom_callback, 10)
        self.create_subscription(
            Path, '/racing_line', self._racing_line_callback, 10)

        # ------------------------------------------------------------------
        # Publisher — to /drive_cmd (gated by deadman_switch)
        # ------------------------------------------------------------------
        self._drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive_cmd', 10)

        # ------------------------------------------------------------------
        # Control timer
        # ------------------------------------------------------------------
        period = 1.0 / max(self.control_frequency_hz, 1.0)
        self._timer = self.create_timer(period, self._control_loop)

        self.get_logger().info(
            f'PurePursuit started | '
            f'lookahead={self.lookahead_distance:.2f} m | '
            f'lookahead_gain={self.lookahead_gain:.2f} | '
            f'speed_scale={self.speed_scale:.2f} | '
            f'freq={self.control_frequency_hz:.0f} Hz'
        )

    # ------------------------------------------------------------------
    # Parameter management
    # ------------------------------------------------------------------

    def _load_params(self) -> None:
        """Read all parameters into instance variables."""
        self.lookahead_distance: float = self.get_parameter(
            'pure_pursuit.lookahead_distance').value
        self.lookahead_gain: float = self.get_parameter(
            'pure_pursuit.lookahead_gain').value
        self.speed_scale: float = self.get_parameter(
            'pure_pursuit.speed_scale').value
        self.min_speed: float = self.get_parameter('pure_pursuit.min_speed').value
        self.max_speed: float = self.get_parameter('pure_pursuit.max_speed').value
        self.wheelbase: float = self.get_parameter('vehicle.wheelbase').value
        self.max_steering_angle: float = self.get_parameter(
            'vehicle.max_steering_angle').value
        self.control_frequency_hz: float = self.get_parameter(
            'pure_pursuit.control_frequency_hz').value

    def _param_callback(self, params: List[Parameter]) -> SetParametersResult:
        """Live parameter update callback."""
        self._load_params()
        self.get_logger().info('PurePursuit parameters updated live.')
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------

    def _odom_callback(self, msg: Odometry) -> None:
        """
        Extract car state [x, y, yaw, speed] from odometry.
        Yaw is extracted from the pose quaternion (z-rotation only).
        """
        x: float = msg.pose.pose.position.x
        y: float = msg.pose.pose.position.y
        qz: float = msg.pose.pose.orientation.z
        qw: float = msg.pose.pose.orientation.w
        yaw: float = 2.0 * math.atan2(qz, qw)
        speed: float = msg.twist.twist.linear.x
        self._state = np.array([x, y, yaw, speed], dtype=np.float64)

    def _racing_line_callback(self, msg: Path) -> None:
        """
        Store racing line as NumPy array (N, 4): [x, y, heading, speed].
        Speed is extracted from the racing line CSV via the path message
        (encoded in the z-position field as a convention, or defaulted to
        max_speed if not available).
        """
        if len(msg.poses) < 2:
            return

        pts = []
        for i, pose in enumerate(msg.poses):
            x = pose.pose.position.x
            y = pose.pose.position.y
            # Decode heading from quaternion
            qz = pose.pose.orientation.z
            qw = pose.pose.orientation.w
            heading = 2.0 * math.atan2(qz, qw)
            # Target speed encoded in z position field (convention with our generator)
            # Falls back to max_speed if z is 0 (standard Path msg)
            target_speed = (pose.pose.position.z
                            if abs(pose.pose.position.z) > 0.01
                            else self.max_speed)
            pts.append([x, y, heading, target_speed])

        self._racing_line = np.array(pts, dtype=np.float64)
        self.get_logger().info(
            f'Pure Pursuit received racing line: {len(self._racing_line)} pts',
            once=True
        )

    # ------------------------------------------------------------------
    # Core Pure Pursuit control loop
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        """
        Main control callback — runs at control_frequency_hz.

        Implements the full Pure Pursuit algorithm:
            1. Find nearest racing line waypoint to current position.
            2. Compute adaptive lookahead distance based on current speed.
            3. Find the lookahead target point on the racing line.
            4. Compute steering angle via the Pure Pursuit arc formula.
            5. Read target speed from the racing line waypoint.
            6. Publish AckermannDriveStamped to /drive_cmd.
        """
        if self._state is None:
            self.get_logger().warn(
                'Waiting for odometry...', throttle_duration_sec=2.0)
            return
        if self._racing_line is None:
            self.get_logger().warn(
                'Waiting for racing line...', throttle_duration_sec=2.0)
            return

        car_x, car_y, car_yaw = (
            self._state[0], self._state[1], self._state[2])
        current_speed: float = abs(float(self._state[3]))

        rl_x = self._racing_line[:, 0]
        rl_y = self._racing_line[:, 1]
        rl_speeds = self._racing_line[:, 3]

        # --- Step 1: Find nearest waypoint ---
        dists = np.hypot(rl_x - car_x, rl_y - car_y)
        nearest_idx: int = int(np.argmin(dists))

        # --- Step 2: Adaptive lookahead distance ---
        # Faster speed → look further ahead for stability
        adaptive_lookahead: float = (
            self.lookahead_distance + self.lookahead_gain * current_speed
        )

        # --- Step 3: Find lookahead target point ---
        # Walk forward along the racing line from nearest_idx until we
        # find the first point further than adaptive_lookahead from the car
        target_idx: int = nearest_idx
        n_pts: int = len(self._racing_line)

        for i in range(1, n_pts):
            idx = (nearest_idx + i) % n_pts
            d = math.hypot(rl_x[idx] - car_x, rl_y[idx] - car_y)
            if d >= adaptive_lookahead:
                target_idx = idx
                break

        target_x: float = float(rl_x[target_idx])
        target_y: float = float(rl_y[target_idx])

        # --- Step 4: Pure Pursuit steering calculation ---
        # Transform target point into the car's local frame (rotate by -yaw)
        dx: float = target_x - car_x
        dy: float = target_y - car_y

        # Lateral error in the car's local frame
        # This is the perpendicular distance from the car's heading to the target
        local_y: float = -math.sin(car_yaw) * dx + math.cos(car_yaw) * dy

        # Actual distance to target (the "ell" in Pure Pursuit literature)
        ell: float = math.hypot(dx, dy)

        if ell < 1e-4:
            # Target is right under the car — go straight
            steering_angle: float = 0.0
        else:
            # Pure Pursuit curvature formula: κ = 2 * y_L / ell²
            # Steering angle: δ = arctan(wheelbase * κ)
            curvature: float = 2.0 * local_y / (ell ** 2)
            steering_angle = math.atan(self.wheelbase * curvature)

        # Hard clamp to vehicle limits
        steering_angle = max(-self.max_steering_angle,
                             min(self.max_steering_angle, steering_angle))

        # --- Step 5: Target speed from racing line ---
        target_speed: float = float(rl_speeds[nearest_idx]) * self.speed_scale
        target_speed = max(self.min_speed, min(self.max_speed, target_speed))

        # --- Step 6: Publish command ---
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = target_speed
        msg.drive.steering_angle = steering_angle
        self._drive_pub.publish(msg)


def main(args=None) -> None:
    """Entry point for the pure_pursuit ROS 2 node."""
    rclpy.init(args=args)
    node = PurePursuitController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('PurePursuit shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
