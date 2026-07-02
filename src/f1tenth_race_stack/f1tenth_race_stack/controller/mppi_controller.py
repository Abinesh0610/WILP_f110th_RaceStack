"""
mppi_controller.py
==================
Model Predictive Path Integral (MPPI) racing controller for F1TENTH.

Purpose:
    Generate high-speed, optimal racing control commands by sampling K=1000
    parallel trajectory rollouts using a kinematic bicycle model, evaluating
    each trajectory's cost (track deviation, speed reward, obstacle proximity),
    and computing a weighted average of the best control sequences.

CRITICAL CONSTRAINT:
    The Intel NUC 12 Pro has NO GPU. All K rollouts are parallelised using
    NumPy vectorised matrix operations. No Python loops over rollouts are used.

Algorithm Overview:
    1. Perturb the current warm-started control sequence U with Gaussian noise
       to produce K candidate sequences.
    2. Simulate each candidate forward T steps using the kinematic bicycle model.
    3. Accumulate trajectory costs at each step (track deviation, speed reward,
       obstacle penalty, heading error, control smoothness).
    4. Compute softmax weights: w_k ∝ exp(−cost_k / λ).
    5. Update the control sequence as the weighted average of candidates.
    6. Publish the first action, then shift the sequence forward (receding horizon).

All parameters are live-tunable via:
    ros2 param set /mppi_controller mppi.<param_name> <value>
or via rqt_reconfigure (launched automatically by racing.launch.py).

Performance Target: < 50 ms per control cycle on Intel i7 (20 Hz loop).
"""

from __future__ import annotations

import time
from typing import List, Optional

import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan


class MPPIController(Node):
    """
    ROS 2 node implementing MPPI racing control.

    Subscribes:
        /scan             (sensor_msgs/LaserScan)     — LiDAR for obstacle avoidance
        /ego_racecar/odom (nav_msgs/Odometry)          — Current car state
        /racing_line      (nav_msgs/Path)              — Reference racing line

    Publishes:
        /drive            (ackermann_msgs/AckermannDriveStamped)

    Timer:
        Runs MPPI loop at 20 Hz (every 50 ms).
    """

    def __init__(self) -> None:
        """Initialise the MPPI controller node with all parameters and state."""
        super().__init__('mppi_controller')

        # ------------------------------------------------------------------
        # Declare all MPPI parameters (sourced from mppi_params.yaml)
        # All are live-tunable.
        # ------------------------------------------------------------------
        # Rollout configuration
        self.declare_parameter('mppi.K', 1000)
        self.declare_parameter('mppi.T', 20)
        self.declare_parameter('mppi.dt', 0.05)

        # Temperature
        self.declare_parameter('mppi.lambda', 0.1)

        # Noise standard deviations
        self.declare_parameter('mppi.noise_std_speed', 1.0)
        self.declare_parameter('mppi.noise_std_steering', 0.15)

        # Cost weights
        self.declare_parameter('mppi.weight_reference_track', 10.0)
        self.declare_parameter('mppi.weight_speed_reward', 1.0)
        self.declare_parameter('mppi.weight_obstacle_penalty', 500.0)
        self.declare_parameter('mppi.weight_heading', 2.0)
        self.declare_parameter('mppi.weight_boundary', 200.0)
        self.declare_parameter('mppi.weight_control_smoothness', 0.1)

        # Safety
        self.declare_parameter('mppi.obstacle_clearance_m', 0.35)

        # Vehicle kinematics
        self.declare_parameter('vehicle.wheelbase', 0.324)
        self.declare_parameter('vehicle.max_steering_angle', 0.43)
        self.declare_parameter('vehicle.max_speed', 10.0)
        self.declare_parameter('vehicle.min_speed', 0.5)

        # ------------------------------------------------------------------
        # Load parameter values into instance variables
        # ------------------------------------------------------------------
        self._load_params()

        # ------------------------------------------------------------------
        # Live parameter callback
        # ------------------------------------------------------------------
        self.add_on_set_parameters_callback(self._param_callback)

        # ------------------------------------------------------------------
        # Internal state
        # ------------------------------------------------------------------
        # Warm-started control sequence: shape (T, 2) → [speed, steering] per step
        self.u_sequence: np.ndarray = np.zeros((self.T, 2), dtype=np.float64)
        # Initialise speed column to min_speed to avoid zero-velocity cold start
        self.u_sequence[:, 0] = self.min_speed

        # Counter for periodic warm-start resets (prevents multi-lap drift)
        self._loop_counter: int = 0

        # Latest car state: [x, y, yaw, speed]
        self._state: Optional[np.ndarray] = None

        # Latest LiDAR obstacle points in local frame: shape (N, 2)
        self._obstacle_points: Optional[np.ndarray] = None

        # Reference racing line as NumPy array: shape (M, 4) → [x, y, heading, speed]
        self._racing_line: Optional[np.ndarray] = None

        # ------------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------------
        self._scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_callback, 10)

        self._odom_sub = self.create_subscription(
            Odometry, '/ego_racecar/odom', self._odom_callback, 10)

        self._rl_sub = self.create_subscription(
            Path, '/racing_line', self._racing_line_callback, 10)

        # ------------------------------------------------------------------
        # Publisher — to /drive_cmd (gated by deadman_switch → /drive)
        # ------------------------------------------------------------------
        self._drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive_cmd', 10)

        # ------------------------------------------------------------------
        # Main MPPI timer — 20 Hz
        # ------------------------------------------------------------------
        self._timer = self.create_timer(0.05, self._mppi_loop)

        self.get_logger().info(
            f'MPPIController started | K={self.K} rollouts | T={self.T} steps | '
            f'dt={self.dt}s | λ={self.lambda_:.3f} | '
            f'w_track={self.weight_reference_track}'
        )

    # ------------------------------------------------------------------
    # Parameter management
    # ------------------------------------------------------------------

    def _load_params(self) -> None:
        """Read all parameters from ROS 2 parameter server into instance variables."""
        self.K: int = int(self.get_parameter('mppi.K').value)
        self.T: int = int(self.get_parameter('mppi.T').value)
        self.dt: float = self.get_parameter('mppi.dt').value
        self.lambda_: float = self.get_parameter('mppi.lambda').value
        self.noise_std_speed: float = self.get_parameter('mppi.noise_std_speed').value
        self.noise_std_steering: float = self.get_parameter('mppi.noise_std_steering').value
        self.weight_reference_track: float = self.get_parameter('mppi.weight_reference_track').value
        self.weight_speed_reward: float = self.get_parameter('mppi.weight_speed_reward').value
        self.weight_obstacle_penalty: float = self.get_parameter('mppi.weight_obstacle_penalty').value
        self.weight_heading: float = self.get_parameter('mppi.weight_heading').value
        self.weight_boundary: float = self.get_parameter('mppi.weight_boundary').value
        self.weight_control_smoothness: float = self.get_parameter('mppi.weight_control_smoothness').value
        self.obstacle_clearance_m: float = self.get_parameter('mppi.obstacle_clearance_m').value
        self.wheelbase: float = self.get_parameter('vehicle.wheelbase').value
        self.max_steering_angle: float = self.get_parameter('vehicle.max_steering_angle').value
        self.max_speed: float = self.get_parameter('vehicle.max_speed').value
        self.min_speed: float = self.get_parameter('vehicle.min_speed').value

    def _param_callback(self, params: List[Parameter]) -> SetParametersResult:
        """
        Live parameter update callback. Called whenever any parameter changes.
        Extracts values directly from the incoming parameter list since get_parameter()
        will still return the old values at this stage of the callback.
        """
        old_K, old_T = self.K, self.T
        
        # Map ROS parameter names to internal instance variable names
        param_map = {
            'mppi.K': ('K', int),
            'mppi.T': ('T', int),
            'mppi.dt': ('dt', float),
            'mppi.lambda': ('lambda_', float),
            'mppi.noise_std_speed': ('noise_std_speed', float),
            'mppi.noise_std_steering': ('noise_std_steering', float),
            'mppi.weight_reference_track': ('weight_reference_track', float),
            'mppi.weight_speed_reward': ('weight_speed_reward', float),
            'mppi.weight_obstacle_penalty': ('weight_obstacle_penalty', float),
            'mppi.weight_heading': ('weight_heading', float),
            'mppi.weight_boundary': ('weight_boundary', float),
            'mppi.weight_control_smoothness': ('weight_control_smoothness', float),
            'mppi.obstacle_clearance_m': ('obstacle_clearance_m', float),
            'vehicle.wheelbase': ('wheelbase', float),
            'vehicle.max_steering_angle': ('max_steering_angle', float),
            'vehicle.max_speed': ('max_speed', float),
            'vehicle.min_speed': ('min_speed', float),
        }

        for param in params:
            # ── CRITICAL: Block use_sim_time changes from rqt_reconfigure ──
            if param.name == 'use_sim_time':
                self.get_logger().warn(
                    'Rejected change to use_sim_time via rqt_reconfigure. '
                    'Do NOT click this checkbox while the car is running!')
                return SetParametersResult(successful=False)

            if param.name in param_map:
                attr_name, cast_func = param_map[param.name]
                setattr(self, attr_name, cast_func(param.value))
                self.get_logger().info(f"{param.name} → {getattr(self, attr_name)}")

        # If rollout dimensions changed, reset the warm-start sequence
        if self.K != old_K or self.T != old_T:
            self.u_sequence = np.zeros((self.T, 2), dtype=np.float64)
            self.u_sequence[:, 0] = self.min_speed
            self.get_logger().info(
                f'MPPI sequence resized to K={self.K}, T={self.T}')

        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------

    def _odom_callback(self, msg: Odometry) -> None:
        """
        Extract current car state [x, y, yaw, speed] from odometry.

        Yaw is extracted from the quaternion orientation using the
        atan2 formula for the z-rotation of a planar vehicle.
        """
        x: float = msg.pose.pose.position.x
        y: float = msg.pose.pose.position.y

        # Quaternion → yaw (z-rotation only, since vehicle is planar)
        qz: float = msg.pose.pose.orientation.z
        qw: float = msg.pose.pose.orientation.w
        yaw: float = 2.0 * np.arctan2(qz, qw)

        speed: float = msg.twist.twist.linear.x
        self._state = np.array([x, y, yaw, speed], dtype=np.float64)

    def _scan_callback(self, msg: LaserScan) -> None:
        """
        Convert 1D LaserScan into a set of 2D obstacle points in the car's
        local (base_link) frame for use in the obstacle cost function.

        The car's local frame has X forward and Y to the left.
        A scan ray at index i subtends angle: θ_i = angle_min + i * angle_increment
        The 2D point in local frame: (r·cos(θ), r·sin(θ))
        """
        ranges = np.array(msg.ranges, dtype=np.float32)
        angles = (msg.angle_min
                  + np.arange(len(ranges), dtype=np.float32) * msg.angle_increment)

        # Keep only valid (finite, positive, within max range) readings
        valid_mask = (
            np.isfinite(ranges)
            & (ranges > 0.05)
            & (ranges < msg.range_max)
        )
        
        # Downsample the scan by taking every 5th point to speed up MPPI calculations
        # (e.g., 1080 points -> ~216 points). This drastically reduces the N*K matrix size.
        valid_ranges = ranges[valid_mask][::5]
        valid_angles = angles[valid_mask][::5]

        # Project polar → Cartesian in local frame
        obs_x = valid_ranges * np.cos(valid_angles)  # Forward
        obs_y = valid_ranges * np.sin(valid_angles)  # Left

        self._obstacle_points = np.column_stack([obs_x, obs_y])


    def _racing_line_callback(self, msg: Path) -> None:
        """
        Store the reference racing line as a NumPy array for cost computation.

        Extracts (x, y, heading) from the Path poses.
        heading is computed from consecutive pose positions.
        """
        poses: list = msg.poses
        if len(poses) < 2:
            return

        points = np.array(
            [[p.pose.position.x, p.pose.position.y] for p in poses],
            dtype=np.float64
        )

        # Compute heading from finite differences between consecutive points
        diffs = np.diff(points, axis=0)
        headings = np.arctan2(diffs[:, 1], diffs[:, 0])
        # Repeat last heading for the final point
        headings = np.append(headings, headings[-1])

        self._racing_line = np.column_stack([points, headings])  # (M, 3)
        self.get_logger().info(
            f'Racing line received: {len(self._racing_line)} waypoints.',
            once=True
        )

    # ------------------------------------------------------------------
    # MPPI core
    # ------------------------------------------------------------------

    def _mppi_loop(self) -> None:
        """
        Main MPPI control loop — called at 20 Hz by the ROS 2 timer.

        Steps:
            1. Guard: ensure all required data is available.
            2. Sample K perturbed control sequences.
            3. Roll out the kinematic bicycle model for all K sequences.
            4. Compute per-rollout costs.
            5. Compute softmax importance weights.
            6. Update the nominal control sequence.
            7. Publish the first action.
            8. Shift the sequence forward (receding horizon warm-start).
        """
        # --- Guard: data availability ---
        if self._state is None:
            self.get_logger().warn('Waiting for odometry...', throttle_duration_sec=2.0)
            return
        if self._racing_line is None:
            self.get_logger().warn('Waiting for racing line...', throttle_duration_sec=2.0)
            return

        # --- Timing: measure computation time for performance monitoring ---
        t_start: float = time.perf_counter()

        # --- Step 1: Sample K perturbed control sequences ---
        # Generate constant noise across the prediction horizon for each rollout.
        # This solves the "white noise cancellation" problem, allowing MPPI to
        # explore deep, sustained turns instead of just jittering around zero.
        noise = np.random.randn(self.K, 1, 2)
        noise = np.repeat(noise, self.T, axis=1)  # (K, T, 2)
        noise[:, :, 0] *= self.noise_std_speed     # Speed perturbation
        noise[:, :, 1] *= self.noise_std_steering  # Steering perturbation

        # Expand nominal sequence to (K, T, 2) and add noise
        U: np.ndarray = self.u_sequence[np.newaxis, :, :] + noise  # (K, T, 2)

        # Clamp to vehicle limits
        U[:, :, 0] = np.clip(U[:, :, 0], self.min_speed, self.max_speed)
        U[:, :, 1] = np.clip(U[:, :, 1], -self.max_steering_angle, self.max_steering_angle)

        # --- Step 2 & 3: Rollout + cost accumulation ---
        costs: np.ndarray = self._rollout(self._state[:3], U)

        # --- Step 4: Add control smoothness cost ---
        # Guard: only compute if T > 1, otherwise no diffs exist
        if self.T > 1:
            u_diffs = np.diff(U, axis=1)  # (K, T-1, 2)
            # Sum speed² + steering² over T-1 steps → shape (K,)
            smoothness_cost = self.weight_control_smoothness * np.sum(
                u_diffs[:, :, 0] ** 2 + u_diffs[:, :, 1] ** 2, axis=1
            )
            costs += smoothness_cost

        # --- Step 5: Softmax importance weights ---
        # Shift by minimum cost for numerical stability (standard MPPI trick)
        beta: float = float(np.min(costs))
        weights: np.ndarray = np.exp(-1.0 / self.lambda_ * (costs - beta))
        weights /= (np.sum(weights) + 1e-9)  # Normalise; avoid division by zero

        # --- Step 6: Update nominal control sequence ---
        # Weighted average of all K candidate sequences: (T, 2)
        self.u_sequence = np.sum(weights[:, np.newaxis, np.newaxis] * U, axis=0)

        # --- Step 7: Publish the first action in the updated sequence ---
        speed: float = float(self.u_sequence[0, 0])
        steer: float = float(self.u_sequence[0, 1])
        self._publish_drive(speed, steer)

        # --- Step 8: Receding horizon warm-start ---
        # Shift the sequence forward by 1 step and replicate the last control
        self.u_sequence = np.roll(self.u_sequence, shift=-1, axis=0)
        self.u_sequence[-1] = self.u_sequence[-2]  # Replicate last action

        # --- Step 9: Clamp warm-start to prevent drift ---
        # After the weighted average update, the sequence can accumulate a
        # systematic steering bias over many laps. Hard-clamp it every step.
        self.u_sequence[:, 0] = np.clip(self.u_sequence[:, 0], self.min_speed, self.max_speed)
        self.u_sequence[:, 1] = np.clip(self.u_sequence[:, 1], -self.max_steering_angle, self.max_steering_angle)

        # --- Step 10: Periodic hard reset every 200 loops (~10 seconds) ---
        # Prevents any residual drift from accumulating across multiple laps.
        self._loop_counter += 1
        if self._loop_counter % 200 == 0:
            self.u_sequence[:, 0] = self.min_speed  # Reset speed to minimum
            self.u_sequence[:, 1] = 0.0              # Reset steering to straight
            self.get_logger().info('Warm-start sequence reset to prevent drift.')

        # --- Timing report ---
        elapsed_ms: float = (time.perf_counter() - t_start) * 1000.0
        if elapsed_ms > 50.0:
            self.get_logger().warn(
                f'MPPI loop took {elapsed_ms:.1f} ms — exceeds 50 ms budget! '
                f'Consider reducing K={self.K}.'
            )
        else:
            self.get_logger().debug(f'MPPI loop: {elapsed_ms:.1f} ms')

    def _rollout(self, state: np.ndarray, u_seq: np.ndarray) -> np.ndarray:
        """
        Simulate K trajectories simultaneously using the kinematic bicycle model.

        The bicycle model (rear-axle referenced) integrates:
            x   += v * cos(yaw) * dt
            y   += v * sin(yaw) * dt
            yaw += (v / L) * tan(δ) * dt

        where L = wheelbase, v = speed, δ = steering angle.

        Parameters
        ----------
        state : np.ndarray, shape (3,)
            Current car state [x, y, yaw].
        u_seq : np.ndarray, shape (K, T, 2)
            K candidate control sequences, each of T steps, each step [speed, steering].

        Returns
        -------
        np.ndarray, shape (K,)
            Total accumulated cost for each of the K rollouts.
        """
        K = self.K

        # Initialise all K rollouts from the same current state
        x: np.ndarray = np.full(K, state[0], dtype=np.float64)
        y: np.ndarray = np.full(K, state[1], dtype=np.float64)
        yaw: np.ndarray = np.full(K, state[2], dtype=np.float64)

        trajectory_costs: np.ndarray = np.zeros(K, dtype=np.float64)

        # Pre-extract racing line columns for vectorised ops
        rl_x = self._racing_line[:, 0]       # (M,)
        rl_y = self._racing_line[:, 1]       # (M,)
        rl_heading = self._racing_line[:, 2]  # (M,)

        for t in range(self.T):
            # --- Extract controls for this step ---
            v: np.ndarray = u_seq[:, t, 0]   # (K,) speed commands
            delta: np.ndarray = u_seq[:, t, 1]  # (K,) steering commands

            # --- Integrate bicycle model ---
            x += v * np.cos(yaw) * self.dt
            y += v * np.sin(yaw) * self.dt
            yaw += (v / self.wheelbase) * np.tan(delta) * self.dt

            # --- Cost 1: Distance to nearest racing line point ---
            # Broadcast: (M, 1) - (K,) → (M, K) → min over M → (K,)
            dx = rl_x[:, np.newaxis] - x[np.newaxis, :]  # (M, K)
            dy = rl_y[:, np.newaxis] - y[np.newaxis, :]  # (M, K)
            dists_sq = dx ** 2 + dy ** 2                  # (M, K)
            nearest_idx = np.argmin(dists_sq, axis=0)     # (K,)
            nearest_dist = np.sqrt(dists_sq[nearest_idx, np.arange(K)])  # (K,)
            trajectory_costs += self.weight_reference_track * nearest_dist

            # --- Cost 2: Heading alignment ---
            # Penalise angle between car heading and racing line tangent heading
            target_heading = rl_heading[nearest_idx]       # (K,)
            heading_error = np.abs(
                np.arctan2(np.sin(yaw - target_heading), np.cos(yaw - target_heading))
            )
            trajectory_costs += self.weight_heading * heading_error

            # --- Cost 3: Speed reward (negative cost = reward) ---
            trajectory_costs -= self.weight_speed_reward * v

            # --- Cost 4: Obstacle penalty from LiDAR ---
            trajectory_costs += self._obstacle_cost(x, y)

            # --- Cost 5: Boundary / wall penalty ---
            # If a rollout point is more than boundary_threshold metres from
            # the nearest racing line point, it has likely left the track.
            boundary_threshold = 1.2  # [m] — half the Levine corridor width
            off_track = nearest_dist > boundary_threshold
            trajectory_costs += self.weight_boundary * off_track.astype(np.float64)

        return trajectory_costs

    def _obstacle_cost(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """
        Compute obstacle penalty for each rollout position using LiDAR data.

        For each rollout (x_k, y_k), compute the minimum Euclidean distance
        to any LiDAR-detected obstacle point. Apply an inverse-distance penalty
        that grows to infinity as distance → obstacle_clearance_m.

        Uses NumPy broadcasting to avoid any Python loop over rollouts.

        Parameters
        ----------
        x : np.ndarray, shape (K,)
            X positions of all K rollouts at a given timestep (in world frame).
        y : np.ndarray, shape (K,)
            Y positions of all K rollouts at a given timestep.

        Returns
        -------
        np.ndarray, shape (K,)
            Per-rollout obstacle cost (0.0 if no obstacle data available).
        """
        if self._obstacle_points is None or len(self._obstacle_points) == 0:
            return np.zeros_like(x)

        # Transform obstacle points from local car frame to world frame
        # using the current car pose (self._state = [x, y, yaw, speed])
        if self._state is None:
            return np.zeros_like(x)

        car_x, car_y, car_yaw = self._state[0], self._state[1], self._state[2]
        cos_yaw = np.cos(car_yaw)
        sin_yaw = np.sin(car_yaw)

        # obs_world: (N, 2) in world frame
        obs_local = self._obstacle_points  # (N, 2)
        obs_world_x = car_x + cos_yaw * obs_local[:, 0] - sin_yaw * obs_local[:, 1]
        obs_world_y = car_y + sin_yaw * obs_local[:, 0] + cos_yaw * obs_local[:, 1]

        # Broadcasting: (N, 1) - (K,) → (N, K) distance matrix
        dx = obs_world_x[:, np.newaxis] - x[np.newaxis, :]  # (N, K)
        dy = obs_world_y[:, np.newaxis] - y[np.newaxis, :]  # (N, K)
        dist_matrix = np.sqrt(dx ** 2 + dy ** 2)            # (N, K)

        # Minimum distance to any obstacle for each rollout
        min_dist = np.min(dist_matrix, axis=0)               # (K,)

        # Penalty: high when closer than clearance, low when far
        # Inverse distance penalty clamped to a maximum
        clearance = self.obstacle_clearance_m
        # Points inside clearance radius get maximum penalty
        in_collision = min_dist < clearance
        safe_dist = np.where(in_collision, 1e-6, min_dist - clearance + 1e-6)
        penalty = self.weight_obstacle_penalty / safe_dist
        # Cap penalty to avoid numerical explosions
        penalty = np.clip(penalty, 0.0, self.weight_obstacle_penalty * 100.0)

        return penalty

    def _publish_drive(self, speed: float, steering_angle: float) -> None:
        """
        Publish an AckermannDriveStamped message to the /drive topic.

        Parameters
        ----------
        speed : float
            Desired speed in m/s.
        steering_angle : float
            Desired steering angle in radians.
        """
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = float(np.clip(speed, self.min_speed, self.max_speed))
        msg.drive.steering_angle = float(
            np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)
        )
        self._drive_pub.publish(msg)


def main(args=None) -> None:
    """Entry point for the mppi_controller ROS 2 node."""
    rclpy.init(args=args)
    node = MPPIController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('MPPIController shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
