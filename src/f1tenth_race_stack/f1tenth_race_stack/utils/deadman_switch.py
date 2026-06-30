"""
deadman_switch.py
=================
Safety deadman switch node for ALL operating modes of the F1TENTH race stack.

Purpose:
    A critical safety layer that sits between every autonomous planner /
    joystick and the VESC motor controller. While the deadman button on the
    RadioMaster MT12 joystick is HELD, commands flow through to the car.
    The moment the button is RELEASED (or the joystick connection drops),
    this node immediately broadcasts zero velocity at 50 Hz to bring the
    car to a complete stop.

Architecture:
    All autonomous planners (MPPI, Pure Pursuit, FTG) publish to:
        /drive_cmd  (AckermannDriveStamped)   — autonomous command input

    Manual joystick drive is constructed from /joy axes by this node.

    This node publishes the final gated output to:
        /drive      (AckermannDriveStamped)   — consumed by vesc_bridge

    Gate logic:
        deadman_button HELD + AUTONOMOUS mode → forward /drive_cmd → /drive
        deadman_button HELD + MANUAL mode     → build drive from joy axes → /drive
        deadman_button NOT HELD (any mode)    → publish zero at 50 Hz → /drive

    The operating mode is received from /race_stack/current_state so the
    node always knows whether to expect manual or autonomous commands.

RadioMaster MT12 Button / Axis Map (default — tune via parameters):
    Button 5 (RB / SF switch) = deadman button (hold to enable)
    Axis 1  (Left stick Y)    = manual speed
    Axis 3  (Right stick X)   = manual steering
"""

from __future__ import annotations

import time
from typing import List, Optional

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import String


# States that use autonomous planners (drive_cmd is forwarded)
AUTONOMOUS_STATES = {
    'MAPPING_AUTO',
    'RACING',
    'TIME_TRIAL',
    'HEAD_TO_HEAD',
}

# States that use manual joystick drive
MANUAL_STATES = {
    'MAPPING_MANUAL',
}

# States where the car should never move
STOPPED_STATES = {
    'IDLE',
    'EMERGENCY_STOP',
}


class DeadmanSwitch(Node):
    """
    ROS 2 safety node implementing a hardware deadman switch.

    The deadman button must be continuously held for any motion command
    (manual or autonomous) to be forwarded to the VESC.

    Subscribes:
        /joy                       (sensor_msgs/Joy)
        /drive_cmd                 (ackermann_msgs/AckermannDriveStamped)
        /race_stack/current_state  (std_msgs/String)

    Publishes:
        /drive                     (ackermann_msgs/AckermannDriveStamped)
    """

    def __init__(self) -> None:
        """Initialise the deadman switch with all parameters and state."""
        super().__init__('deadman_switch')

        # ------------------------------------------------------------------
        # Declare tunable parameters
        # ------------------------------------------------------------------
        # Joystick button index for deadman (RadioMaster MT12: button 5 = SF/RB)
        self.declare_parameter('deadman_button_idx', 5)

        # Manual drive axes (RadioMaster MT12 defaults)
        self.declare_parameter('manual_speed_axis', 1)      # Left stick vertical
        self.declare_parameter('manual_steer_axis', 3)      # Right stick horizontal

        # Vehicle limits for manual drive scaling
        self.declare_parameter('manual_max_speed', 3.0)     # [m/s] safe manual speed
        self.declare_parameter('vehicle.max_steering_angle', 0.43)  # [rad]

        # Timeout: if no joy message received in this many seconds → STOP
        self.declare_parameter('joy_timeout_s', 0.5)

        # Zero-command publish rate when deadman is released [Hz]
        self.declare_parameter('zero_publish_rate_hz', 50.0)

        self._load_params()
        self.add_on_set_parameters_callback(self._param_callback)

        # ------------------------------------------------------------------
        # Internal state
        # ------------------------------------------------------------------
        self._deadman_held: bool = False
        self._last_joy_time: float = time.monotonic()
        self._current_state: str = 'IDLE'
        self._latest_drive_cmd: Optional[AckermannDriveStamped] = None
        self._latest_joy: Optional[Joy] = None

        # ------------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------------
        self.create_subscription(Joy, '/joy', self._joy_callback, 10)
        self.create_subscription(
            AckermannDriveStamped, '/drive_cmd', self._drive_cmd_callback, 10)
        self.create_subscription(
            String, '/race_stack/current_state', self._state_callback, 10)

        # ------------------------------------------------------------------
        # Publisher
        # ------------------------------------------------------------------
        self._drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)

        # ------------------------------------------------------------------
        # Main gate timer — runs at zero_publish_rate_hz
        # ------------------------------------------------------------------
        period = 1.0 / max(self.zero_publish_rate_hz, 10.0)
        self._gate_timer = self.create_timer(period, self._gate_loop)

        self.get_logger().info(
            f'DeadmanSwitch ready | '
            f'button={self.deadman_button_idx} | '
            f'joy_timeout={self.joy_timeout_s}s | '
            f'manual_max_speed={self.manual_max_speed} m/s\n'
            f'>>> HOLD button {self.deadman_button_idx} on RadioMaster MT12 to enable motion.'
        )

    # ------------------------------------------------------------------
    # Parameter management
    # ------------------------------------------------------------------

    def _load_params(self) -> None:
        """Load all parameters into instance variables."""
        self.deadman_button_idx: int = int(
            self.get_parameter('deadman_button_idx').value)
        self.manual_speed_axis: int = int(
            self.get_parameter('manual_speed_axis').value)
        self.manual_steer_axis: int = int(
            self.get_parameter('manual_steer_axis').value)
        self.manual_max_speed: float = self.get_parameter('manual_max_speed').value
        self.max_steering_angle: float = self.get_parameter(
            'vehicle.max_steering_angle').value
        self.joy_timeout_s: float = self.get_parameter('joy_timeout_s').value
        self.zero_publish_rate_hz: float = self.get_parameter(
            'zero_publish_rate_hz').value

    def _param_callback(self, params: List[Parameter]) -> SetParametersResult:
        """Apply parameter updates immediately."""
        self._load_params()
        self.get_logger().info('DeadmanSwitch parameters updated.')
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------

    def _joy_callback(self, msg: Joy) -> None:
        """
        Process incoming joystick message.
        Updates the deadman button state and stores the message for manual drive.
        """
        self._last_joy_time = time.monotonic()
        self._latest_joy = msg

        # Check if deadman button index is within bounds
        if self.deadman_button_idx < len(msg.buttons):
            self._deadman_held = bool(msg.buttons[self.deadman_button_idx])
        else:
            self._deadman_held = False
            self.get_logger().warn(
                f'deadman_button_idx={self.deadman_button_idx} out of range '
                f'(joy has {len(msg.buttons)} buttons). '
                f'Car will NOT move until correct button index is set.',
                throttle_duration_sec=5.0
            )

    def _drive_cmd_callback(self, msg: AckermannDriveStamped) -> None:
        """Cache the latest autonomous drive command from planners."""
        self._latest_drive_cmd = msg

    def _state_callback(self, msg: String) -> None:
        """Cache the current FSM state to determine gate behaviour."""
        self._current_state = msg.data.strip().upper()

    # ------------------------------------------------------------------
    # Main gate loop
    # ------------------------------------------------------------------

    def _gate_loop(self) -> None:
        """
        Core safety gate — runs at 50 Hz.

        Decision tree:
            1. If EMERGENCY_STOP or IDLE → always publish zero.
            2. If joy timed out (controller disconnected) → publish zero + warn.
            3. If deadman NOT held → publish zero.
            4. If deadman HELD:
                a. MANUAL state → build drive from joystick axes.
                b. AUTONOMOUS state → forward /drive_cmd unchanged.
        """
        # --- Rule 1: Hard stops for certain FSM states ---
        if self._current_state in STOPPED_STATES:
            self._publish_zero('FSM state requires stopped car.')
            return

        # --- Rule 2: Joy timeout check (controller disconnected / battery dead) ---
        joy_age = time.monotonic() - self._last_joy_time
        if joy_age > self.joy_timeout_s:
            self._publish_zero(
                f'Joystick silent for {joy_age:.1f}s — STOPPING.',
                warn=True
            )
            return

        # --- Rule 3: Deadman not held → stop ---
        if not self._deadman_held:
            self._publish_zero()
            return

        # --- Rule 4a: Manual drive from joystick ---
        if self._current_state in MANUAL_STATES:
            self._publish_manual_drive()
            return

        # --- Rule 4b: Autonomous mode → forward drive_cmd ---
        if self._current_state in AUTONOMOUS_STATES:
            if self._latest_drive_cmd is not None:
                # Update stamp and re-publish
                self._latest_drive_cmd.header.stamp = self.get_clock().now().to_msg()
                self._drive_pub.publish(self._latest_drive_cmd)
            else:
                # No autonomous command yet → stay stopped
                self._publish_zero('Waiting for autonomous planner command...')
            return

        # Fallback for any unknown state
        self._publish_zero(f'Unknown state "{self._current_state}" — STOPPING.')

    # ------------------------------------------------------------------
    # Drive helpers
    # ------------------------------------------------------------------

    def _publish_manual_drive(self) -> None:
        """
        Construct an AckermannDriveStamped from raw joystick axes and publish.

        Axis values are in [-1.0, +1.0]. Scaled to physical units:
            speed = axis_value * manual_max_speed   [m/s]
            steer = axis_value * max_steering_angle [rad]
        """
        if self._latest_joy is None:
            self._publish_zero()
            return

        joy = self._latest_joy

        # --- Extract axis values (with bounds checking) ---
        n_axes = len(joy.axes)
        speed_raw: float = (joy.axes[self.manual_speed_axis]
                            if self.manual_speed_axis < n_axes else 0.0)
        steer_raw: float = (joy.axes[self.manual_steer_axis]
                            if self.manual_steer_axis < n_axes else 0.0)

        # --- Scale to physical units ---
        speed: float = speed_raw * self.manual_max_speed
        steer: float = steer_raw * self.max_steering_angle

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = speed
        msg.drive.steering_angle = steer
        self._drive_pub.publish(msg)

    def _publish_zero(
        self,
        reason: str = '',
        warn: bool = False
    ) -> None:
        """
        Publish a zero-velocity AckermannDriveStamped to halt the car.

        Parameters
        ----------
        reason : str
            Optional log message explaining why the car is being stopped.
        warn : bool
            If True, log at WARN level; otherwise use DEBUG.
        """
        if reason:
            if warn:
                self.get_logger().warn(reason, throttle_duration_sec=2.0)
            else:
                self.get_logger().debug(reason)

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        self._drive_pub.publish(msg)


def main(args=None) -> None:
    """Entry point for the deadman_switch ROS 2 node."""
    rclpy.init(args=args)
    node = DeadmanSwitch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('DeadmanSwitch shutting down — publishing zero.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
