"""
race_state_machine.py
=====================
High-level Finite State Machine (FSM) that controls which stack module is
active at any given time. Inspired by ForzaETH's stack_master architecture.

States:
    IDLE             — Car is stationary, all motion suppressed.
    MAPPING_MANUAL   — Joystick commands are forwarded to VESC; SLAM running.
    MAPPING_AUTO     — Follow-The-Gap node is active; SLAM running.
    RACING           — MPPI controller is active; racing line loaded.
    EMERGENCY_STOP   — All motion overridden; zero velocity at 50 Hz for 2 s.

Transitions:
    Triggered by publishing a std_msgs/String to /race_mode:
        "IDLE", "MAPPING_MANUAL", "MAPPING_AUTO", "RACING", "EMERGENCY_STOP"

On EMERGENCY_STOP:
    Immediately publishes a zero-speed AckermannDriveStamped for 2 seconds
    at 50 Hz to ensure the VESC and car come to a complete stop.
"""

from __future__ import annotations

import time
from enum import Enum
from typing import List

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String


class RaceState(Enum):
    """Enumeration of all valid stack states."""
    IDLE = 'IDLE'
    MAPPING_MANUAL = 'MAPPING_MANUAL'
    MAPPING_AUTO = 'MAPPING_AUTO'
    TIME_TRIAL = 'TIME_TRIAL'        # Pure Pursuit — solo racing, fastest clean lap
    HEAD_TO_HEAD = 'HEAD_TO_HEAD'    # MPPI — opponent avoidance + overtaking
    RACING = 'RACING'                # Alias for HEAD_TO_HEAD (legacy)
    EMERGENCY_STOP = 'EMERGENCY_STOP'


class RaceStateMachine(Node):
    """
    ROS 2 FSM node for the F1TENTH race stack.

    Subscribes:
        /race_mode  (std_msgs/String)   — State transition commands
        /drive      (ackermann_msgs/AckermannDriveStamped) — Monitors drive commands

    Publishes:
        /drive      (ackermann_msgs/AckermannDriveStamped) — Overrides with zero on EMERGENCY_STOP
        /race_stack/current_state (std_msgs/String)        — Broadcasts current state
    """

    def __init__(self) -> None:
        """Initialise the FSM node."""
        super().__init__('race_state_machine')

        # ------------------------------------------------------------------
        # Declare parameters
        # ------------------------------------------------------------------
        # Initial state can be overridden from launch file
        self.declare_parameter('initial_state', 'IDLE')
        # Duration of emergency stop zero-velocity broadcast
        self.declare_parameter('emergency_stop_duration_s', 2.0)

        self._load_params()
        self.add_on_set_parameters_callback(self._param_callback)

        # ------------------------------------------------------------------
        # FSM state
        # ------------------------------------------------------------------
        initial_state_str: str = self.get_parameter('initial_state').value
        try:
            self._state: RaceState = RaceState(initial_state_str.upper())
        except ValueError:
            self.get_logger().warn(
                f'Invalid initial_state "{initial_state_str}", defaulting to IDLE.'
            )
            self._state = RaceState.IDLE

        self._in_emergency: bool = False
        self._emergency_end_time: float = 0.0

        # ------------------------------------------------------------------
        # Publisher: drive commands (for EMERGENCY override)
        # ------------------------------------------------------------------
        self._drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)

        # Publisher: current state broadcast
        self._state_pub = self.create_publisher(
            String, '/race_stack/current_state', 10)

        # ------------------------------------------------------------------
        # Subscriber: state transition commands
        # ------------------------------------------------------------------
        self._mode_sub = self.create_subscription(
            String, '/race_mode', self._mode_callback, 10)

        # ------------------------------------------------------------------
        # Emergency stop timer at 50 Hz
        # ------------------------------------------------------------------
        self._estop_timer = self.create_timer(0.02, self._emergency_stop_loop)

        # ------------------------------------------------------------------
        # State broadcast timer at 2 Hz
        # ------------------------------------------------------------------
        self._state_broadcast_timer = self.create_timer(0.5, self._broadcast_state)

        self.get_logger().info(
            f'RaceStateMachine started | initial state: {self._state.value}'
        )

    # ------------------------------------------------------------------
    # Parameter management
    # ------------------------------------------------------------------

    def _load_params(self) -> None:
        """Load parameters into instance variables."""
        self.emergency_stop_duration_s: float = self.get_parameter(
            'emergency_stop_duration_s').value

    def _param_callback(self, params: List[Parameter]) -> SetParametersResult:
        """Handle live parameter updates."""
        self._load_params()
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # State transition
    # ------------------------------------------------------------------

    def _mode_callback(self, msg: String) -> None:
        """
        Handle a state transition request from /race_mode.

        Valid messages: "IDLE", "MAPPING_MANUAL", "MAPPING_AUTO",
                        "RACING", "EMERGENCY_STOP"

        Parameters
        ----------
        msg : std_msgs/String
            Requested state name.
        """
        requested_str: str = msg.data.strip().upper()

        try:
            requested_state = RaceState(requested_str)
        except ValueError:
            self.get_logger().error(
                f'Unknown state requested: "{msg.data}". '
                f'Valid states: {[s.value for s in RaceState]}'
            )
            return

        old_state = self._state
        self._state = requested_state

        self.get_logger().info(
            f'State transition: {old_state.value} → {self._state.value}'
        )

        # --- Handle EMERGENCY_STOP entry actions ---
        if self._state == RaceState.EMERGENCY_STOP:
            self._in_emergency = True
            self._emergency_end_time = time.monotonic() + self.emergency_stop_duration_s
            self.get_logger().error(
                f'EMERGENCY STOP activated! Publishing zero velocity for '
                f'{self.emergency_stop_duration_s:.1f} s.'
            )

        # --- Announce state change to any monitoring tools ---
        self._broadcast_state()

    # ------------------------------------------------------------------
    # Emergency stop loop
    # ------------------------------------------------------------------

    def _emergency_stop_loop(self) -> None:
        """
        Timer callback at 50 Hz. During an active emergency stop, overrides
        /drive with a zero-velocity command for the configured duration.
        After the duration expires, transitions to IDLE.
        """
        if not self._in_emergency:
            return

        if time.monotonic() < self._emergency_end_time:
            # Keep broadcasting zero velocity
            self._publish_zero_drive()
        else:
            # Emergency period expired → return to IDLE
            self._in_emergency = False
            self._state = RaceState.IDLE
            self.get_logger().info(
                'Emergency stop duration elapsed. Transitioning to IDLE.'
            )
            self._broadcast_state()

    def _publish_zero_drive(self) -> None:
        """Publish a zero-speed AckermannDriveStamped to halt the car."""
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        msg.drive.acceleration = -10.0  # Request hard braking
        self._drive_pub.publish(msg)

    # ------------------------------------------------------------------
    # State broadcasting
    # ------------------------------------------------------------------

    def _broadcast_state(self) -> None:
        """Publish the current FSM state to /race_stack/current_state."""
        msg = String()
        msg.data = self._state.value
        self._state_pub.publish(msg)


def main(args=None) -> None:
    """Entry point for the race_state_machine ROS 2 node."""
    rclpy.init(args=args)
    node = RaceStateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('RaceStateMachine shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
