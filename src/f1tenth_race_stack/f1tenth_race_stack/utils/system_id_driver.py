"""
system_id_driver.py
===================
Open-loop system identification driver for the F1TENTH car.

Purpose:
    Drive the car at a sequence of fixed speeds and record odometry to
    calibrate the VESC erpm_gain and servo steering parameters. This node
    is launched by system_id.launch.py and drives the car in a straight
    line at each test speed for a fixed duration.

Procedure:
    1. Wait for operator readiness (5 second countdown).
    2. For each speed in the test_speeds list:
       a. Publish AckermannDriveStamped at that speed with zero steering.
       b. Record odom-derived speed for 3 seconds.
       c. Log the average measured speed vs commanded speed.
    3. After all speeds tested, publish zero velocity.
    4. Log a calibration table to the console.

Usage:
    ros2 run f1tenth_race_stack system_id_driver
    Then read the console output to compute erpm_gain = ERPM / measured_speed.
"""

from __future__ import annotations

import time
from typing import List

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry


class SystemIdDriver(Node):
    """
    Open-loop system identification node.
    Drives at fixed speeds and logs measured vs commanded speed.
    """

    def __init__(self) -> None:
        """Initialise the system ID driver."""
        super().__init__('system_id_driver')

        # ------------------------------------------------------------------
        # Declare parameters
        # ------------------------------------------------------------------
        self.declare_parameter('test_speeds', [0.5, 1.0, 1.5, 2.0, 2.5, 3.0])
        self.declare_parameter('test_duration_s', 3.0)
        self.declare_parameter('startup_delay_s', 5.0)

        self._test_speeds: List[float] = list(
            self.get_parameter('test_speeds').value)
        self._test_duration: float = self.get_parameter('test_duration_s').value
        self._startup_delay: float = self.get_parameter('startup_delay_s').value

        # ------------------------------------------------------------------
        # Odometry tracking
        # ------------------------------------------------------------------
        self._measured_speeds: List[float] = []
        self._current_commanded_speed: float = 0.0

        # ------------------------------------------------------------------
        # Publisher
        # ------------------------------------------------------------------
        self._drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)

        # ------------------------------------------------------------------
        # Subscriber
        # ------------------------------------------------------------------
        self._odom_sub = self.create_subscription(
            Odometry, '/ego_racecar/odom', self._odom_callback, 20)

        # ------------------------------------------------------------------
        # One-shot timer to start after startup delay
        # ------------------------------------------------------------------
        self._start_timer = self.create_timer(
            self._startup_delay, self._begin_system_id)
        self._started = False

        self.get_logger().info(
            f'SystemIdDriver ready. Starting in {self._startup_delay:.0f} s...\n'
            f'Test speeds: {self._test_speeds} m/s'
        )

    def _odom_callback(self, msg: Odometry) -> None:
        """Record measured speed from odometry during active test."""
        if self._current_commanded_speed > 0.0:
            self._measured_speeds.append(abs(msg.twist.twist.linear.x))

    def _begin_system_id(self) -> None:
        """
        Run the full system identification sequence.
        This is called once after startup delay and then cancelled.
        """
        if self._started:
            return
        self._started = True
        self._start_timer.cancel()

        self.get_logger().info('System ID sequence starting...')
        results: List[tuple] = []

        for cmd_speed in self._test_speeds:
            self.get_logger().info(f'Testing speed: {cmd_speed:.2f} m/s...')
            self._current_commanded_speed = cmd_speed
            self._measured_speeds = []

            # Drive at commanded speed
            t_end = time.monotonic() + self._test_duration
            while time.monotonic() < t_end:
                self._publish_drive(cmd_speed, 0.0)
                time.sleep(0.05)  # 20 Hz publish rate

            # Stop and collect statistics
            self._current_commanded_speed = 0.0
            self._publish_drive(0.0, 0.0)
            time.sleep(0.5)

            if self._measured_speeds:
                avg_speed = sum(self._measured_speeds) / len(self._measured_speeds)
                results.append((cmd_speed, avg_speed))
                self.get_logger().info(
                    f'  Commanded: {cmd_speed:.2f} m/s | '
                    f'Measured: {avg_speed:.3f} m/s | '
                    f'Samples: {len(self._measured_speeds)}'
                )
            else:
                self.get_logger().warn(f'  No odometry received for {cmd_speed} m/s test.')

        # Print calibration summary
        self.get_logger().info('\n=== SYSTEM ID RESULTS ===')
        self.get_logger().info('cmd_speed | meas_speed | ratio (tune erpm_gain)')
        self.get_logger().info('-' * 50)
        for cmd, meas in results:
            ratio = cmd / meas if meas > 0.01 else float('nan')
            self.get_logger().info(f'  {cmd:.2f} m/s | {meas:.3f} m/s | {ratio:.4f}')
        self.get_logger().info('System ID complete. Review log to tune erpm_gain.')

    def _publish_drive(self, speed: float, steer: float) -> None:
        """Publish a single drive command."""
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = speed
        msg.drive.steering_angle = steer
        self._drive_pub.publish(msg)


def main(args=None) -> None:
    """Entry point for system_id_driver node."""
    rclpy.init(args=args)
    node = SystemIdDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('SystemIdDriver shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
