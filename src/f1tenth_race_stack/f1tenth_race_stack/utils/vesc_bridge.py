"""
vesc_bridge.py
==============
Bridges between the high-level /drive topic (AckermannDriveStamped) and
the VESC MK-VI motor controller ROS 2 driver topics.

Purpose:
    The MPPI controller, Follow-The-Gap node, and joystick teleop all publish
    to the unified /drive topic. This node converts Ackermann commands into
    the specific VESC driver topic format expected by the vesc_driver package.

Conversion:
    Motor speed:
        erpm = speed_mps * erpm_gain
        Published to: /commands/motor/speed (std_msgs/Float64)

    Servo position:
        servo_pos = steering_angle * steering_to_servo_gain + steering_to_servo_offset
        Clamped to [0.0, 1.0]
        Published to: /commands/servo/position (std_msgs/Float64)

All parameters are live-tunable via ros2 param set or rqt_reconfigure.
Tune erpm_gain and servo parameters using system_id.launch.py.
"""

from __future__ import annotations

from typing import List

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64


class VescBridge(Node):
    """
    ROS 2 node bridging AckermannDriveStamped → VESC driver topics.

    Subscribes:
        /drive (ackermann_msgs/AckermannDriveStamped)

    Publishes:
        /commands/motor/speed    (std_msgs/Float64)  — ERPM command
        /commands/servo/position (std_msgs/Float64)  — Servo position [0.0, 1.0]
    """

    def __init__(self) -> None:
        """Initialise the bridge node with conversion parameters."""
        super().__init__('vesc_bridge')

        # ------------------------------------------------------------------
        # Declare all tunable conversion parameters
        # ------------------------------------------------------------------
        self.declare_parameter('vesc_bridge.erpm_gain', 4614.0)
        self.declare_parameter('vesc_bridge.steering_to_servo_gain', -0.64)
        self.declare_parameter('vesc_bridge.steering_to_servo_offset', 0.5)

        # Safety limits
        self.declare_parameter('vehicle.max_speed', 10.0)
        self.declare_parameter('vehicle.max_steering_angle', 0.43)

        self._load_params()
        self.add_on_set_parameters_callback(self._param_callback)

        # ------------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------------
        self._motor_pub = self.create_publisher(
            Float64, '/commands/motor/speed', 10)
        self._servo_pub = self.create_publisher(
            Float64, '/commands/servo/position', 10)

        # ------------------------------------------------------------------
        # Subscriber
        # ------------------------------------------------------------------
        self._drive_sub = self.create_subscription(
            AckermannDriveStamped, '/drive', self._drive_callback, 10)

        self.get_logger().info(
            f'VescBridge started | erpm_gain={self.erpm_gain} | '
            f'servo_gain={self.steering_to_servo_gain:.3f} | '
            f'servo_offset={self.steering_to_servo_offset:.3f}'
        )

    # ------------------------------------------------------------------
    # Parameter management
    # ------------------------------------------------------------------

    def _load_params(self) -> None:
        """Load conversion parameters into instance variables."""
        self.erpm_gain: float = self.get_parameter('vesc_bridge.erpm_gain').value
        self.steering_to_servo_gain: float = self.get_parameter(
            'vesc_bridge.steering_to_servo_gain').value
        self.steering_to_servo_offset: float = self.get_parameter(
            'vesc_bridge.steering_to_servo_offset').value
        self.max_speed: float = self.get_parameter('vehicle.max_speed').value
        self.max_steering_angle: float = self.get_parameter(
            'vehicle.max_steering_angle').value

    def _param_callback(self, params: List[Parameter]) -> SetParametersResult:
        """Live parameter update — new values applied on next drive callback."""
        self._load_params()
        self.get_logger().info('VescBridge parameters updated live.')
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Drive callback
    # ------------------------------------------------------------------

    def _drive_callback(self, msg: AckermannDriveStamped) -> None:
        """
        Convert an AckermannDriveStamped command into VESC driver messages.

        Parameters
        ----------
        msg : AckermannDriveStamped
            Incoming drive command with .drive.speed [m/s] and
            .drive.steering_angle [rad].
        """
        speed_mps: float = msg.drive.speed
        steering_rad: float = msg.drive.steering_angle

        # --- Safety clamp ---
        speed_mps = max(-self.max_speed, min(self.max_speed, speed_mps))
        steering_rad = max(-self.max_steering_angle,
                           min(self.max_steering_angle, steering_rad))

        # --- Motor speed conversion ---
        # erpm = speed [m/s] * erpm_gain [ERPM/(m/s)]
        erpm: float = speed_mps * self.erpm_gain
        motor_msg = Float64()
        motor_msg.data = erpm
        self._motor_pub.publish(motor_msg)

        # --- Servo position conversion ---
        # servo_pos = steering_angle [rad] * gain + offset
        # Clamped to [0.0, 1.0] (VESC servo range)
        servo_pos: float = (steering_rad * self.steering_to_servo_gain
                            + self.steering_to_servo_offset)
        servo_pos = max(0.0, min(1.0, servo_pos))
        servo_msg = Float64()
        servo_msg.data = servo_pos
        self._servo_pub.publish(servo_msg)


def main(args=None) -> None:
    """Entry point for the vesc_bridge ROS 2 node."""
    rclpy.init(args=args)
    node = VescBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('VescBridge shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
