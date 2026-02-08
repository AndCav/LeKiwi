#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class LeKiwiCmdVelToWheelVelocity(Node):
    def __init__(self) -> None:
        super().__init__("lekiwi_cmd_vel_to_wheel_velocity")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter(
            "wheel_command_topic", "/lekiwi/wheel_velocity_controller/commands"
        )
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("wheel_radius_m", 0.05)
        self.declare_parameter("robot_radius_m", 0.08)
        self.declare_parameter("wheel_angles_deg", [0.0, 120.0, 240.0])
        self.declare_parameter("wheel_signs", [1.0, 1.0, 1.0])
        self.declare_parameter("max_wheel_speed_rad_s", 0.0)

        self._wheel_angles_rad = self._load_triplet_parameter(
            "wheel_angles_deg", [0.0, 120.0, 240.0], math.radians
        )
        self._wheel_signs = self._load_triplet_parameter(
            "wheel_signs", [1.0, 1.0, 1.0], float
        )

        self._wheel_radius_m = float(self.get_parameter("wheel_radius_m").value)
        if self._wheel_radius_m <= 0.0:
            self.get_logger().warn("wheel_radius_m <= 0.0, forcing to 0.05")
            self._wheel_radius_m = 0.05

        self._robot_radius_m = float(self.get_parameter("robot_radius_m").value)
        if self._robot_radius_m < 0.0:
            self.get_logger().warn("robot_radius_m < 0.0, forcing to 0.08")
            self._robot_radius_m = 0.08

        self._max_wheel_speed_rad_s = float(
            self.get_parameter("max_wheel_speed_rad_s").value
        )
        if self._max_wheel_speed_rad_s < 0.0:
            self.get_logger().warn("max_wheel_speed_rad_s < 0.0, forcing to 0.0")
            self._max_wheel_speed_rad_s = 0.0

        self._command_timeout_s = float(self.get_parameter("command_timeout_s").value)
        if self._command_timeout_s < 0.0:
            self.get_logger().warn("command_timeout_s < 0.0, forcing to 0.0")
            self._command_timeout_s = 0.0

        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        wheel_command_topic = str(self.get_parameter("wheel_command_topic").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if publish_rate_hz <= 0.0:
            self.get_logger().warn("publish_rate_hz <= 0.0, forcing to 50.0")
            publish_rate_hz = 50.0

        self._latest_vx = 0.0
        self._latest_vy = 0.0
        self._latest_wz = 0.0
        self._last_cmd_time = None
        self._has_received_cmd = False
        self._timeout_zero_sent = False

        self.create_subscription(Twist, cmd_vel_topic, self._on_cmd_vel, 10)
        self._wheel_command_pub = self.create_publisher(
            Float64MultiArray, wheel_command_topic, 10
        )
        self.create_timer(1.0 / publish_rate_hz, self._on_timer)

    def _load_triplet_parameter(self, name: str, fallback, cast):
        raw = list(self.get_parameter(name).value)
        if len(raw) != 3:
            self.get_logger().warn(f"{name} size {len(raw)} != 3, using {fallback}")
            return [cast(v) for v in fallback]
        try:
            return [cast(v) for v in raw]
        except (TypeError, ValueError):
            self.get_logger().warn(f"{name} contains invalid values, using {fallback}")
            return [cast(v) for v in fallback]

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._latest_vx = float(msg.linear.x)
        self._latest_vy = float(msg.linear.y)
        self._latest_wz = float(msg.angular.z)
        self._last_cmd_time = self.get_clock().now()
        self._has_received_cmd = True
        self._timeout_zero_sent = False

    def _is_command_fresh(self, now_clock) -> bool:
        if self._last_cmd_time is None:
            return False
        if self._command_timeout_s <= 0.0:
            return True
        dt_s = (now_clock - self._last_cmd_time).nanoseconds * 1e-9
        return dt_s <= self._command_timeout_s

    def _twist_to_wheel_speeds(self, vx: float, vy: float, wz: float):
        wheel_speeds = []
        for wheel_angle, wheel_sign in zip(self._wheel_angles_rad, self._wheel_signs):
            speed = (
                (-math.sin(wheel_angle) * vx)
                + (math.cos(wheel_angle) * vy)
                + (self._robot_radius_m * wz)
            ) / self._wheel_radius_m
            speed *= wheel_sign
            if self._max_wheel_speed_rad_s > 0.0:
                speed = max(
                    -self._max_wheel_speed_rad_s,
                    min(self._max_wheel_speed_rad_s, speed),
                )
            wheel_speeds.append(speed)
        return wheel_speeds

    def _publish_wheel_speeds(self, wheel_speeds) -> None:
        msg = Float64MultiArray()
        msg.data = [float(v) for v in wheel_speeds]
        self._wheel_command_pub.publish(msg)

    def _on_timer(self) -> None:
        now_clock = self.get_clock().now()
        if self._is_command_fresh(now_clock):
            self._publish_wheel_speeds(
                self._twist_to_wheel_speeds(
                    self._latest_vx, self._latest_vy, self._latest_wz
                )
            )
            return

        if self._has_received_cmd and not self._timeout_zero_sent:
            self._publish_wheel_speeds([0.0, 0.0, 0.0])
            self._timeout_zero_sent = True


def main() -> None:
    rclpy.init()
    node = LeKiwiCmdVelToWheelVelocity()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
