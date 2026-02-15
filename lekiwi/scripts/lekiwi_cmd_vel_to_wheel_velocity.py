#!/usr/bin/env python3
import ast
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class LeKiwiCmdVelToWheelVelocity(Node):
    def __init__(self) -> None:
        super().__init__("lekiwi_cmd_vel_to_wheel_velocity")

        # Output ordering is fixed to joint7, joint8, joint9.
        # With current URDF layout this corresponds to:
        # wheel1 (front-left/top-left), wheel2 (rear), wheel3 (front-right/top-right).
        self.declare_parameter("cmd_vel_topic", "/lekiwi/cmd_vel")
        self.declare_parameter(
            "wheel_command_topic", "/lekiwi/wheel_velocity_controller/commands"
        )
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("linear_deadband_m_s", 0.01)
        self.declare_parameter("angular_deadband_rad_s", 0.05)
        self.declare_parameter("wheel_radius_m", 0.05)
        self.declare_parameter("robot_radius_m", 0.125)
        self.declare_parameter("wheel_angles_deg", [60.0, 180.0, 300.0])
        self.declare_parameter("wheel_signs", [1.0, 1.0, 1.0])
        self.declare_parameter("max_wheel_speed_rad_s", 0.0)

        self._wheel_angles_rad = self._load_triplet_parameter(
            "wheel_angles_deg", [60.0, 180.0, 300.0], math.radians
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
            self.get_logger().warn("robot_radius_m < 0.0, forcing to 0.125")
            self._robot_radius_m = 0.125

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

        self._linear_deadband_m_s = float(
            self.get_parameter("linear_deadband_m_s").value
        )
        if self._linear_deadband_m_s < 0.0:
            self.get_logger().warn("linear_deadband_m_s < 0.0, forcing to 0.01")
            self._linear_deadband_m_s = 0.01

        self._angular_deadband_rad_s = float(
            self.get_parameter("angular_deadband_rad_s").value
        )
        if self._angular_deadband_rad_s < 0.0:
            self.get_logger().warn("angular_deadband_rad_s < 0.0, forcing to 0.05")
            self._angular_deadband_rad_s = 0.05

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
        self.get_logger().info(
            "cmd_vel kinematics active (order joint7,joint8,joint9): "
            f"cmd_vel_topic={cmd_vel_topic}, "
            f"wheel_angles_deg={[math.degrees(v) for v in self._wheel_angles_rad]}, "
            f"wheel_signs={self._wheel_signs}, "
            f"wheel_radius_m={self._wheel_radius_m}, robot_radius_m={self._robot_radius_m}, "
            f"linear_deadband_m_s={self._linear_deadband_m_s}, "
            f"angular_deadband_rad_s={self._angular_deadband_rad_s}"
        )

    def _load_triplet_parameter(self, name: str, fallback, cast):
        value = self.get_parameter(name).value
        raw = value
        if isinstance(value, str):
            text = value.strip()
            try:
                raw = ast.literal_eval(text)
            except (ValueError, SyntaxError):
                raw = [part.strip() for part in text.split(",") if part.strip()]
        if not isinstance(raw, (list, tuple)):
            self.get_logger().warn(f"{name} is not a list, using {fallback}")
            return [cast(v) for v in fallback]

        if len(raw) != 3:
            self.get_logger().warn(f"{name} size {len(raw)} != 3, using {fallback}")
            return [cast(v) for v in fallback]
        try:
            return [cast(v) for v in raw]
        except (TypeError, ValueError):
            self.get_logger().warn(f"{name} contains invalid values, using {fallback}")
            return [cast(v) for v in fallback]

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._latest_vx = self._apply_deadband(
            float(msg.linear.x), self._linear_deadband_m_s
        )
        self._latest_vy = self._apply_deadband(
            float(msg.linear.y), self._linear_deadband_m_s
        )
        self._latest_wz = self._apply_deadband(
            float(msg.angular.z), self._angular_deadband_rad_s
        )
        self._last_cmd_time = self.get_clock().now()
        self._has_received_cmd = True
        self._timeout_zero_sent = False

    @staticmethod
    def _apply_deadband(value: float, deadband: float) -> float:
        if deadband <= 0.0:
            return value
        if abs(value) < deadband:
            return 0.0
        return value

    def _is_command_fresh(self, now_clock) -> bool:
        if self._last_cmd_time is None:
            return False
        if self._command_timeout_s <= 0.0:
            return True
        dt_s = (now_clock - self._last_cmd_time).nanoseconds * 1e-9
        return dt_s <= self._command_timeout_s

    def _twist_to_wheel_speeds(self, vx: float, vy: float, wz: float):
        # ROS body-frame convention: +x forward, +y left, +wz CCW.
        # alpha is measured from +x in CCW direction.
        wheel_speeds = []
        for wheel_angle, wheel_sign in zip(self._wheel_angles_rad, self._wheel_signs):
            speed = (
                (math.sin(wheel_angle) * vx)
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
