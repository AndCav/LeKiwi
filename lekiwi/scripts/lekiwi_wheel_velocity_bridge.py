#!/usr/bin/env python3
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class LeKiwiWheelVelocityBridge(Node):
    def __init__(self) -> None:
        super().__init__("lekiwi_wheel_velocity_bridge")

        self.declare_parameter("wheel_joint_names", ["joint7", "joint8", "joint9"])
        self.declare_parameter(
            "command_topic", "/lekiwi/wheel_velocity_controller/commands"
        )
        self.declare_parameter("isaac_topic", "/isaac_joint_commands_wheels")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("publish_startup_zero", True)

        self._wheel_joint_names = [
            str(v) for v in list(self.get_parameter("wheel_joint_names").value)
        ]
        if not self._wheel_joint_names:
            self.get_logger().warn(
                "wheel_joint_names is empty, falling back to [joint7, joint8, joint9]"
            )
            self._wheel_joint_names = ["joint7", "joint8", "joint9"]
        self._wheel_count = len(self._wheel_joint_names)

        self._command_timeout_s = float(self.get_parameter("command_timeout_s").value)
        if self._command_timeout_s < 0.0:
            self.get_logger().warn("command_timeout_s < 0.0, forcing to 0.0")
            self._command_timeout_s = 0.0
        self._publish_startup_zero = bool(
            self.get_parameter("publish_startup_zero").value
        )

        command_topic = str(self.get_parameter("command_topic").value)
        isaac_topic = str(self.get_parameter("isaac_topic").value)

        self._cmd_velocity = [0.0] * self._wheel_count
        self._last_cmd_time = None
        self._has_received_cmd = False

        self.create_subscription(
            Float64MultiArray, command_topic, self._on_wheel_velocity_cmd, 10
        )
        self._isaac_pub = self.create_publisher(JointState, isaac_topic, 10)

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        period_s = 1.0 / publish_rate_hz if publish_rate_hz > 0.0 else 0.02
        self.create_timer(period_s, self._on_timer)

    def _on_wheel_velocity_cmd(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != self._wheel_count:
            self.get_logger().warn(
                f"wheel command size {len(msg.data)} != {self._wheel_count}"
            )
            return

        self._cmd_velocity = [float(v) for v in msg.data]
        self._last_cmd_time = self.get_clock().now()
        self._has_received_cmd = True

    def _is_command_fresh(self, now_clock) -> bool:
        if self._last_cmd_time is None:
            return False
        if self._command_timeout_s <= 0.0:
            return True
        dt_s = (now_clock - self._last_cmd_time).nanoseconds * 1e-9
        return dt_s <= self._command_timeout_s

    def _publish_velocity(self, velocity) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._wheel_joint_names)
        msg.position = []
        msg.velocity = list(velocity)
        msg.effort = []
        self._isaac_pub.publish(msg)

    def _on_timer(self) -> None:
        if (
            self._publish_startup_zero
            and not self._has_received_cmd
        ):
            self._publish_velocity([0.0] * self._wheel_count)
            return

        now_clock = self.get_clock().now()
        if self._is_command_fresh(now_clock):
            self._publish_velocity(self._cmd_velocity)
            return

        # Safety hold: keep sending zero while command stream is stale.
        self._publish_velocity([0.0] * self._wheel_count)


def main() -> None:
    rclpy.init()
    node = LeKiwiWheelVelocityBridge()
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
