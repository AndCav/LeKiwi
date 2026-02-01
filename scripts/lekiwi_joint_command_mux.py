#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class LeKiwiJointCommandMux(Node):
    def __init__(self) -> None:
        super().__init__("lekiwi_joint_command_mux")
        self.declare_parameter("publish_rate_hz", 50.0)

        self._wheel_joints = ["joint7", "joint8", "joint9"]
        self._arm_joints = [
            "STS3215_03a_v1_Revolute_45",
            "STS3215_03a_v1_1_Revolute_49",
            "STS3215_03a_v1_2_Revolute_51",
            "STS3215_03a_v1_3_Revolute_53",
            "STS3215_03a_Wrist_Roll_v1_Revolute_55",
            "STS3215_03a_v1_4_Revolute_57",
        ]

        self._wheel_cmd = [0.0] * len(self._wheel_joints)
        self._arm_cmd = [0.0] * (len(self._arm_joints) - 1)
        self._gripper_cmd = [0.0]

        self.create_subscription(
            Float64MultiArray,
            "/lekiwi/kiwi_controller/commands",
            self._on_wheel_cmd,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/lekiwi/arm_position_controller/commands",
            self._on_arm_cmd,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/lekiwi/gripper_position_controller/commands",
            self._on_gripper_cmd,
            10,
        )

        self._pub_wheels = self.create_publisher(
            JointState, "/isaac_joint_commands_wheels", 10
        )
        self._pub_arm = self.create_publisher(
            JointState, "/isaac_joint_commands_arm", 10
        )

        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        period = 1.0 / rate_hz if rate_hz > 0.0 else 0.02
        self.create_timer(period, self._publish)

    def _on_wheel_cmd(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != len(self._wheel_cmd):
            self.get_logger().warn(
                f"wheel command size {len(msg.data)} != {len(self._wheel_cmd)}"
            )
        for i in range(min(len(msg.data), len(self._wheel_cmd))):
            self._wheel_cmd[i] = float(msg.data[i])

    def _on_arm_cmd(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != len(self._arm_cmd):
            self.get_logger().warn(
                f"arm command size {len(msg.data)} != {len(self._arm_cmd)}"
            )
        for i in range(min(len(msg.data), len(self._arm_cmd))):
            self._arm_cmd[i] = float(msg.data[i])

    def _on_gripper_cmd(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != 1:
            self.get_logger().warn(f"gripper command size {len(msg.data)} != 1")
        if msg.data:
            self._gripper_cmd[0] = float(msg.data[0])

    def _publish(self) -> None:
        now = self.get_clock().now().to_msg()

        wheels_msg = JointState()
        wheels_msg.header.stamp = now
        wheels_msg.name = list(self._wheel_joints)
        wheels_msg.position = [0.0] * len(self._wheel_joints)
        wheels_msg.velocity = list(self._wheel_cmd)
        wheels_msg.effort = [0.0] * len(self._wheel_joints)
        self._pub_wheels.publish(wheels_msg)

        arm_msg = JointState()
        arm_msg.header.stamp = now
        arm_msg.name = list(self._arm_joints)
        arm_positions = list(self._arm_cmd) + list(self._gripper_cmd)
        arm_msg.position = arm_positions
        arm_msg.velocity = [0.0] * len(self._arm_joints)
        arm_msg.effort = [0.0] * len(self._arm_joints)
        self._pub_arm.publish(arm_msg)


def main() -> None:
    rclpy.init()
    node = LeKiwiJointCommandMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
