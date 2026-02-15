#!/usr/bin/env python3
from builtin_interfaces.msg import Duration
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class LeKiwiJointCommandMux(Node):
    def __init__(self) -> None:
        super().__init__("lekiwi_joint_command_mux")
        self.declare_parameter("publish_rate_hz", 50.0)
        self._arm_joints = [
            "STS3215_03a_v1_Revolute_45",
            "STS3215_03a_v1_1_Revolute_49",
            "STS3215_03a_v1_2_Revolute_51",
            "STS3215_03a_v1_3_Revolute_53",
            "STS3215_03a_Wrist_Roll_v1_Revolute_55",
            "STS3215_03a_v1_4_Revolute_57",
        ]
        self._arm_cmd = [0.0] * (len(self._arm_joints) - 1)
        self._arm_control_joints = list(self._arm_joints[:-1])
        self._arm_joint_to_index = {
            joint_name: idx for idx, joint_name in enumerate(self._arm_control_joints)
        }
        self._arm_traj_points = []
        self._arm_traj_start_time = None
        self._gripper_cmd = [0.0]
        self._gripper_joint = "STS3215_03a_v1_4_Revolute_57"
        self._gripper_traj_points = []
        self._gripper_traj_start_time = None

        self.create_subscription(
            Float64MultiArray,
            "/lekiwi/arm_position_controller/commands",
            self._on_arm_cmd,
            10,
        )
        self.create_subscription(
            JointTrajectory,
            "/lekiwi/arm_position_controller/joint_trajectory",
            self._on_arm_trajectory,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/lekiwi/gripper_position_controller/commands",
            self._on_gripper_cmd,
            10,
        )
        self.create_subscription(
            JointTrajectory,
            "/lekiwi/gripper_position_controller/joint_trajectory",
            self._on_gripper_trajectory,
            10,
        )

        self._pub_arm = self.create_publisher(
            JointState, "/isaac_joint_commands_arm", 10
        )
        self._pub_arm_trajectory = self.create_publisher(
            JointTrajectory, "/lekiwi/arm_position_controller/joint_trajectory", 10
        )
        self._pub_gripper_trajectory = self.create_publisher(
            JointTrajectory, "/lekiwi/gripper_position_controller/joint_trajectory", 10
        )

        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        period = 1.0 / rate_hz if rate_hz > 0.0 else 0.02
        self.create_timer(period, self._publish)

    def _on_arm_cmd(self, msg: Float64MultiArray) -> None:
        n = len(msg.data)
        if n < 1 or n > len(self._arm_cmd):
            self.get_logger().warn(
                f"arm command size {n} must be in [1, {len(self._arm_cmd)}]"
            )
            return

        joint_names = list(self._arm_control_joints[:n])
        start_positions = [float(v) for v in self._arm_cmd[:n]]
        target_positions = [float(v) for v in msg.data[:n]]

        traj_msg = JointTrajectory()
        traj_msg.joint_names = joint_names
        start_point = JointTrajectoryPoint()
        start_point.positions = start_positions
        start_point.time_from_start = Duration(sec=0, nanosec=0)
        target_point = JointTrajectoryPoint()
        target_point.positions = target_positions
        target_point.time_from_start = Duration(sec=1, nanosec=0)
        traj_msg.points = [start_point, target_point]

        self._set_arm_trajectory(traj_msg)
        self._pub_arm_trajectory.publish(traj_msg)

    def _on_arm_trajectory(self, msg: JointTrajectory) -> None:
        self._set_arm_trajectory(msg)

    def _set_arm_trajectory(self, msg: JointTrajectory) -> None:
        if not msg.points:
            return

        current_cmd = [float(v) for v in self._arm_cmd]
        traj_points = []

        if msg.joint_names:
            if len(set(msg.joint_names)) != len(msg.joint_names):
                self.get_logger().warn(
                    "arm trajectory rejected: duplicate joint names in joint_names"
                )
                return

            invalid_joint_names = [
                joint_name
                for joint_name in msg.joint_names
                if joint_name not in self._arm_joint_to_index
            ]
            if invalid_joint_names:
                self.get_logger().warn(
                    "arm trajectory rejected: unknown joints "
                    + ", ".join(invalid_joint_names)
                )
                return

            indexed_joints = [
                (idx, self._arm_joint_to_index[joint_name])
                for idx, joint_name in enumerate(msg.joint_names)
            ]
            for point in msg.points:
                if len(point.positions) < len(indexed_joints):
                    self.get_logger().warn(
                        "arm trajectory rejected: positions shorter than joint_names"
                    )
                    return

                t = float(point.time_from_start.sec) + float(point.time_from_start.nanosec) * 1e-9
                if t < 0.0:
                    self.get_logger().warn("arm trajectory rejected: negative time_from_start")
                    return

                full_positions = list(current_cmd)
                for pos_idx, joint_idx in indexed_joints:
                    full_positions[joint_idx] = float(point.positions[pos_idx])
                traj_points.append((t, full_positions))
        else:
            for point in msg.points:
                n = len(point.positions)
                if n < 1 or n > len(self._arm_cmd):
                    self.get_logger().warn(
                        f"arm trajectory rejected: positions size {n} must be in [1, {len(self._arm_cmd)}]"
                    )
                    return

                t = float(point.time_from_start.sec) + float(point.time_from_start.nanosec) * 1e-9
                if t < 0.0:
                    self.get_logger().warn("arm trajectory rejected: negative time_from_start")
                    return

                full_positions = list(current_cmd)
                for i in range(n):
                    full_positions[i] = float(point.positions[i])
                traj_points.append((t, full_positions))

        if not traj_points:
            return

        traj_points.sort(key=lambda p: p[0])
        if traj_points[0][0] > 0.0:
            traj_points.insert(0, (0.0, list(current_cmd)))

        self._arm_traj_points = traj_points
        self._arm_traj_start_time = self.get_clock().now()
        self._arm_cmd = list(traj_points[0][1])

    def _update_arm_cmd_from_trajectory(self, now):
        if not self._arm_traj_points or self._arm_traj_start_time is None:
            return

        elapsed = (now - self._arm_traj_start_time).nanoseconds * 1e-9
        points = self._arm_traj_points

        if elapsed <= points[0][0]:
            self._arm_cmd = list(points[0][1])
            return

        if elapsed >= points[-1][0]:
            self._arm_cmd = list(points[-1][1])
            self._arm_traj_points = []
            self._arm_traj_start_time = None
            return

        for i in range(len(points) - 1):
            t0, p0 = points[i]
            t1, p1 = points[i + 1]
            if t0 <= elapsed <= t1:
                if t1 <= t0:
                    self._arm_cmd = list(p1)
                    return
                alpha = (elapsed - t0) / (t1 - t0)
                self._arm_cmd = [
                    float(a + alpha * (b - a)) for a, b in zip(p0, p1)
                ]
                return

    def _on_gripper_cmd(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != 1:
            self.get_logger().warn(f"gripper command size {len(msg.data)} != 1")
            return

        start_position = float(self._gripper_cmd[0])
        target_position = float(msg.data[0])
        traj_msg = JointTrajectory()
        traj_msg.joint_names = [self._gripper_joint]
        start_point = JointTrajectoryPoint()
        start_point.positions = [start_position]
        start_point.time_from_start = Duration(sec=0, nanosec=0)
        target_point = JointTrajectoryPoint()
        target_point.positions = [target_position]
        target_point.time_from_start = Duration(sec=0, nanosec=200_000_000)
        traj_msg.points = [start_point, target_point]
        self._set_gripper_trajectory(traj_msg)
        self._pub_gripper_trajectory.publish(traj_msg)

    def _on_gripper_trajectory(self, msg: JointTrajectory) -> None:
        self._set_gripper_trajectory(msg)

    def _set_gripper_trajectory(self, msg: JointTrajectory) -> None:
        if not msg.points:
            return

        joint_index = None
        if msg.joint_names:
            if self._gripper_joint not in msg.joint_names:
                return
            joint_index = msg.joint_names.index(self._gripper_joint)
        else:
            joint_index = 0

        traj_points = []
        for point in msg.points:
            if joint_index >= len(point.positions):
                return
            t = float(point.time_from_start.sec) + float(point.time_from_start.nanosec) * 1e-9
            traj_points.append((t, float(point.positions[joint_index])))

        if not traj_points:
            return

        # Ensure temporal ordering and inject a start point when needed for smooth ramps.
        traj_points.sort(key=lambda p: p[0])
        if traj_points[0][0] > 0.0:
            traj_points.insert(0, (0.0, float(self._gripper_cmd[0])))

        self._gripper_traj_points = traj_points
        self._gripper_traj_start_time = self.get_clock().now()
        self._gripper_cmd[0] = float(traj_points[0][1])

    def _update_gripper_cmd_from_trajectory(self, now):
        if not self._gripper_traj_points or self._gripper_traj_start_time is None:
            return

        elapsed = (now - self._gripper_traj_start_time).nanoseconds * 1e-9
        points = self._gripper_traj_points

        if elapsed <= points[0][0]:
            self._gripper_cmd[0] = float(points[0][1])
            return

        if elapsed >= points[-1][0]:
            self._gripper_cmd[0] = float(points[-1][1])
            self._gripper_traj_points = []
            self._gripper_traj_start_time = None
            return

        for i in range(len(points) - 1):
            t0, p0 = points[i]
            t1, p1 = points[i + 1]
            if t0 <= elapsed <= t1:
                if t1 <= t0:
                    self._gripper_cmd[0] = float(p1)
                    return
                alpha = (elapsed - t0) / (t1 - t0)
                self._gripper_cmd[0] = float(p0 + alpha * (p1 - p0))
                return

    def _publish(self) -> None:
        now_clock = self.get_clock().now()
        self._update_arm_cmd_from_trajectory(now_clock)
        self._update_gripper_cmd_from_trajectory(now_clock)
        now = now_clock.to_msg()

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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
