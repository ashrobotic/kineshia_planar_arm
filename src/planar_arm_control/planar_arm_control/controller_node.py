#!/usr/bin/env python3
"""Cartesian-target to joint-space trajectory controller.

Pipeline:
    Cartesian (x, y)
        -> validate / project onto workspace
        -> PlanarArm.inverse_kinematics()
        -> constraint checks
        -> quintic joint-space trajectory
        -> timer-driven execution through RobotBackend
        -> sensor_msgs/JointState

The planner never talks to hardware directly. SimulationBackend stores
commanded joints; a future DynamixelBackend can implement RobotBackend.
"""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from planar_arm_control.planar_arm import PlanarArm
from planar_arm_control.robot_backend import RobotBackend, SimulationBackend
from planar_arm_control.srv import MoveToTarget
from planar_arm_control.trajectory import QuinticJointTrajectory

LINK_LENGTHS = [3.0, 2.0, 1.5]
JOINT_NAMES = ["joint1", "joint2", "joint3"]
INITIAL_JOINTS = [0.0, 0.0, 0.0]
TRAJECTORY_CHECK_SAMPLES = 40
IK_POSITION_TOLERANCE = 1e-2


class ControllerNode(Node):
    """ROS 2 node that plans and executes planar-arm Cartesian moves."""

    def __init__(self) -> None:
        super().__init__("controller_node")

        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("trajectory_duration", 2.0)
        self.declare_parameter("control_mode", "position")
        self.declare_parameter("ik_position_tolerance", IK_POSITION_TOLERANCE)

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.trajectory_duration = float(
            self.get_parameter("trajectory_duration").value
        )
        self.control_mode = str(self.get_parameter("control_mode").value)
        self.ik_position_tolerance = float(
            self.get_parameter("ik_position_tolerance").value
        )

        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        if self.trajectory_duration < 0.0:
            raise ValueError("trajectory_duration must be non-negative")

        self.arm = PlanarArm(LINK_LENGTHS)
        self.backend: RobotBackend = SimulationBackend(JOINT_NAMES, INITIAL_JOINTS)

        self._trajectory: Optional[QuinticJointTrajectory] = None
        self._trajectory_start = None
        self._busy = False

        self._joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self._move_srv = self.create_service(
            MoveToTarget,
            "/move_to_target",
            self._on_move_to_target,
        )

        period = 1.0 / self.publish_rate_hz
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            "controller_node started "
            f"(rate={self.publish_rate_hz:.1f} Hz, "
            f"T={self.trajectory_duration:.2f} s, "
            f"mode={self.control_mode}, backend=SimulationBackend)"
        )

    def _on_move_to_target(
        self,
        request: MoveToTarget.Request,
        response: MoveToTarget.Response,
    ) -> MoveToTarget.Response:
        success, message = self._handle_cartesian_target(request.x, request.y)
        response.success = success
        response.message = message
        if success:
            self.get_logger().info(message)
        else:
            self.get_logger().warn(message)
        return response

    def _handle_cartesian_target(self, x: float, y: float) -> tuple[bool, str]:
        """Validate, plan, and arm a trajectory. Returns promptly."""
        if self._busy:
            return (
                False,
                "Controller is currently executing another trajectory; "
                "command rejected.",
            )

        if not math.isfinite(x) or not math.isfinite(y):
            return False, f"Malformed target ({x}, {y}): coordinates must be finite."

        self.get_logger().info(f"Received target ({x:.3f}, {y:.3f})")

        current_q = self.backend.get_positions()
        original = (x, y)
        planned_xy, projected = self._workspace_projection(original)
        if projected:
            self.get_logger().warn(
                f"Target ({original[0]:.3f}, {original[1]:.3f}) is outside "
                f"the maximum workspace (max reach "
                f"{sum(self.arm.link_lengths):.3f}). "
                f"Using closest reachable target "
                f"({planned_xy[0]:.3f}, {planned_xy[1]:.3f})."
            )

        try:
            target_q = self.arm.inverse_kinematics(
                planned_xy, initial_guess=current_q
            )
        except Exception as exc:  # noqa: BLE001 — keep the node alive
            self.get_logger().error(f"IK failed: {exc}")
            return False, f"IK failure: {exc}"

        if target_q is None or len(target_q) != self.arm.n_joints:
            return False, "IK returned no valid joint configuration."

        self.get_logger().info(
            "IK selected target joints (rad): "
            + ", ".join(f"{q:.4f}" for q in target_q)
        )

        if not self.arm.within_joint_limits(target_q):
            return False, "IK solution violates joint limits."
        if not self.arm.arm_above_base(target_q):
            return False, "IK solution violates the ground constraint (y >= 0)."

        achieved = self.arm.end_effector(target_q)
        ee_error = math.hypot(
            achieved[0] - planned_xy[0], achieved[1] - planned_xy[1]
        )
        if ee_error > self.ik_position_tolerance:
            return (
                False,
                "No valid IK solution: end-effector error "
                f"{ee_error:.4f} exceeds tolerance "
                f"{self.ik_position_tolerance:.4f}.",
            )

        try:
            trajectory = QuinticJointTrajectory(
                current_q, target_q, self.trajectory_duration
            )
        except ValueError as exc:
            return False, f"Trajectory generation failure: {exc}"

        invalid = self._first_invalid_sample(trajectory)
        if invalid is not None:
            t_bad, reason = invalid
            return (
                False,
                f"Invalid trajectory at t={t_bad:.3f}s: {reason}",
            )

        self._trajectory = trajectory
        self._trajectory_start = self.get_clock().now()
        self._busy = True
        self.get_logger().info(
            "Trajectory start: "
            f"{self._format_q(current_q)} -> {self._format_q(target_q)} "
            f"in {self.trajectory_duration:.2f}s"
            + (" (workspace-projected target)" if projected else "")
        )
        return True, self._success_message(original, planned_xy, projected, target_q)

    def _workspace_projection(self, target_xy: tuple[float, float]) -> tuple[tuple[float, float], bool]:
        """Project onto the maximum-reach circle if the target is too far."""
        x, y = target_xy
        max_reach = sum(self.arm.link_lengths)
        distance = math.hypot(x, y)
        if distance <= max_reach or distance == 0.0:
            return target_xy, False
        scale = max_reach / distance
        return (x * scale, y * scale), True

    def _first_invalid_sample(
        self,
        trajectory: QuinticJointTrajectory,
    ) -> Optional[tuple[float, str]]:
        """Return (t, reason) for the first unsafe sample, if any."""
        samples = max(TRAJECTORY_CHECK_SAMPLES, 1)
        duration = trajectory.duration
        for i in range(samples + 1):
            t = duration * i / samples if duration > 0.0 else 0.0
            q, _qd = trajectory.sample(t)
            if not self.arm.within_joint_limits(q):
                return t, "joint-limit violation"
            if not self.arm.arm_above_base(q):
                return t, "ground-constraint violation"
        return None

    def _on_timer(self) -> None:
        if self._busy and self._trajectory is not None:
            now = self.get_clock().now()
            elapsed = (now - self._trajectory_start).nanoseconds * 1e-9
            q, qd = self._trajectory.sample(elapsed)

            if not self.arm.within_joint_limits(q) or not self.arm.arm_above_base(q):
                self.get_logger().error(
                    "Aborting trajectory: intermediate configuration "
                    "violates joint limits or the ground constraint."
                )
                self.backend.command_joints(
                    self.backend.get_positions(),
                    [0.0] * self.arm.n_joints,
                )
                self._clear_trajectory()
            elif self._trajectory.is_finished(elapsed):
                final_q = self._trajectory.qf
                self.backend.command_joints(final_q, [0.0] * self.arm.n_joints)
                ee = self.arm.end_effector(final_q)
                self.get_logger().info(
                    "Trajectory complete. "
                    f"q={self._format_q(final_q)} "
                    f"ee=({ee[0]:.3f}, {ee[1]:.3f})"
                )
                self._clear_trajectory()
            else:
                self.backend.command_joints(q, qd)

        self._publish_joint_state()

    def _clear_trajectory(self) -> None:
        self._trajectory = None
        self._trajectory_start = None
        self._busy = False

    def _publish_joint_state(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.backend.joint_names()
        msg.position = self.backend.get_positions()
        msg.velocity = self.backend.get_velocities()
        self._joint_pub.publish(msg)

    @staticmethod
    def _format_q(q: list[float]) -> str:
        return "[" + ", ".join(f"{qi:.4f}" for qi in q) + "]"

    @staticmethod
    def _success_message(
        original: tuple[float, float],
        planned: tuple[float, float],
        projected: bool,
        target_q: list[float],
    ) -> str:
        if projected:
            return (
                f"Accepted projected target ({planned[0]:.3f}, {planned[1]:.3f}) "
                f"for original ({original[0]:.3f}, {original[1]:.3f}); "
                f"q={ControllerNode._format_q(target_q)}"
            )
        return (
            f"Accepted target ({original[0]:.3f}, {original[1]:.3f}); "
            f"q={ControllerNode._format_q(target_q)}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControllerNode()
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
