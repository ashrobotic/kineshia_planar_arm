"""
planar_arm.py  —  PROVIDED KINEMATICS.  DO NOT MODIFY.

3-DoF planar (2D) revolute arm used for the Kineshia Robotics take-home task.
You are given a complete, working kinematics library so you can focus on the
ROS 2 architecture, control, and GUI — not on re-deriving trigonometry.

Public API you will use:
    arm = PlanarArm(link_lengths=[3.0, 2.0, 1.5])
    arm.n_joints                      -> 3
    arm.joint_limits                  -> np.ndarray shape (3, 2), radians
    arm.forward_kinematics(q)         -> list[(x, y)] for base..end-effector
    arm.end_effector(q)               -> (x, y) of the tool tip
    arm.inverse_kinematics(target_xy, initial_guess=q) -> list[q1, q2, q3]
    arm.within_joint_limits(q)        -> bool
    arm.arm_above_base(q)             -> bool  (no link below y = 0)

Angles are RELATIVE joint angles in radians. q1 is measured from +x.
"""

import numpy as np


class PlanarArm:

    def __init__(self, link_lengths):
        self.link_lengths = list(link_lengths)
        self.n_joints = len(self.link_lengths)

        # Per-joint (lower, upper) limits in radians.
        self.joint_limits = np.radians([
            [0, 180],
            [-120, 120],
            [-120, 120],
        ])

        # Ground constraint: no part of the arm may go below y = 0.
        self.min_y = 0.0

    # ------------------------------------------------------------------
    # Constraints
    # ------------------------------------------------------------------
    def within_joint_limits(self, q):
        q = np.asarray(q, dtype=float)
        for i in range(self.n_joints):
            lower = self.joint_limits[i][0]
            upper = self.joint_limits[i][1]
            if q[i] < lower or q[i] > upper:
                return False
        return True

    def arm_above_base(self, q, tolerance=1e-6):
        for x, y in self.forward_kinematics(q):
            if y < self.min_y - tolerance:
                return False
        return True

    # ------------------------------------------------------------------
    # Forward kinematics
    # ------------------------------------------------------------------
    def forward_kinematics(self, joint_angles):
        if len(joint_angles) != self.n_joints:
            raise ValueError(
                f"Expected {self.n_joints} joint angles, got {len(joint_angles)}"
            )

        x = 0.0
        y = 0.0
        theta = 0.0
        points = [(x, y)]

        for length, angle in zip(self.link_lengths, joint_angles):
            theta += angle
            x += length * np.cos(theta)
            y += length * np.sin(theta)
            points.append((x, y))

        return points

    def end_effector(self, joint_angles):
        return self.forward_kinematics(joint_angles)[-1]

    @staticmethod
    def wrap_angle(angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    # ------------------------------------------------------------------
    # Numerical Jacobian
    # ------------------------------------------------------------------
    def numerical_jacobian(self, joint_angles, delta=1e-6):
        q = np.asarray(joint_angles, dtype=float)
        if len(q) != self.n_joints:
            raise ValueError(
                f"Expected {self.n_joints} joint angles, got {len(q)}"
            )

        current_position = np.asarray(self.end_effector(q), dtype=float)
        J = np.zeros((2, self.n_joints))

        for i in range(self.n_joints):
            q_test = q.copy()
            q_test[i] += delta
            new_position = np.asarray(self.end_effector(q_test), dtype=float)
            J[:, i] = (new_position - current_position) / delta

        return J

    # ------------------------------------------------------------------
    # Reachable target (projects onto workspace boundary if too far)
    # ------------------------------------------------------------------
    def reachable_target(self, target_xy):
        target = np.asarray(target_xy, dtype=float)
        max_reach = sum(self.link_lengths)
        distance = np.linalg.norm(target)

        if distance <= max_reach:
            return target
        if distance == 0:
            return target

        reachable = target * max_reach / distance
        print(f"Target {tuple(target_xy)} is outside workspace.")
        print(
            f"Using closest reachable target: "
            f"({reachable[0]:.3f}, {reachable[1]:.3f})"
        )
        return reachable

    # ------------------------------------------------------------------
    # Multiple-solution analytical IK (with Jacobian fallback)
    # ------------------------------------------------------------------
    def inverse_kinematics(self, target_xy, initial_guess=None,
                           q3_samples=361, tolerance=1e-5):

        target = self.reachable_target(target_xy)

        if self.n_joints != 3:
            raise ValueError(
                "This multiple-solution IK implementation expects exactly 3 joints."
            )

        L1, L2, L3 = self.link_lengths

        if initial_guess is None:
            current_q = np.zeros(3)
        else:
            current_q = np.asarray(initial_guess, dtype=float).copy()
            if len(current_q) != 3:
                raise ValueError("initial_guess must contain 3 angles")

        target_distance = np.linalg.norm(target)
        if target_distance < tolerance:
            return current_q.tolist()

        target_angle = np.arctan2(target[1], target[0])
        solutions = []
        q3_values = np.linspace(-np.pi, np.pi, q3_samples)

        for q3 in q3_values:
            A = L2 + L3 * np.cos(q3)
            B = L3 * np.sin(q3)
            R = np.sqrt(A ** 2 + B ** 2)
            beta = np.arctan2(B, A)

            cosine_value = (
                target_distance ** 2 - L1 ** 2 - R ** 2
            ) / (2 * L1 * R)

            if cosine_value < -1.0:
                if cosine_value < -1.000001:
                    continue
                cosine_value = -1.0
            if cosine_value > 1.0:
                if cosine_value > 1.000001:
                    continue
                cosine_value = 1.0

            delta = np.arccos(cosine_value)

            for elbow_sign in [1, -1]:
                delta_signed = elbow_sign * delta
                correction = np.arctan2(
                    R * np.sin(delta_signed),
                    L1 + R * np.cos(delta_signed),
                )
                q1 = target_angle - correction
                theta2 = q1 + delta_signed
                q2 = theta2 - q1
                q3_candidate = q3

                candidate = np.array([
                    self.wrap_angle(q1),
                    self.wrap_angle(q2),
                    self.wrap_angle(q3_candidate),
                ])

                if not self.within_joint_limits(candidate):
                    continue
                if not self.arm_above_base(candidate):
                    continue

                actual_position = np.asarray(self.end_effector(candidate))
                position_error = np.linalg.norm(target - actual_position)
                if position_error > tolerance:
                    continue

                angle_difference = candidate - current_q
                angle_difference = np.array([
                    self.wrap_angle(x) for x in angle_difference
                ])
                movement_cost = np.sum(np.abs(angle_difference))

                solutions.append({
                    "q": candidate,
                    "error": position_error,
                    "movement": movement_cost,
                })

        if len(solutions) == 0:
            print("No analytical IK solution found.")
            print("Falling back to Jacobian IK.")
            return self.jacobian_ik(target, current_q)

        # Prefer the solution requiring the least joint movement.
        solutions.sort(key=lambda solution: solution["movement"])
        best_solution = solutions[0]
        return best_solution["q"].tolist()

    # ------------------------------------------------------------------
    # Damped-least-squares Jacobian IK (fallback only)
    # ------------------------------------------------------------------
    def jacobian_ik(self, target, initial_guess, max_iterations=1000,
                    tolerance=1e-5, damping=0.15, step_size=0.5,
                    max_joint_step=0.08):

        q = np.asarray(initial_guess, dtype=float).copy()
        best_q = q.copy()
        best_error = float("inf")

        for _ in range(max_iterations):
            current_position = np.asarray(self.end_effector(q))
            error = target - current_position
            error_norm = np.linalg.norm(error)

            if error_norm < best_error:
                best_error = error_norm
                best_q = q.copy()
            if error_norm < tolerance:
                return q.tolist()

            J = self.numerical_jacobian(q)
            I = np.eye(2)
            J_damped = J.T @ np.linalg.inv(J @ J.T + damping ** 2 * I)
            dq = J_damped @ error

            dq_norm = np.linalg.norm(dq)
            if dq_norm > max_joint_step:
                dq *= max_joint_step / dq_norm

            q += step_size * dq
            q = np.array([self.wrap_angle(x) for x in q])

        return best_q.tolist()


if __name__ == "__main__":
    arm = PlanarArm([3.0, 2.0, 1.5])
    current_angles = [np.radians(30), np.radians(-20), np.radians(-10)]
    print("End effector:", arm.end_effector(current_angles))
    solution = arm.inverse_kinematics((4.0, 2.0), initial_guess=current_angles)
    print("IK solution (deg):", [f"{np.degrees(a):.2f}" for a in solution])
    print("Achieved:", [f"{v:.3f}" for v in arm.end_effector(solution)])
