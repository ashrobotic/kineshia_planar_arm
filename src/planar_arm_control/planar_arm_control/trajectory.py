"""Smooth joint-space trajectory generation.

A quintic polynomial in normalized time gives zero velocity and acceleration
at the start and end of the motion:

    q(t) = q0 + (qf - q0) * (10s^3 - 15s^4 + 6s^5)
    s = clamp(t / T, 0, 1)
"""

from __future__ import annotations

from typing import Sequence


class QuinticJointTrajectory:
    """Time-parameterized quintic interpolation from q0 to qf."""

    def __init__(
        self,
        q0: Sequence[float],
        qf: Sequence[float],
        duration: float,
    ) -> None:
        if len(q0) != len(qf):
            raise ValueError("q0 and qf must have the same length")
        if duration < 0.0:
            raise ValueError("duration must be non-negative")

        self.q0 = [float(q) for q in q0]
        self.qf = [float(q) for q in qf]
        self.duration = float(duration)
        self.dof = len(self.q0)
        self.delta = [qf_i - q0_i for q0_i, qf_i in zip(self.q0, self.qf)]

    def is_finished(self, t: float) -> bool:
        """Return True when t has reached or passed the trajectory duration."""
        return t >= self.duration

    def sample(self, t: float) -> tuple[list[float], list[float]]:
        """Return (positions, velocities) at time t seconds from start."""
        if self.duration <= 0.0:
            return list(self.qf), [0.0] * self.dof

        s = max(0.0, min(1.0, t / self.duration))
        s2 = s * s
        s3 = s2 * s
        s4 = s3 * s
        s5 = s4 * s

        pos_blend = 10.0 * s3 - 15.0 * s4 + 6.0 * s5
        # d/dt of the blend: (30s^2 - 60s^3 + 30s^4) / T
        vel_blend = (30.0 * s2 - 60.0 * s3 + 30.0 * s4) / self.duration

        positions = [q0 + d * pos_blend for q0, d in zip(self.q0, self.delta)]
        velocities = [d * vel_blend for d in self.delta]
        return positions, velocities
