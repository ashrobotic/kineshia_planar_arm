"""Joint-level backend abstraction for the planar arm controller.

Planning and IK stay independent of how joint commands are realized.
SimulationBackend is the only implementation required for this assignment;
a future DynamixelBackend can implement the same interface.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Optional, Sequence


class RobotBackend(ABC):
    """Hardware-independent joint command/state interface."""

    @abstractmethod
    def joint_names(self) -> list[str]:
        """Return ordered joint names matching position vectors."""

    @abstractmethod
    def get_positions(self) -> list[float]:
        """Return the current joint positions in radians."""

    @abstractmethod
    def get_velocities(self) -> list[float]:
        """Return the current joint velocities in radians per second."""

    @abstractmethod
    def command_joints(
        self,
        positions: Sequence[float],
        velocities: Optional[Sequence[float]] = None,
    ) -> None:
        """Command a joint configuration.

        Simulation applies this immediately. A hardware backend would send
        the same command to actuators (or a tracking loop) instead.
        """


class SimulationBackend(RobotBackend):
    """In-memory joint state used for the simulation-only assignment."""

    def __init__(
        self,
        joint_names: Sequence[str],
        initial_positions: Sequence[float],
    ) -> None:
        if len(joint_names) != len(initial_positions):
            raise ValueError("joint_names and initial_positions must match")
        self._joint_names = list(joint_names)
        self._positions = [float(q) for q in initial_positions]
        self._velocities = [0.0] * len(self._joint_names)

    def joint_names(self) -> list[str]:
        return list(self._joint_names)

    def get_positions(self) -> list[float]:
        return list(self._positions)

    def get_velocities(self) -> list[float]:
        return list(self._velocities)

    def command_joints(
        self,
        positions: Sequence[float],
        velocities: Optional[Sequence[float]] = None,
    ) -> None:
        if len(positions) != len(self._joint_names):
            raise ValueError(
                f"Expected {len(self._joint_names)} positions, got {len(positions)}"
            )
        self._positions = [float(q) for q in positions]
        if velocities is None:
            self._velocities = [0.0] * len(self._positions)
            return
        if len(velocities) != len(self._joint_names):
            raise ValueError(
                f"Expected {len(self._joint_names)} velocities, got {len(velocities)}"
            )
        self._velocities = [float(qd) for qd in velocities]
