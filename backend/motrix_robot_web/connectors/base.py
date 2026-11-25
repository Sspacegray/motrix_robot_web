from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

from ..schemas import ObservationSnapshot, RobotDescriptor


class RobotConnector(ABC):
    """Unified high-level connector interface aligned to Edge Observation/Action Layout.

    All hardware writes are delegated through the underlying Edge/Bridge system;
    this high-level platform never owns hardware CAN/ROS drivers directly.
    """

    descriptor: RobotDescriptor

    @abstractmethod
    def step(self, dt: float) -> None:
        """Advance internal mock simulation or refresh cached telemetry."""

    @abstractmethod
    def get_observation(self) -> ObservationSnapshot:
        """Return latest canonical observation snapshot."""

    @abstractmethod
    def get_camera_svg(self, stream_key: str) -> str:
        """Return an annotated camera frame (SVG in mock mode or proxy placeholder)."""

    @abstractmethod
    def send_joint_targets(self, targets: dict[str, float], duration_s: float = 1.2) -> dict[str, Any]:
        """Forward validated joint targets."""

    @abstractmethod
    def send_ee_step(self, arm: str, dx: float, dy: float, dz: float, droll_deg: float, dpitch_deg: float, dyaw_deg: float) -> dict[str, Any]:
        """Forward 6D end-effector step."""

    @abstractmethod
    def send_chassis_velocity(self, vx: float, vy: float, omega: float, duration_s: float = 0.4) -> dict[str, Any]:
        """Forward chassis velocity command (for mobile-base robots)."""

    @abstractmethod
    def set_navigation_mode(self, mode: str) -> dict[str, Any]:
        """Switch MANUAL / NAVIGATION / FAULT authority."""

    @abstractmethod
    def send_navigation_goal(
        self, x: float, y: float, yaw: float, path_points: list[list[float]] | None = None
    ) -> dict[str, Any]:
        """Send 2D navigation goal in map frame."""

    @abstractmethod
    def cancel_navigation(self) -> dict[str, Any]:
        """Cancel active navigation goal."""

    @abstractmethod
    def start_localization_spin(self, active: bool) -> dict[str, Any]:
        """Start or cancel in-place localization spin."""

    @abstractmethod
    def control_perception(self, action: str, payload: dict[str, Any]) -> dict[str, Any]:
        """Execute high-level perception command (head_depth, yolo, latch, foundation_pose, aruco, pick_3d)."""

    @abstractmethod
    def control_policy(self, action: str, payload: dict[str, Any]) -> dict[str, Any]:
        """Execute policy intent command (switch_checkpoint, wizard_step, start_infer, stop_infer, offline_eval, tag_episode)."""

    @abstractmethod
    def safe_stop(self, reason: str) -> None:
        """Immediate software safe-stop (hold position / zero chassis vel / stop policy)."""

    def inject_fault(self, fault_type: str) -> dict[str, Any]:
        return {"ok": False, "message": "Fault injection is only supported in Mock mode."}
