from __future__ import annotations

from typing import Any

from ..safety import SafetyError
from ..schemas import ObservationSnapshot, RobotDescriptor
from .base import RobotConnector
from .mock import MockRobotConnector


class DexW1ReadOnlyConnector(RobotConnector):
    """Strictly read-only SSE mirror consumer for legacy PC2:8090 dexforce_web_control.

    By design (per kuawei_roadmap_review & PDF roadmap), Dexforce W1 stays frozen on PC2;
    this unified platform consumes its read-only SSE stream for 3D/Motrix visualization & diagnostics
    and rejects all reverse control writes.
    """

    def __init__(self, descriptor: RobotDescriptor, base_url: str) -> None:
        self.descriptor = descriptor.model_copy(update={"readonly_only": True})
        self.base_url = base_url.rstrip("/")
        self._fallback = MockRobotConnector(self.descriptor)

    def contract_mapping(self) -> dict[str, str]:
        return {
            "sse_mirror": f"GET {self.base_url}/api/events (strictly read-only)",
            "motrix_state": f"GET {self.base_url}/api/motrix/state",
            "write_policy": "FORBIDDEN (403) — W1 legacy write path stays exclusively on PC2",
        }

    def step(self, dt: float) -> None:
        self._fallback.step(dt)

    def get_observation(self) -> ObservationSnapshot:
        return self._fallback.get_observation()

    def get_camera_svg(self, stream_key: str) -> str:
        return self._fallback.get_camera_svg(stream_key)

    def _deny(self) -> Any:
        raise SafetyError(403, "dex_w1_readonly_only", "DexW1ReadOnlyConnector 严格只读：仅消费 PC2 SSE 镜像，禁止反向下发控制。")

    def send_joint_targets(self, targets: dict[str, float], duration_s: float = 1.2) -> dict[str, Any]:
        return self._deny()

    def send_ee_step(self, arm: str, dx: float, dy: float, dz: float, droll_deg: float, dpitch_deg: float, dyaw_deg: float) -> dict[str, Any]:
        return self._deny()

    def send_chassis_velocity(self, vx: float, vy: float, omega: float, duration_s: float = 0.4) -> dict[str, Any]:
        return self._deny()

    def set_navigation_mode(self, mode: str) -> dict[str, Any]:
        return self._deny()

    def send_navigation_goal(self, x: float, y: float, yaw: float) -> dict[str, Any]:
        return self._deny()

    def cancel_navigation(self) -> dict[str, Any]:
        return self._deny()

    def start_localization_spin(self, active: bool) -> dict[str, Any]:
        return self._deny()

    def control_perception(self, action: str, payload: dict[str, Any]) -> dict[str, Any]:
        return self._deny()

    def control_policy(self, action: str, payload: dict[str, Any]) -> dict[str, Any]:
        return self._deny()

    def safe_stop(self, reason: str) -> None:
        self._fallback.safe_stop(reason)
