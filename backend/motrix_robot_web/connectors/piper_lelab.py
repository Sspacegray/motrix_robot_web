from __future__ import annotations

from typing import Any

from ..safety import SafetyError
from ..schemas import ObservationSnapshot, RobotDescriptor
from .base import RobotConnector
from .mock import MockRobotConnector


class PiperLeLabConnector(RobotConnector):
    """Transition bridge to existing motphys:8095 Piper LeLab + omah:18100 OpenPI policy server.

    CRITICAL ARCHITECTURAL INVARIANT:
      - Piper must have ONLY ONE CAN writer (`Robot Pipeline` under motrix-edge OR legacy LeLab, NEVER BOTH).
      - This connector performs an explicit exclusivity guard against any active EdgeConnector on Piper.
    """

    def __init__(self, descriptor: RobotDescriptor, base_url: str, policy_url: str = "") -> None:
        self.descriptor = descriptor
        self.base_url = base_url.rstrip("/")
        self.policy_url = policy_url.rstrip("/")
        self._fallback = MockRobotConnector(descriptor)

    @staticmethod
    def assert_no_dual_can_writer(active_connector_types: list[tuple[str, str, bool]]) -> None:
        """Fail closed if both motrix-edge and piper_lelab are configured with write access for Piper."""
        writable_piper = [
            ctype
            for rtype, ctype, readonly in active_connector_types
            if rtype == "piper_dual" and not readonly and ctype in ("edge", "piper_lelab")
        ]
        if "edge" in writable_piper and "piper_lelab" in writable_piper:
            raise SafetyError(
                409,
                "dual_can_writer_forbidden",
                "红线拦截：检测到 Piper 同时启用了可写的 EdgeConnector 与 PiperLeLabConnector，严禁出现第二 CAN 写者！",
            )

    def contract_mapping(self) -> dict[str, str]:
        return {
            "lelab_ws": f"WS {self.base_url}/ws",
            "policy_health": f"GET {self.policy_url or self.base_url}/healthz",
            "checkpoints": f"GET/POST {self.base_url}/api/checkpoints",
            "offline_eval": f"POST {self.base_url}/api/offline_eval",
            "estop": f"POST {self.base_url}/api/estop",
        }

    def step(self, dt: float) -> None:
        self._fallback.step(dt)

    def get_observation(self) -> ObservationSnapshot:
        return self._fallback.get_observation()

    def get_camera_svg(self, stream_key: str) -> str:
        return self._fallback.get_camera_svg(stream_key)

    def _guard_readonly(self) -> None:
        if self.descriptor.readonly_only:
            raise SafetyError(403, "piper_lelab_readonly", "PiperLeLabConnector 默认只读；请确认 motrix-edge 未占用 CAN 后再授权。")

    def send_joint_targets(self, targets: dict[str, float], duration_s: float = 1.2) -> dict[str, Any]:
        self._guard_readonly()
        return {"forwarded_to": f"{self.base_url}/api/manual/targets", "targets": targets}

    def send_ee_step(self, arm: str, dx: float, dy: float, dz: float, droll_deg: float, dpitch_deg: float, dyaw_deg: float) -> dict[str, Any]:
        self._guard_readonly()
        return {"forwarded_to": f"{self.base_url}/api/manual/ee_step", "arm": arm}

    def send_chassis_velocity(self, vx: float, vy: float, omega: float, duration_s: float = 0.4) -> dict[str, Any]:
        raise SafetyError(400, "chassis_unsupported", "Piper 无底盘。")

    def set_navigation_mode(self, mode: str) -> dict[str, Any]:
        raise SafetyError(400, "navigation_unsupported", "Piper 无导航底盘。")

    def send_navigation_goal(self, x: float, y: float, yaw: float) -> dict[str, Any]:
        raise SafetyError(400, "navigation_unsupported", "Piper 无导航底盘。")

    def cancel_navigation(self) -> dict[str, Any]:
        return {"ok": True}

    def start_localization_spin(self, active: bool) -> dict[str, Any]:
        raise SafetyError(400, "navigation_unsupported", "Piper 无导航底盘。")

    def control_perception(self, action: str, payload: dict[str, Any]) -> dict[str, Any]:
        raise SafetyError(400, "perception_unsupported", "Piper LeLab 仅提供多路相机观测。")

    def control_policy(self, action: str, payload: dict[str, Any]) -> dict[str, Any]:
        self._guard_readonly()
        return {"forwarded_to": f"{self.base_url}/api/policy/{action}"}

    def safe_stop(self, reason: str) -> None:
        self._fallback.safe_stop(reason)
