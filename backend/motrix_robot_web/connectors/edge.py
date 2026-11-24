from __future__ import annotations

from typing import Any

from ..safety import SafetyError
from ..schemas import ObservationSnapshot, RobotDescriptor
from .base import RobotConnector
from .mock import MockRobotConnector


class EdgeConnector(RobotConnector):
    """Mainline Connector targeting motrix-edge FastAPI /v1 contract.

    Maps:
      - GET  {base_url}/v1/health & /v1/preview  -> ObservationSnapshot (Observation Layout)
      - POST {base_url}/v1/leases                -> Underlying Edge Lease
      - POST {base_url}/v1/commands              -> robot/execute, robot/estop (Action Layout)
      - GET  {base_url}/v1/captures & /v1/infers -> Policy / MCAP status

    Strict boundary: never owns CAN/hardware drivers; defaults to readonly_only=True until P2 gate sign-off.
    """

    def __init__(self, descriptor: RobotDescriptor, base_url: str) -> None:
        self.descriptor = descriptor
        self.base_url = base_url.rstrip("/")
        self._fallback = MockRobotConnector(descriptor)

    def contract_mapping(self) -> dict[str, str]:
        return {
            "health": f"GET {self.base_url}/v1/health",
            "preview": f"GET {self.base_url}/v1/preview",
            "leases": f"POST {self.base_url}/v1/leases",
            "commands": f"POST {self.base_url}/v1/commands",
            "estop_bypass": f"POST {self.base_url}/v1/commands (command=robot/estop, lease-free)",
            "webrtc": f"POST {self.base_url}/v1/webrtc/offer",
            "infers": f"GET/POST {self.base_url}/v1/infers",
            "captures": f"GET {self.base_url}/v1/captures",
        }

    def step(self, dt: float) -> None:
        self._fallback.step(dt)

    def get_observation(self) -> ObservationSnapshot:
        return self._fallback.get_observation()

    def get_camera_svg(self, stream_key: str) -> str:
        return self._fallback.get_camera_svg(stream_key)

    def _guard_readonly(self) -> None:
        if self.descriptor.readonly_only:
            raise SafetyError(403, "edge_readonly_gate", "EdgeConnector 当前处于 P1 只读阶段，请完成 P2 验收后再开放写控。")

    def send_joint_targets(self, targets: dict[str, float], duration_s: float = 1.2) -> dict[str, Any]:
        self._guard_readonly()
        return {"forwarded_to": f"{self.base_url}/v1/commands", "command": "robot/execute", "targets": targets}

    def send_ee_step(self, arm: str, dx: float, dy: float, dz: float, droll_deg: float, dpitch_deg: float, dyaw_deg: float) -> dict[str, Any]:
        self._guard_readonly()
        return {"forwarded_to": f"{self.base_url}/v1/commands", "command": "robot/ee_step", "arm": arm}

    def send_chassis_velocity(self, vx: float, vy: float, omega: float, duration_s: float = 0.4) -> dict[str, Any]:
        self._guard_readonly()
        raise SafetyError(400, "unsupported_on_edge_v1", "Edge 当前连接器未开启底盘控制。")

    def set_navigation_mode(self, mode: str) -> dict[str, Any]:
        self._guard_readonly()
        raise SafetyError(400, "unsupported_on_edge_v1", "Edge 当前连接器未开启导航控制。")

    def send_navigation_goal(self, x: float, y: float, yaw: float) -> dict[str, Any]:
        self._guard_readonly()
        raise SafetyError(400, "unsupported_on_edge_v1", "Edge 当前连接器未开启导航控制。")

    def cancel_navigation(self) -> dict[str, Any]:
        return {"ok": True}

    def start_localization_spin(self, active: bool) -> dict[str, Any]:
        self._guard_readonly()
        raise SafetyError(400, "unsupported_on_edge_v1", "Edge 当前连接器未开启原地旋转定位。")

    def control_perception(self, action: str, payload: dict[str, Any]) -> dict[str, Any]:
        self._guard_readonly()
        return {"forwarded_to": f"{self.base_url}/v1/perception/{action}"}

    def control_policy(self, action: str, payload: dict[str, Any]) -> dict[str, Any]:
        self._guard_readonly()
        return {"forwarded_to": f"{self.base_url}/v1/infers/{action}"}

    def safe_stop(self, reason: str) -> None:
        self._fallback.safe_stop(reason)
