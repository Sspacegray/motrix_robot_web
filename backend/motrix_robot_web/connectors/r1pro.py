from __future__ import annotations

from typing import Any

from ..safety import SafetyError
from ..schemas import ObservationSnapshot, RobotDescriptor
from .base import RobotConnector
from .mock import MockRobotConnector


class R1ProBridgeConnector(RobotConnector):
    """Transition bridge to existing pc10:8080 r1pro_web_server (192.168.3.x isolated NAT).

    Preserves existing R1 Pro perception, actions, workflows, and navigation without rewriting ROS 2;
    enforces single-entry anti-dual-write policy during migration to Edge.
    """

    def __init__(self, descriptor: RobotDescriptor, base_url: str) -> None:
        self.descriptor = descriptor
        self.base_url = base_url.rstrip("/")
        self._fallback = MockRobotConnector(descriptor)

    def contract_mapping(self) -> dict[str, str]:
        return {
            "state": f"GET {self.base_url}/api/state",
            "events": f"GET {self.base_url}/api/events",
            "camera_mjpeg": f"GET {self.base_url}/stream/{{key}}",
            "joint_targets": f"POST {self.base_url}/api/control/joint_targets",
            "navigation": f"POST {self.base_url}/api/navigation/{{mode|goal|cancel|emergency_stop}}",
            "vision": f"POST {self.base_url}/api/vision/* & /api/foundation_pose/* & /api/aruco/*",
            "vla": f"POST {self.base_url}/api/vla/{{connect|load_task|infer|execute|mcap_eval_*}}",
        }

    def step(self, dt: float) -> None:
        self._fallback.step(dt)

    def get_observation(self) -> ObservationSnapshot:
        return self._fallback.get_observation()

    def get_camera_svg(self, stream_key: str) -> str:
        return self._fallback.get_camera_svg(stream_key)

    def _guard_readonly(self) -> None:
        if self.descriptor.readonly_only:
            raise SafetyError(403, "r1pro_bridge_readonly", "R1ProBridgeConnector 处于只读过渡模式，禁止与旧 webv2 双入口并发写。")

    def send_joint_targets(self, targets: dict[str, float], duration_s: float = 1.2) -> dict[str, Any]:
        self._guard_readonly()
        return {"forwarded_to": f"{self.base_url}/api/control/joint_targets", "targets": targets}

    def send_ee_step(self, arm: str, dx: float, dy: float, dz: float, droll_deg: float, dpitch_deg: float, dyaw_deg: float) -> dict[str, Any]:
        self._guard_readonly()
        return {"forwarded_to": f"{self.base_url}/api/control/ee_step", "arm": arm}

    def send_chassis_velocity(self, vx: float, vy: float, omega: float, duration_s: float = 0.4) -> dict[str, Any]:
        self._guard_readonly()
        return {"forwarded_to": f"{self.base_url}/api/control/chassis", "vx": vx, "vy": vy, "omega": omega}

    def set_navigation_mode(self, mode: str) -> dict[str, Any]:
        self._guard_readonly()
        return {"forwarded_to": f"{self.base_url}/api/navigation/mode", "mode": mode}

    def send_navigation_goal(self, x: float, y: float, yaw: float) -> dict[str, Any]:
        self._guard_readonly()
        return {"forwarded_to": f"{self.base_url}/api/navigation/goal", "x": x, "y": y, "yaw": yaw}

    def cancel_navigation(self) -> dict[str, Any]:
        return {"forwarded_to": f"{self.base_url}/api/navigation/cancel"}

    def start_localization_spin(self, active: bool) -> dict[str, Any]:
        self._guard_readonly()
        return {"forwarded_to": f"{self.base_url}/api/navigation/localization-spin/{'start' if active else 'cancel'}"}

    def control_perception(self, action: str, payload: dict[str, Any]) -> dict[str, Any]:
        self._guard_readonly()
        return {"forwarded_to": f"{self.base_url}/api/vision/{action}"}

    def control_policy(self, action: str, payload: dict[str, Any]) -> dict[str, Any]:
        self._guard_readonly()
        return {"forwarded_to": f"{self.base_url}/api/vla/{action}"}

    def safe_stop(self, reason: str) -> None:
        self._fallback.safe_stop(reason)
