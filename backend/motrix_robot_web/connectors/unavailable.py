"""Fail-closed adapter scaffolds: no network, synthetic telemetry or command delivery claims."""
from __future__ import annotations
import time
from typing import Any
from ..schemas import ObservationSnapshot, InterlockItem, PolicyState
from ..safety import SafetyError
from .base import RobotConnector


class UnavailableConnector(RobotConnector):
    def step(self, dt: float) -> None:
        pass

    def get_observation(self) -> ObservationSnapshot:
        return ObservationSnapshot(
            robot_id=self.descriptor.id, timestamp=time.time(), mode=self.descriptor.mode,
            telemetry_source="unavailable", connected=False, feedback_age_ms=1e9, measured_hz=0,
            joint_positions={}, joint_targets={}, joint_velocities={}, joint_efforts={},
            policy=PolicyState(connected=False, endpoint="unimplemented"),
            interlocks=[InterlockItem(code="connector_unimplemented", label="未实现真实接入",
                severity="block", detail="无遥测、无 Edge 鉴权/Lease、无命令或停机送达确认。")],
        )

    def _deny(self, *args: Any, **kwargs: Any) -> Any:
        if self.descriptor.connector_type == "dex_w1":
            raise SafetyError(403, "dex_w1_readonly_only", "W1 适配器严格只读且未实现遥测接入。")
        raise SafetyError(501, "connector_unimplemented", "适配器未实现；未发送任何请求，不能确认远程执行或停机。")

    get_camera_svg = _deny
    send_joint_targets = _deny
    send_ee_step = _deny
    send_chassis_velocity = _deny
    set_navigation_mode = _deny
    send_navigation_goal = _deny
    cancel_navigation = _deny
    start_localization_spin = _deny
    control_perception = _deny
    control_policy = _deny
    inject_fault = _deny

    def safe_stop(self, reason: str) -> None:
        # Local lease/latch can be revoked, but there is no remote transport to stop.
        self.last_local_stop_reason = reason
