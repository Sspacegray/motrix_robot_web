from __future__ import annotations

import math
import time
import uuid
from datetime import datetime, timezone
from typing import Callable

from .schemas import AuthorityState, ObservationSnapshot, RobotDescriptor, TaskPermission


def utc_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


class SafetyError(Exception):
    def __init__(self, status_code: int, code: str, message: str):
        super().__init__(message)
        self.status_code = status_code
        self.code = code
        self.message = message


class RobotSafetyGate:
    """High-level safety gate & audit layer (never replaces Edge's underlying ControlArbiter).

    Enforces:
      - Separate high-level task permission vs underlying Edge Lease
      - Default read-only startup (zero motion without active lease)
      - 3s operator heartbeat watchdog + 300s idle auto-disarm
      - Full joint limit + NaN/Inf + active block interlock checks
      - Lease-free global E-Stop bypass with audit trail
      - Arms-only teach constraint (torso/chassis/head never released)
    """

    def __init__(
        self,
        descriptor: RobotDescriptor,
        hb_timeout_s: float = 3.0,
        idle_timeout_s: float = 300.0,
        lease_ttl_s: float = 120.0,
    ) -> None:
        self.on_timeout: Callable[[str], None] | None = None
        self.descriptor = descriptor
        self.hb_timeout_s = hb_timeout_s
        self.idle_timeout_s = idle_timeout_s
        self.lease_ttl_s = lease_ttl_s

        self._task_permission: TaskPermission = "operator"
        self._operator_name: str = "operator@motphys"
        self._lease_active: bool = False
        self._lease_id: str | None = None
        self._lease_source: str = "none"
        self._lease_granted_mono: float = 0.0
        self._last_hb_mono: float = 0.0
        self._last_action_mono: float = 0.0

        self._estop_latched: bool = False
        self._estop_reason: str = ""
        self._estop_by: str = ""
        self._estop_at: str = ""

    def set_task_permission(self, permission: TaskPermission, operator_name: str | None = None) -> AuthorityState:
        self._task_permission = permission
        if operator_name:
            self._operator_name = operator_name
        # Downgrading to viewer immediately revokes control lease
        if permission == "viewer" and self._lease_active:
            self.release_lease()
        return self.snapshot()

    def acquire_lease(
        self,
        operator_name: str,
        task_permission: TaskPermission = "operator",
        confirm_safety: bool = False,
        lease_source: str = "edge_lease_proxy",
    ) -> AuthorityState:
        self.check_watchdogs()
        if self._lease_active:
            raise SafetyError(409, "lease_already_active", "已有活动租约，请先释放，不能静默接管。")
        if self.descriptor.readonly_only:
            raise SafetyError(403, "readonly_robot", f"机器人 {self.descriptor.name} 配置为硬只读模式，禁止获取写控租约。")
        if not confirm_safety:
            raise SafetyError(400, "confirm_required", "获取控制租约前必须勾选现场安全确认。")
        if task_permission == "viewer":
            raise SafetyError(403, "insufficient_task_permission", "高层任务权限为只读 (viewer)，无法申请底层控制租约。")
        if self._estop_latched:
            raise SafetyError(409, "estop_latched", "当前处于 E-Stop 锁存状态，请先排查原因并显式清除急停后再申请租约。")

        now = time.monotonic()
        self._task_permission = task_permission
        self._operator_name = operator_name or "operator@motphys"
        self._lease_active = True
        self._lease_id = f"lease-{uuid.uuid4().hex[:10]}"
        self._lease_source = lease_source
        self._lease_granted_mono = now
        self._last_hb_mono = now
        self._last_action_mono = now
        return self.snapshot()

    def release_lease(self) -> AuthorityState:
        self._lease_active = False
        self._lease_id = None
        self._lease_source = "none"
        return self.snapshot()

    def heartbeat(self, lease_id: str | None = None) -> AuthorityState:
        if self.check_watchdogs():
            raise SafetyError(409, "lease_expired", "租约已过期并停机，必须显式重新授权。")
        if not self._lease_active:
            raise SafetyError(403, "lease_required", "无活动租约，心跳不能恢复控制。")
        if not lease_id or lease_id != self._lease_id:
            raise SafetyError(409, "lease_mismatch", "租约 ID 不匹配，可能已被其他会话接管或已失效。")
        now = time.monotonic()
        self._last_hb_mono = now
        # Sliding renewal within lease TTL window
        self._lease_granted_mono = now
        return self.snapshot()

    def mark_activity(self) -> None:
        if self._lease_active:
            now = time.monotonic()
            self._last_action_mono = now

    def trigger_estop(self, source: str = "web_ui", reason: str = "Operator E-Stop") -> AuthorityState:
        """Always allowed regardless of lease or task permission (bypass queue)."""
        self._estop_latched = True
        self._estop_reason = reason
        self._estop_by = source
        self._estop_at = utc_iso()
        self._lease_active = False
        self._lease_id = None
        self._lease_source = "none"
        return self.snapshot()

    def clear_estop(self, operator_name: str = "operator@motphys") -> AuthorityState:
        if self._task_permission == "viewer":
            raise SafetyError(403, "viewer_cannot_clear_estop", "只读角色无权清除急停锁存。")
        self._estop_latched = False
        self._estop_reason = ""
        self._estop_by = f"cleared_by:{operator_name}"
        self._estop_at = utc_iso()
        return self.snapshot()

    def check_watchdogs(self, now: float | None = None, on_timeout: Callable[[str], None] | None = None) -> str | None:
        if not self._lease_active:
            return None
        t = time.monotonic() if now is None else now
        hb_age = t - self._last_hb_mono
        idle_age = t - self._last_action_mono
        lease_age = t - self._lease_granted_mono

        reason: str | None = None
        if hb_age > self.hb_timeout_s:
            reason = f"heartbeat_timeout ({hb_age:.1f}s > {self.hb_timeout_s:.1f}s)"
        elif idle_age > self.idle_timeout_s:
            reason = f"idle_timeout ({idle_age:.0f}s > {self.idle_timeout_s:.0f}s)"
        elif lease_age > self.lease_ttl_s:
            reason = f"lease_ttl_expired ({lease_age:.0f}s > {self.lease_ttl_s:.0f}s)"

        if reason:
            self.release_lease()
            callback = on_timeout or self.on_timeout
            if callback:
                callback(reason)
        return reason

    def require_control_authority(self, obs: ObservationSnapshot | None = None) -> None:
        self.check_watchdogs()
        if self.descriptor.readonly_only:
            raise SafetyError(403, "readonly_robot", "当前机器人处于硬只读模式。")
        if self._estop_latched:
            raise SafetyError(409, "estop_latched", "急停已锁存，禁止下发运动或控制指令。")
        if self._task_permission == "viewer":
            raise SafetyError(403, "viewer_readonly", "当前高层任务权限为只读 (viewer)。")
        if not self._lease_active or not self._lease_id:
            raise SafetyError(403, "lease_required", "未持有有效底层控制租约 (Lease)，请先在顶栏或安全栏授权控制。")
        if obs is not None:
            if not obs.connected:
                raise SafetyError(409, "disconnected", "无有效机器人连接。")
            if obs.feedback_age_ms > 1000.0:
                raise SafetyError(409, "stale_feedback", f"状态反馈陈旧 ({obs.feedback_age_ms:.0f} ms > 1000 ms)，安全门禁拒绝下发。")
            blocks = [it for it in obs.interlocks if it.severity == "block"]
            if blocks:
                msgs = "; ".join(f"{b.label}: {b.detail}" for b in blocks)
                raise SafetyError(409, "interlock_blocked", f"安全联锁阻断下发：{msgs}")
        self.mark_activity()

    def validate_joint_targets(self, targets: dict[str, float], obs: ObservationSnapshot) -> dict[str, float]:
        self.require_control_authority(obs)
        if not targets:
            raise SafetyError(400, "empty_targets", "目标关节列表为空。")
        spec_map = {j.name: j for j in self.descriptor.joints}
        validated: dict[str, float] = {}
        for name, val in targets.items():
            if name not in spec_map:
                raise SafetyError(400, "unknown_joint", f"未知关节名称：{name}")
            if not isinstance(val, (int, float)) or not math.isfinite(val):
                raise SafetyError(400, "invalid_number", f"关节 {name} 目标包含非法值 (NaN/Inf)。")
            spec = spec_map[name]
            if val < spec.lower - 1e-6 or val > spec.upper + 1e-6:
                raise SafetyError(
                    400,
                    "joint_limit_exceeded",
                    f"关节 {name} 目标 {val:.4f} 超出软限位 [{spec.lower:.3f}, {spec.upper:.3f}]。",
                )
            validated[name] = float(val)
        return validated

    def validate_teach_request(self, groups: list[str]) -> None:
        forbidden = [g for g in groups if g in ("torso", "chassis", "head", "leg")]
        if forbidden:
            raise SafetyError(
                403,
                "teach_arms_only",
                f"示教模式严格仅允许释放手臂，绝不释放躯干/底盘/头部/腿部：{', '.join(forbidden)}",
            )

    def snapshot(self, now: float | None = None) -> AuthorityState:
        t = time.monotonic() if now is None else now
        hb_age = max(0.0, t - self._last_hb_mono) if self._lease_active else 0.0
        idle_age = max(0.0, t - self._last_action_mono) if self._lease_active else 0.0
        remaining = max(0.0, self.lease_ttl_s - (t - self._lease_granted_mono)) if self._lease_active else 0.0
        return AuthorityState(
            robot_id=self.descriptor.id,
            readonly_only=self.descriptor.readonly_only,
            task_permission=self._task_permission,
            operator_name=self._operator_name,
            lease_active=self._lease_active,
            lease_id=self._lease_id,
            lease_source=self._lease_source,
            lease_ttl_s=self.lease_ttl_s,
            lease_remaining_s=round(remaining, 1),
            heartbeat_age_s=round(hb_age, 2),
            hb_timeout_s=self.hb_timeout_s,
            idle_age_s=round(idle_age, 1),
            idle_timeout_s=self.idle_timeout_s,
            estop_latched=self._estop_latched,
            estop_reason=self._estop_reason,
            estop_by=self._estop_by,
            estop_at=self._estop_at,
            teach_arms_only=True,
        )
