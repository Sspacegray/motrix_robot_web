from __future__ import annotations

"""Strictly read-only Motrix WASM仿真镜像桥 (isolated from all robot write drivers).

Architectural invariant (verified by tests/test_connectors.py):
  - Never imports hardware drivers (rclpy, pyAgxArm, can, ros2)
  - Never holds a reference to any write/control command API
  - Rejects any incoming WebSocket command that attempts to write robot joints
"""

import asyncio
import json
import sys
import time
from typing import Any

from fastapi import WebSocket

FORBIDDEN_DRIVER_MODULES = frozenset({"rclpy", "pyAgxArm", "can", "motrix_edge.control"})


class MotrixReadOnlyBridge:
    """Broadcasts read-only joint observations to connected Motrix WASM viewers."""

    READ_ONLY = True

    def __init__(self) -> None:
        self._clients: set[WebSocket] = set()
        self._rejected_writes: int = 0
        self._last_broadcast_ts: float = 0.0

    @property
    def client_count(self) -> int:
        return len(self._clients)

    @property
    def rejected_writes(self) -> int:
        return self._rejected_writes

    def assert_driver_isolation(self) -> dict[str, Any]:
        loaded_forbidden = [m for m in FORBIDDEN_DRIVER_MODULES if m in sys.modules]
        return {
            "read_only": self.READ_ONLY,
            "isolated": len(loaded_forbidden) == 0,
            "forbidden_modules_loaded": loaded_forbidden,
            "connected_clients": len(self._clients),
            "rejected_write_attempts": self._rejected_writes,
            "last_broadcast_ts": self._last_broadcast_ts,
        }

    def handle_incoming_message(self, raw: str) -> dict[str, Any]:
        """Only allows read-only ping / subscribe messages; explicitly blocks write commands."""
        try:
            msg = json.loads(raw)
        except Exception:
            return {"type": "error", "code": "invalid_json", "message": "Malformed WebSocket message"}

        msg_type = str(msg.get("type", "")).lower()
        if msg_type in ("ping", "subscribe", "status"):
            return {"type": "ack", "read_only": True, "ts": time.time()}

        # Explicitly reject any reverse-control attempt
        self._rejected_writes += 1
        return {
            "type": "rejected",
            "code": "readonly_mirror_violation",
            "message": "Motrix 仿真镜像严格只读，禁止通过 /ws/motrix 反向下发任何控制指令。",
        }

    async def register(self, ws: WebSocket) -> None:
        await ws.accept()
        self._clients.add(ws)
        await ws.send_json({"type": "hello", "read_only": True, "isolation": self.assert_driver_isolation()})

    def unregister(self, ws: WebSocket) -> None:
        self._clients.discard(ws)

    async def broadcast_observation(self, robot_id: str, joint_positions: dict[str, float], mode: str) -> None:
        if not self._clients:
            return
        self._last_broadcast_ts = time.time()
        payload = {
            "type": "mirror_pose",
            "read_only": True,
            "robot_id": robot_id,
            "mode": mode,
            "timestamp": self._last_broadcast_ts,
            "joints": joint_positions,
        }
        dead: list[WebSocket] = []
        for ws in list(self._clients):
            try:
                await asyncio.wait_for(ws.send_json(payload), timeout=0.5)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self._clients.discard(ws)
