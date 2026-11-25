from __future__ import annotations

import asyncio
import json
import math
from pathlib import Path
from typing import Any

import pytest
import websockets
from fastapi.testclient import TestClient

from motrix_robot_web.app import create_app
from motrix_robot_web.connectors.mock import _r1pro_descriptor
from motrix_robot_web.connectors.ros2_bridge import Ros2BridgeConnector, probe_ros2_bridge
from motrix_robot_web.safety import SafetyError
from motrix_robot_web.schemas import Ros2BridgeConfig


class _MockRosbridgeServer:
    """In-process mock rosbridge_server WebSocket server for deterministic local testing."""

    def __init__(self) -> None:
        self.received: list[dict[str, Any]] = []
        self.clients: set[Any] = set()
        self.server: Any = None
        self.port: int = 0
        self.topics = ["/joint_states", "/odom", "/battery_state", "/cmd_vel", "/goal_pose"]

    async def _handler(self, ws: Any) -> None:
        self.clients.add(ws)
        try:
            async for raw in ws:
                msg = json.loads(raw)
                self.received.append(msg)
                if msg.get("op") == "call_service" and msg.get("service") == "/rosapi/topics":
                    await ws.send(
                        json.dumps(
                            {
                                "op": "service_response",
                                "id": msg.get("id"),
                                "service": "/rosapi/topics",
                                "result": True,
                                "values": {"topics": self.topics},
                            }
                        )
                    )
        finally:
            self.clients.discard(ws)

    async def start(self) -> str:
        self.server = await websockets.serve(self._handler, "127.0.0.1", 0)
        sock = next(iter(self.server.sockets))
        self.port = int(sock.getsockname()[1])
        return f"ws://127.0.0.1:{self.port}"

    async def broadcast_publish(self, topic: str, msg: dict[str, Any]) -> None:
        payload = json.dumps({"op": "publish", "topic": topic, "msg": msg})
        for c in list(self.clients):
            await c.send(payload)

    async def stop(self) -> None:
        if self.server is not None:
            self.server.close()
            await self.server.wait_closed()


@pytest.mark.asyncio
async def test_ros2_bridge_subscribe_telemetry_readonly_and_authorized_publish() -> None:
    srv = _MockRosbridgeServer()
    ws_url = await srv.start()
    try:
        # 1. Probe finds server and discovers ROS 2 topics via /rosapi/topics
        probe = await probe_ros2_bridge(ws_url, timeout_s=2.0)
        assert probe["ok"] is True
        assert probe["rosapi_available"] is True
        assert "/joint_states" in probe["topics"]

        # 2. Read-only Ros2BridgeConnector receives /joint_states, /odom, /battery_state and rejects writes
        base_desc = _r1pro_descriptor()
        joints = [j.model_copy(update={"sign": -1.0}) if j.name == "left_arm_1" else j for j in base_desc.joints]
        desc = base_desc.model_copy(update={"id": "r1pro-ros2-ro", "mode": "real", "readonly_only": True, "joints": joints})
        cfg = Ros2BridgeConfig(ws_url=ws_url, ros_domain_id=102)
        conn = Ros2BridgeConnector(desc, cfg)
        conn.start_background()

        # Wait for WebSocket connect & subscriptions
        for _ in range(40):
            if conn.get_observation().connected and len(srv.received) >= 3:
                break
            await asyncio.sleep(0.05)
        assert conn.get_observation().connected is True

        # Stream JointState including JSON null (ROS 2 NaN) — must NOT crash or disconnect!
        await srv.broadcast_publish(
            "/joint_states",
            {
                "name": ["left_arm_joint1", "left_arm_joint2", "left_gripper_finger_joint1"],
                "position": [-0.45, 0.52, 0.025],  # left_arm_1 has sign=-1 -> canonical +0.45; gripper scale=0.05 -> 0.5
                "velocity": [None, -0.05, None],
                "effort": [1.5, None, 0.3],
            },
        )
        await srv.broadcast_publish(
            "/odom",
            {
                "pose": {
                    "pose": {
                        "position": {"x": 1.25, "y": -0.60, "z": 0.0},
                        "orientation": {"x": 0.0, "y": 0.0, "z": math.sin(0.5), "w": math.cos(0.5)},
                    }
                },
                "twist": {"twist": {"linear": {"x": 0.2, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.1}}},
            },
        )
        await srv.broadcast_publish("/battery_state", {"percentage": 0.84, "voltage": 51.2, "current": -3.1})
        await asyncio.sleep(0.1)

        obs = conn.get_observation()
        assert obs.telemetry_source == "ros2_bridge"
        assert abs(obs.joint_positions["left_arm_1"] - 0.45) < 1e-4
        assert abs(obs.joint_positions["left_arm_2"] - 0.52) < 1e-4
        assert abs(obs.joint_positions["left_gripper"] - 0.50) < 1e-4
        assert abs(obs.navigation.pose_map["x"] - 1.25) < 1e-3
        assert abs(obs.navigation.pose_map["yaw"] - 1.0) < 1e-2
        assert obs.battery.percentage == 84.0
        assert "left" in obs.ee_poses and "right" in obs.ee_poses

        # Freshness threshold strictly aligned with SafetyGate (1000ms)
        conn._last_msg_ts -= 1.5
        stale_obs = conn.get_observation()
        fresh_item = next(it for it in stale_obs.interlocks if it.code == "feedback_freshness")
        assert fresh_item.severity == "block"
        conn._last_msg_ts = stale_obs.timestamp

        # Read-only bridge MUST reject all write operations with 403
        with pytest.raises(SafetyError) as exc_info:
            conn.send_joint_targets({"left_arm_1": 0.1})
        assert exc_info.value.status_code == 403
        await conn.stop_background()

        # 3. Writable Ros2BridgeConnector publishes JointState, Twist, PoseStamped & zero-vel safe_stop
        w_desc = desc.model_copy(update={"id": "r1pro-ros2-rw", "readonly_only": False})
        w_conn = Ros2BridgeConnector(w_desc, cfg)
        w_conn.start_background()
        for _ in range(40):
            if w_conn.get_observation().connected:
                break
            await asyncio.sleep(0.05)

        srv.received.clear()
        w_conn.send_joint_targets({"left_arm_1": 0.30, "left_gripper": 0.80}, duration_s=1.0)
        w_conn.send_chassis_velocity(0.25, 0.0, 0.4, 0.5)
        w_conn.send_navigation_goal(2.0, -1.0, 0.5)
        w_conn.safe_stop("test_estop")
        await asyncio.sleep(0.15)

        pubs = [m for m in srv.received if m.get("op") == "publish"]
        by_topic = {}
        for p in pubs:
            by_topic.setdefault(p["topic"], []).append(p["msg"])

        assert "/joint_commands" in by_topic
        jmsg = by_topic["/joint_commands"][0]
        assert jmsg["name"] == ["left_arm_joint1", "left_gripper_finger_joint1"]
        # sign=-1 -> -0.30, gripper scale=0.05 -> 0.04
        assert abs(jmsg["position"][0] - (-0.30)) < 1e-4
        assert abs(jmsg["position"][1] - 0.04) < 1e-4

        assert "/goal_pose" in by_topic
        assert abs(by_topic["/goal_pose"][0]["pose"]["position"]["x"] - 2.0) < 1e-4

        # Last /cmd_vel publish MUST be zero Twist from safe_stop
        assert len(by_topic["/cmd_vel"]) >= 2
        assert by_topic["/cmd_vel"][-1]["linear"]["x"] == 0.0
        assert by_topic["/cmd_vel"][-1]["angular"]["z"] == 0.0
        await w_conn.stop_background()
    finally:
        await srv.stop()


def test_ros2_probe_api_and_import_wizard_ros2_connector(tmp_path: Path) -> None:
    app = create_app(data_dir=tmp_path)
    with TestClient(app) as client:
        # 1. Invalid scheme & SSRF targets (cloud metadata / public IP / external domain) blocked cleanly
        for bad_url in (
            "http://127.0.0.1:9090",
            "ws://169.254.169.254:80/latest",
            "ws://8.8.8.8:9090",
            "ws://evil.example:9090",
        ):
            bad = client.post("/api/robots/ros2/probe", json={"ws_url": bad_url})
            assert bad.status_code == 200
            assert bad.json()["ok"] is False

        # Oversized EE step rejected at schema level (422)
        big_step = client.post("/api/robots/r1pro-mock/ee/step", json={"arm": "left", "dx": 10.0})
        assert big_step.status_code == 422

        # 2. Import sample robot configured as ros2_bridge (with readonly_only=True)
        insp = client.post("/api/robots/import/inspect?sample=true").json()
        manifest = insp["manifest"]
        manifest["id"] = "arm6-ros2-readonly"
        manifest["connector_type"] = "ros2_bridge"
        manifest["mode"] = "real"
        manifest["readonly_only"] = True
        manifest["endpoint"] = "ws://127.0.0.1:19999"
        manifest["ros2_config"] = {
            "ws_url": "ws://127.0.0.1:19999",
            "ros_domain_id": 42,
            "joint_states_topic": "/arm6/joint_states",
        }

        # Bad http:// endpoint fails Gate 4
        bad_ep = {**manifest, "endpoint": "http://127.0.0.1:19999"}
        v_bad = client.post("/api/robots/import/validate", json={"stage_id": insp["stage_id"], "manifest": bad_ep}).json()
        assert v_bad["ready_to_commit"] is False

        # Valid ws:// + readonly_only=True passes all 6 gates and hot-registers Ros2BridgeConnector
        v_ok = client.post("/api/robots/import/validate", json={"stage_id": insp["stage_id"], "manifest": manifest}).json()
        assert v_ok["ready_to_commit"] is True, v_ok["checks"]

        committed = client.post("/api/robots/import/commit", json={"stage_id": insp["stage_id"], "manifest": manifest})
        assert committed.status_code == 200, committed.text
        desc = committed.json()["robot"]
        assert desc["connector_type"] == "ros2_bridge"
        assert desc["readonly_only"] is True
        assert desc["ros2_config"]["joint_states_topic"] == "/arm6/joint_states"

        state = client.get("/api/robots/arm6-ros2-readonly/state").json()
        assert state["descriptor"]["connector_type"] == "ros2_bridge"
        assert state["observation"]["connected"] is False

        # Clean delete
        assert client.delete("/api/robots/arm6-ros2-readonly").status_code == 200
