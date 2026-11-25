from __future__ import annotations

import asyncio
import html
import ipaddress
import json
import math
import os
import time
from collections import deque
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

import websockets

from ..safety import SafetyError
from ..schemas import (
    BatteryState,
    EEPose,
    InterlockItem,
    MotorDiag,
    NavigationState,
    ObservationSnapshot,
    PerceptionState,
    PolicyState,
    RobotDescriptor,
    Ros2BridgeConfig,
)
from .base import RobotConnector


def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def _yaw_to_quat(yaw: float) -> dict[str, float]:
    half = yaw * 0.5
    return {"x": 0.0, "y": 0.0, "z": math.sin(half), "w": math.cos(half)}


def _finite_float(val: Any) -> float | None:
    if val is None or isinstance(val, bool) or not isinstance(val, (int, float)):
        return None
    f = float(val)
    return f if math.isfinite(f) else None


def _is_allowed_ros2_host(hostname: str) -> bool:
    host = hostname.strip().lower()
    if not host:
        return False
    extra = {h.strip().lower() for h in os.environ.get("MOTRIX_ROS2_ALLOW_HOSTS", "").split(",") if h.strip()}
    if host in extra or host in ("localhost", "127.0.0.1", "::1") or host.endswith(".local"):
        return True
    try:
        ip = ipaddress.ip_address(host)
    except ValueError:
        # Reject unverified external DNS hostnames unless explicitly allowlisted
        return False
    # Explicitly block cloud link-local metadata (169.254.0.0/16) while allowing loopback & RFC1918 LAN
    if ip.is_link_local or ip.is_multicast or ip.is_unspecified:
        return False
    return bool(ip.is_loopback or ip.is_private)


async def probe_ros2_bridge(ws_url: str, timeout_s: float = 2.5) -> dict[str, Any]:
    """Non-intrusive read-only connectivity probe against a local/LAN rosbridge_server WebSocket endpoint."""
    url = str(ws_url or "").strip()
    if len(url) > 256:
        return {"ok": False, "ws_url": url[:64], "error": "URL 长度超限", "topics": []}
    parsed = urlparse(url)
    if parsed.scheme not in ("ws", "wss") or not parsed.hostname:
        return {"ok": False, "ws_url": url, "error": "ROS 2 Bridge 地址必须以 ws:// 或 wss:// 开头且包含主机名", "topics": []}
    if parsed.username or parsed.password:
        return {"ok": False, "ws_url": url, "error": "不允许在 URL 中内嵌凭据", "topics": []}
    if parsed.port is not None and not (1 <= parsed.port <= 65535):
        return {"ok": False, "ws_url": url, "error": "端口号超出有效范围 (1-65535)", "topics": []}
    if not _is_allowed_ros2_host(parsed.hostname):
        return {
            "ok": False,
            "ws_url": url,
            "error": f"安全拦截 (SSRF 防护)：仅允许探测本机/局域网私有地址 (127.0.0.1 / RFC1918 / *.local)，拒绝 '{parsed.hostname}'（可通过 MOTRIX_ROS2_ALLOW_HOSTS 显式白名单放行）",
            "topics": [],
        }

    t0 = time.monotonic()
    try:
        async with websockets.connect(url, open_timeout=timeout_s, close_timeout=1.0) as ws:
            latency_ms = round((time.monotonic() - t0) * 1000.0, 1)
            topics: list[str] = []
            rosapi_ok = False
            try:
                req = {"op": "call_service", "id": "motphys-probe-topics", "service": "/rosapi/topics", "args": {}}
                await ws.send(json.dumps(req))
                raw = await asyncio.wait_for(ws.recv(), timeout=min(1.2, timeout_s))
                msg = json.loads(raw)
                if msg.get("op") == "service_response" and msg.get("result"):
                    values = msg.get("values") or {}
                    topics = sorted(str(t) for t in (values.get("topics") or []))
                    rosapi_ok = True
            except Exception:
                # rosbridge_server works fine even without rosapi node launched
                rosapi_ok = False
            return {
                "ok": True,
                "ws_url": url,
                "latency_ms": latency_ms,
                "rosapi_available": rosapi_ok,
                "topics": topics,
                "topic_count": len(topics),
                "message": f"已连通 rosbridge WebSocket ({latency_ms} ms)"
                + (f"，发现 {len(topics)} 个 ROS 2 Topic" if rosapi_ok else "（未启用 rosapi，不影响直接订阅 /joint_states 等话题）"),
            }
    except Exception as exc:
        return {
            "ok": False,
            "ws_url": url,
            "error": f"无法连接 {url}: {exc}",
            "topics": [],
        }


class Ros2BridgeConnector(RobotConnector):
    """Zero-rclpy ROS 2 WebSocket bridge connector speaking standard rosbridge_suite v2 JSON protocol.

    - Connects to rosbridge_server (ws://<robot-ip>:9090) in a background asyncio task with auto-reconnect.
    - Defaults to readonly_only=True (safe P1 observation): streams /joint_states, /odom, /battery_state
      into canonical ObservationSnapshot without touching hardware control loops.
    - When readonly_only=False (explicit operator unlock + active Lease + SafetyGate), publishes validated
      joint commands, chassis Twist, navigation PoseStamped goals, and lease-free zero-velocity safe_stop.
    """

    def __init__(
        self,
        descriptor: RobotDescriptor,
        ros2_config: Ros2BridgeConfig | None = None,
        custom_robots_dir: Path | None = None,
    ) -> None:
        cfg = ros2_config or descriptor.ros2_config or Ros2BridgeConfig(ws_url=descriptor.endpoint or "ws://127.0.0.1:9090")
        self.config = cfg
        self.descriptor = descriptor.model_copy(
            update={
                "connector_type": "ros2_bridge",
                "endpoint": cfg.ws_url,
                "ros2_config": cfg,
                "integration_status": "live",
            }
        )

        self._positions: dict[str, float] = {j.name: j.default for j in self.descriptor.joints}
        self._targets: dict[str, float] = dict(self._positions)
        self._velocities: dict[str, float] = {j.name: 0.0 for j in self.descriptor.joints}
        self._efforts: dict[str, float] = {j.name: 0.0 for j in self.descriptor.joints}

        # Lookup table: ROS 2 joint name (urdf_joint or descriptor name) -> JointSpec
        self._ros_to_spec = {}
        for j in self.descriptor.joints:
            self._ros_to_spec[j.name] = j
            if j.urdf_joint:
                self._ros_to_spec[j.urdf_joint] = j

        # Real URDF FK/IK solver for live EE pose readout & 6D step mapping
        self._kin = None
        try:
            from ..kinematics import RobotKinematics
            self._kin = RobotKinematics(self.descriptor, custom_robots_dir=custom_robots_dir)
        except Exception:
            self._kin = None

        self._connected: bool = False
        self._last_msg_ts: float = 0.0
        self._last_error: str = ""
        self._topic_hits: dict[str, deque[float]] = {
            cfg.joint_states_topic: deque(maxlen=60),
            cfg.odom_topic: deque(maxlen=60),
            cfg.battery_topic: deque(maxlen=30),
        }
        self._nav = NavigationState(
            supported=self.descriptor.capabilities.navigation,
            mode="MANUAL",
            navigation_ready=True,
            pose_map={"x": 0.0, "y": 0.0, "yaw": 0.0},
            goal_map=None,
            chassis_vel={"vx": 0.0, "vy": 0.0, "omega": 0.0},
            path_points=[],
        )
        self._battery = BatteryState(percentage=0.0, voltage_v=0.0, current_a=0.0, source="ros2_bridge")
        self._perc = PerceptionState(
            logs=[f"[ros2_bridge] 等待连接 {cfg.ws_url} (只读={self.descriptor.readonly_only})"]
        )
        self._policy = PolicyState(connected=False, endpoint=cfg.ws_url, active_checkpoint="")

        self._ws: Any = None
        self._outbox: asyncio.Queue[dict[str, Any]] | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._task: asyncio.Task[Any] | None = None
        self._stop_event: asyncio.Event | None = None
        self._advertised: set[str] = set()

    # ---- Async Lifecycle (managed by AppContext) ----

    def start_background(self) -> None:
        if self._task is not None and not self._task.done():
            return
        try:
            loop = asyncio.get_running_loop()
        except RuntimeError:
            return
        self._loop = loop
        self._outbox = asyncio.Queue(maxsize=256)
        self._stop_event = asyncio.Event()
        self._task = loop.create_task(self._ws_runner(), name=f"ros2-bridge-{self.descriptor.id}")

    async def stop_background(self) -> None:
        if self._stop_event is not None:
            self._stop_event.set()
        if self._ws is not None:
            try:
                await self._ws.close()
            except Exception:
                pass
        if self._task is not None:
            self._task.cancel()
            try:
                await self._task
            except asyncio.CancelledError:
                pass
            self._task = None
        self._connected = False

    async def _ws_runner(self) -> None:
        backoff = 1.0
        assert self._stop_event is not None
        assert self._outbox is not None

        while not self._stop_event.is_set():
            try:
                async with websockets.connect(
                    self.config.ws_url,
                    open_timeout=3.0,
                    ping_interval=10.0,
                    ping_timeout=5.0,
                ) as ws:
                    self._ws = ws
                    self._connected = True
                    self._last_error = ""
                    self._advertised.clear()
                    backoff = 1.0

                    # Subscribe to canonical ROS 2 observation topics
                    subs = [
                        (self.config.joint_states_topic, "sensor_msgs/msg/JointState"),
                        (self.config.battery_topic, "sensor_msgs/msg/BatteryState"),
                    ]
                    if self.descriptor.capabilities.navigation or self.config.odom_topic:
                        subs.append((self.config.odom_topic, "nav_msgs/msg/Odometry"))
                    if self.descriptor.capabilities.navigation and self.config.nav_plan_topic:
                        subs.append((self.config.nav_plan_topic, "nav_msgs/msg/Path"))
                    for topic, msg_type in subs:
                        if topic:
                            await ws.send(json.dumps({"op": "subscribe", "topic": topic, "type": msg_type}))

                    reader = asyncio.create_task(self._reader_loop(ws))
                    writer = asyncio.create_task(self._writer_loop(ws))
                    done, pending = await asyncio.wait(
                        {reader, writer}, return_when=asyncio.FIRST_COMPLETED
                    )
                    for p in pending:
                        p.cancel()
                    for d in done:
                        if not d.cancelled():
                            d.exception()
            except asyncio.CancelledError:
                break
            except Exception as exc:
                self._last_error = str(exc)
            finally:
                self._ws = None
                self._connected = False
                self._advertised.clear()

            if self._stop_event.is_set():
                break
            try:
                await asyncio.wait_for(self._stop_event.wait(), timeout=backoff)
            except asyncio.TimeoutError:
                backoff = min(5.0, backoff * 1.5)

    async def _reader_loop(self, ws: Any) -> None:
        async for raw in ws:
            try:
                packet = json.loads(raw)
            except Exception:
                continue
            if packet.get("op") != "publish":
                continue
            topic = str(packet.get("topic") or "")
            msg = packet.get("msg")
            if isinstance(msg, dict):
                self._handle_ros_message(topic, msg)

    async def _writer_loop(self, ws: Any) -> None:
        assert self._outbox is not None
        while True:
            item = await self._outbox.get()
            topic = item.get("topic")
            msg_type = item.get("type")
            if topic and msg_type and topic not in self._advertised:
                await ws.send(json.dumps({"op": "advertise", "topic": topic, "type": msg_type}))
                self._advertised.add(topic)
            await ws.send(json.dumps({"op": "publish", "topic": topic, "msg": item["msg"]}))

    def _enqueue_publish(self, topic: str, msg_type: str, msg: dict[str, Any]) -> None:
        if not self._connected or self._outbox is None or self._loop is None:
            raise SafetyError(503, "ros2_disconnected", f"ROS 2 Bridge 未连接 ({self.config.ws_url})，无法下发控制指令")

        def _put() -> None:
            assert self._outbox is not None
            if self._outbox.full():
                try:
                    self._outbox.get_nowait()
                except asyncio.QueueEmpty:
                    pass
            self._outbox.put_nowait({"topic": topic, "type": msg_type, "msg": msg})

        self._loop.call_soon_threadsafe(_put)

    # ---- Inbound ROS 2 Message Handlers ----

    def _handle_ros_message(self, topic: str, msg: dict[str, Any]) -> None:
        now = time.time()
        self._last_msg_ts = now
        if topic not in self._topic_hits:
            self._topic_hits[topic] = deque(maxlen=60)
        self._topic_hits[topic].append(now)

        if topic == self.config.joint_states_topic:
            names = msg.get("name") or []
            pos = msg.get("position") or []
            vel = msg.get("velocity") or []
            eff = msg.get("effort") or []
            for idx, rname in enumerate(names):
                spec = self._ros_to_spec.get(str(rname))
                if spec is None:
                    continue
                p_val = _finite_float(pos[idx]) if idx < len(pos) else None
                if p_val is not None:
                    scale = float(spec.urdf_scale) or 1.0
                    sign = float(spec.sign) or 1.0
                    offset = float(spec.zero_offset)
                    canonical = ((p_val / scale) - offset) / sign
                    self._positions[spec.name] = round(canonical, 5)
                v_val = _finite_float(vel[idx]) if idx < len(vel) else None
                if v_val is not None:
                    self._velocities[spec.name] = round(v_val, 4)
                e_val = _finite_float(eff[idx]) if idx < len(eff) else None
                if e_val is not None:
                    self._efforts[spec.name] = round(e_val, 3)

        elif topic == self.config.odom_topic:
            pose = (msg.get("pose") or {}).get("pose") or {}
            pos = pose.get("position") or {}
            ori = pose.get("orientation") or {}
            ox = _finite_float(ori.get("x")) or 0.0
            oy = _finite_float(ori.get("y")) or 0.0
            oz = _finite_float(ori.get("z")) or 0.0
            ow = _finite_float(ori.get("w"))
            yaw = _quat_to_yaw(ox, oy, oz, 1.0 if ow is None else ow)
            px = _finite_float(pos.get("x")) or 0.0
            py = _finite_float(pos.get("y")) or 0.0
            self._nav.pose_map = {
                "x": round(px, 4),
                "y": round(py, 4),
                "yaw": round(yaw, 4),
            }
            twist = (msg.get("twist") or {}).get("twist") or {}
            lin = twist.get("linear") or {}
            ang = twist.get("angular") or {}
            self._nav.chassis_vel = {
                "vx": round(_finite_float(lin.get("x")) or 0.0, 3),
                "vy": round(_finite_float(lin.get("y")) or 0.0, 3),
                "omega": round(_finite_float(ang.get("z")) or 0.0, 3),
            }

        elif topic == self.config.nav_plan_topic:
            poses = msg.get("poses") or []
            if isinstance(poses, list) and poses:
                step = max(1, len(poses) // 220)
                pts: list[list[float]] = []
                for idx in range(0, len(poses), step):
                    p = ((poses[idx] or {}).get("pose") or {}).get("position") or {}
                    px = _finite_float(p.get("x"))
                    py = _finite_float(p.get("y"))
                    if px is not None and py is not None:
                        pts.append([round(px, 3), round(py, 3)])
                if len(poses) > 1:
                    last_p = ((poses[-1] or {}).get("pose") or {}).get("position") or {}
                    lx, ly = _finite_float(last_p.get("x")), _finite_float(last_p.get("y"))
                    if lx is not None and ly is not None and (not pts or pts[-1] != [round(lx, 3), round(ly, 3)]):
                        pts.append([round(lx, 3), round(ly, 3)])
                self._nav.path_points = pts
                self._nav.path_source = "nav2_plan" if pts else ""

        elif topic == self.config.battery_topic:
            pct = _finite_float(msg.get("percentage")) or 0.0
            if 0.0 < pct <= 1.0:
                pct *= 100.0
            self._battery = BatteryState(
                percentage=round(pct, 1),
                voltage_v=round(_finite_float(msg.get("voltage")) or 0.0, 2),
                current_a=round(_finite_float(msg.get("current")) or 0.0, 2),
                source="ros2_bridge",
            )

    def _measure_hz(self, hits: deque[float], now: float) -> float:
        recent = [t for t in hits if now - t <= 3.0]
        if len(recent) < 2:
            return 0.0
        span = max(0.2, recent[-1] - recent[0])
        return round((len(recent) - 1) / span, 1)

    # ---- RobotConnector Interface ----

    def step(self, dt: float) -> None:
        # Telemetry is pushed asynchronously via rosbridge WebSocket; ensure background loop is running.
        if self._task is None:
            self.start_background()

    def get_observation(self) -> ObservationSnapshot:
        now = time.time()
        age_ms = round((now - self._last_msg_ts) * 1000.0, 1) if self._last_msg_ts > 0 else 9999.0
        fresh = self._connected and self._last_msg_ts > 0 and age_ms <= 1000.0
        js_hz = self._measure_hz(self._topic_hits.get(self.config.joint_states_topic, deque()), now)

        ee_poses: dict[str, EEPose] = {}
        if self._kin is not None and self.descriptor.capabilities.ee_6d:
            for side in self._kin.arm_cfg:
                try:
                    fk = self._kin.fk(side, self._positions)
                    ee_poses[side] = EEPose(
                        arm=side,
                        frame=fk["base_frame"],
                        x=round(fk["position"][0], 4),
                        y=round(fk["position"][1], 4),
                        z=round(fk["position"][2], 4),
                        roll_deg=round(fk["rpy_deg"][0], 2),
                        pitch_deg=round(fk["rpy_deg"][1], 2),
                        yaw_deg=round(fk["rpy_deg"][2], 2),
                    )
                except Exception:
                    pass

        motors = [
            MotorDiag(
                joint=j.name,
                enabled=fresh,
                temperature_c=0.0,
                effort_nm=self._efforts.get(j.name, 0.0),
                error_code=0,
                statusword="ROS2" if fresh else "OFFLINE",
                severity="ok" if fresh else "warn",
            )
            for j in self.descriptor.joints
        ]

        interlocks = [
            InterlockItem(
                code="ros2_ws_link",
                label="ROS 2 rosbridge 连接",
                severity="ok" if self._connected else "block",
                detail=f"{self.config.ws_url} 已连接" if self._connected else f"未连接 ({self._last_error or '等待 rosbridge_server'})",
            ),
            InterlockItem(
                code="feedback_freshness",
                label="ROS 2 观测新鲜度",
                severity="ok" if fresh else "block",
                detail=f"{age_ms:.0f} ms ({js_hz:.1f} Hz)" if self._last_msg_ts > 0 else "尚未收到 /joint_states",
            ),
            InterlockItem(
                code="write_policy",
                label="ROS 2 写权保护策略",
                severity="ok",
                detail="只读镜像模式 (readonly_only=True)" if self.descriptor.readonly_only else "已授权受控发布（须过 Lease + 3s 心跳）",
            ),
        ]

        topics_list = [
            {
                "name": tname,
                "hz": self._measure_hz(hits, now),
                "status": "ok" if self._measure_hz(hits, now) > 0 else ("idle" if self._connected else "offline"),
            }
            for tname, hits in self._topic_hits.items()
            if tname
        ]

        return ObservationSnapshot(
            telemetry_source="ros2_bridge" if fresh else "unavailable",
            robot_id=self.descriptor.id,
            timestamp=now,
            connected=self._connected,
            mode=self.descriptor.mode,
            feedback_age_ms=age_ms if self._last_msg_ts > 0 else 9999.0,
            measured_hz=js_hz,
            joint_positions=dict(self._positions),
            joint_targets=dict(self._targets),
            joint_velocities=dict(self._velocities),
            joint_efforts=dict(self._efforts),
            ee_poses=ee_poses,
            motors=motors,
            interlocks=interlocks,
            battery=self._battery,
            navigation=self._nav.model_copy(deep=True),
            perception=self._perc.model_copy(deep=True),
            policy=self._policy.model_copy(deep=True),
            topics=topics_list,
        )

    def _require_writable(self, op_name: str) -> None:
        if self.descriptor.readonly_only:
            raise SafetyError(
                403,
                "readonly_bridge",
                f"ROS 2 Bridge '{self.descriptor.id}' 当前处于只读保护模式 (readonly_only=True)，禁止执行 {op_name}。",
            )

    def get_camera_svg(self, stream_key: str) -> str:
        status = "CONNECTED" if self._connected else "DISCONNECTED"
        return (
            f'<svg xmlns="http://www.w3.org/2000/svg" width="640" height="360" viewBox="0 0 640 360">'
            f'<rect width="640" height="360" fill="#0f172a"/>'
            f'<text x="320" y="160" text-anchor="middle" fill="#38bdf8" font-family="monospace" font-size="18">'
            f'ROS 2 Camera Stream: {html.escape(str(stream_key), quote=True)} ({status})</text>'
            f'<text x="320" y="195" text-anchor="middle" fill="#94a3b8" font-family="sans-serif" font-size="13">'
            f'建议由边缘端 web_video_server / WebRTC 提供 MJPEG/H264 流，不经 JSON WebSocket 传大图</text>'
            f'</svg>'
        )

    def send_joint_targets(self, targets: dict[str, float], duration_s: float = 1.2) -> dict[str, Any]:
        self._require_writable("send_joint_targets")
        names: list[str] = []
        positions: list[float] = []
        spec_by_name = {j.name: j for j in self.descriptor.joints}
        for name, val in targets.items():
            spec = spec_by_name.get(name)
            if spec is None:
                continue
            self._targets[name] = float(val)
            ros_name = spec.urdf_joint or spec.name
            ros_val = (float(val) * float(spec.sign) + float(spec.zero_offset)) * float(spec.urdf_scale)
            names.append(ros_name)
            positions.append(round(ros_val, 6))

        if self.config.joint_cmd_type == "trajectory_msgs/msg/JointTrajectory":
            sec = int(max(0.1, duration_s))
            nanosec = int((max(0.1, duration_s) - sec) * 1e9)
            msg = {
                "joint_names": names,
                "points": [{"positions": positions, "time_from_start": {"sec": sec, "nanosec": nanosec}}],
            }
        else:
            msg = {"name": names, "position": positions, "velocity": [], "effort": []}

        self._enqueue_publish(self.config.joint_cmd_topic, self.config.joint_cmd_type, msg)
        return {"ok": True, "published_topic": self.config.joint_cmd_topic, "joints": len(names)}

    def send_ee_step(
        self, arm: str, dx: float, dy: float, dz: float, droll_deg: float, dpitch_deg: float, dyaw_deg: float
    ) -> dict[str, Any]:
        self._require_writable("send_ee_step")
        if self._kin is None:
            raise SafetyError(400, "no_kinematics", "当前 ROS 2 机型未配置 URDF 运动学链，无法求解 6D IK")
        fk = self._kin.fk(arm, self._positions)
        tgt_xyz = [fk["position"][0] + dx, fk["position"][1] + dy, fk["position"][2] + dz]
        tgt_rpy = [fk["rpy_deg"][0] + droll_deg, fk["rpy_deg"][1] + dpitch_deg, fk["rpy_deg"][2] + dyaw_deg]
        ik = self._kin.solve_ik(arm, tgt_xyz, tgt_rpy, self._positions)
        spec_map = {j.name: j for j in self.descriptor.joints}
        safe_targets: dict[str, float] = {}
        for jname, val in (ik.get("solved_joints") or {}).items():
            spec = spec_map.get(jname)
            fval = _finite_float(val)
            if spec is None or fval is None:
                raise SafetyError(400, "invalid_ik_solution", f"IK 求解包含非法关节或数值: {jname}={val}")
            if fval < spec.lower - 1e-6 or fval > spec.upper + 1e-6:
                raise SafetyError(
                    400,
                    "joint_limit_exceeded",
                    f"IK 解出关节 {jname}={fval:.4f} 超出软限位 [{spec.lower:.3f}, {spec.upper:.3f}]",
                )
            safe_targets[jname] = fval
        return self.send_joint_targets(safe_targets, duration_s=0.8)

    def send_chassis_velocity(self, vx: float, vy: float, omega: float, duration_s: float = 0.4) -> dict[str, Any]:
        self._require_writable("send_chassis_velocity")
        msg = {
            "linear": {"x": float(vx), "y": float(vy), "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": float(omega)},
        }
        self._enqueue_publish(self.config.cmd_vel_topic, "geometry_msgs/msg/Twist", msg)
        return {"ok": True, "published_topic": self.config.cmd_vel_topic}

    def set_navigation_mode(self, mode: str) -> dict[str, Any]:
        self._require_writable("set_navigation_mode")
        if mode not in ("MANUAL", "NAVIGATION", "FAULT"):
            raise SafetyError(400, "invalid_nav_mode", f"无效导航模式: {mode}")
        self._nav.mode = mode  # type: ignore[assignment]
        return {"ok": True, "mode": mode}

    def send_navigation_goal(
        self, x: float, y: float, yaw: float, path_points: list[list[float]] | None = None
    ) -> dict[str, Any]:
        self._require_writable("send_navigation_goal")
        msg = {
            "header": {"frame_id": "map"},
            "pose": {
                "position": {"x": float(x), "y": float(y), "z": 0.0},
                "orientation": _yaw_to_quat(float(yaw)),
            },
        }
        self._nav.goal_map = {"x": float(x), "y": float(y), "yaw": float(yaw)}
        self._enqueue_publish(self.config.nav_goal_topic, "geometry_msgs/msg/PoseStamped", msg)
        return {"ok": True, "published_topic": self.config.nav_goal_topic, "goal": self._nav.goal_map}

    def cancel_navigation(self) -> dict[str, Any]:
        self._nav.goal_map = None
        self._nav.path_points = []
        self._nav.path_source = ""
        if not self.descriptor.readonly_only and self._connected:
            try:
                self.send_chassis_velocity(0.0, 0.0, 0.0, 0.2)
            except Exception:
                pass
        return {"ok": True, "canceled": True}

    def start_localization_spin(self, active: bool) -> dict[str, Any]:
        self._require_writable("start_localization_spin")
        self._nav.localization_spin_active = active
        return self.send_chassis_velocity(0.0, 0.0, 0.35 if active else 0.0, 0.5)

    def control_perception(self, action: str, payload: dict[str, Any]) -> dict[str, Any]:
        raise SafetyError(501, "ros2_perception_unmapped", f"ROS 2 Bridge 暂未绑定感知服务动作 '{action}'")

    def control_policy(self, action: str, payload: dict[str, Any]) -> dict[str, Any]:
        raise SafetyError(501, "ros2_policy_unmapped", f"ROS 2 Bridge 暂未绑定策略推理服务动作 '{action}'")

    def safe_stop(self, reason: str) -> None:
        self._nav.goal_map = None
        self._nav.localization_spin_active = False
        if not self.descriptor.readonly_only and self._connected:
            try:
                self._enqueue_publish(
                    self.config.cmd_vel_topic,
                    "geometry_msgs/msg/Twist",
                    {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}},
                )
            except Exception:
                pass

    def contract_mapping(self) -> dict[str, str]:
        return {
            "transport": f"rosbridge WebSocket ({self.config.ws_url})",
            "ros_domain_id": str(self.config.ros_domain_id) if self.config.ros_domain_id is not None else "bridge_default",
            "joint_states": f"SUB {self.config.joint_states_topic} (sensor_msgs/msg/JointState)",
            "odom": f"SUB {self.config.odom_topic} (nav_msgs/msg/Odometry)",
            "joint_cmd": f"PUB {self.config.joint_cmd_topic} ({self.config.joint_cmd_type})",
            "cmd_vel": f"PUB {self.config.cmd_vel_topic} (geometry_msgs/msg/Twist)",
            "nav_goal": f"PUB {self.config.nav_goal_topic} (geometry_msgs/msg/PoseStamped)",
            "nav_plan": f"SUB {self.config.nav_plan_topic} (nav_msgs/msg/Path)",
        }
