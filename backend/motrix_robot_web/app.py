from __future__ import annotations

import asyncio
import json
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

import math
import yaml
from fastapi import FastAPI, HTTPException, Query, Request, WebSocket, WebSocketDisconnect
from fastapi.exceptions import RequestValidationError
from fastapi.responses import FileResponse, JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles

from .connectors import (
    DexW1ReadOnlyConnector,
    EdgeConnector,
    PiperLeLabConnector,
    R1ProBridgeConnector,
    RobotConnector,
    Ros2BridgeConnector,
    build_default_mock_connectors,
    probe_ros2_bridge,
)
from .connectors.mock import MockRobotConnector, _piper_descriptor, _r1pro_descriptor, _w1_descriptor
from .kinematics import RobotKinematics
from .motrix_bridge import MotrixReadOnlyBridge
from .robot_importer import BUILTIN_ROBOT_IDS, RobotPackageManager, build_sample_robot_package_zip
from .safety import RobotSafetyGate, SafetyError, utc_iso
from .schemas import (
    ChassisVelRequest,
    EEStepRequest,
    EstopRequest,
    JointTargetsRequest,
    KinematicsFKRequest,
    KinematicsIKRequest,
    LeaseRequest,
    NavGoalRequest,
    RobotDescriptor,
    TaskPermission,
    TrajectoryPlanRequest,
    WorkflowDefinition,
)
from .storage import LocalStore
from .workflow_engine import WorkflowRunner, action_produces, action_requires

PROJECT_ROOT = Path(__file__).resolve().parents[2]
FRONTEND_DIR = PROJECT_ROOT / "frontend"
DEFAULT_DATA_DIR = PROJECT_ROOT / "data"


class AppContext:
    def __init__(self, data_dir: Path | None = None, config_path: Path | None = None) -> None:
        self.store = LocalStore(data_dir or DEFAULT_DATA_DIR)
        self.importer = RobotPackageManager(self.store.data_dir / "custom_robots")
        self.motrix_bridge = MotrixReadOnlyBridge()
        self.connectors: dict[str, RobotConnector] = build_default_mock_connectors()
        self._load_config_connectors(config_path)

        self.gates: dict[str, RobotSafetyGate] = {
            rid: RobotSafetyGate(conn.descriptor) for rid, conn in self.connectors.items()
        }
        self.kinematics: dict[str, RobotKinematics] = {}
        for rid, conn in self.connectors.items():
            try:
                self.kinematics[rid] = RobotKinematics(conn.descriptor, custom_robots_dir=self.importer.packages_dir)
            except Exception:
                pass
        self.runner = WorkflowRunner(self.store, kinematics_map=self.kinematics)
        for rid, gate in self.gates.items():
            gate.on_timeout = lambda reason, rid=rid: self.stop_robot(rid, f"watchdog:{reason}")

        # Reload persisted custom robots imported via Robot Package Wizard
        for desc in self.importer.load_persisted_descriptors():
            if desc.id not in self.connectors:
                self.register_robot(desc)

        # Enforce Piper single-CAN-writer invariant at startup
        PiperLeLabConnector.assert_no_dual_can_writer(
            [
                (c.descriptor.robot_type, c.descriptor.connector_type, c.descriptor.readonly_only)
                for c in self.connectors.values()
            ]
        )
        self._bg_task: asyncio.Task[Any] | None = None

    def register_robot(self, desc: RobotDescriptor) -> RobotConnector:
        """Hot-register a validated RobotDescriptor into live connectors, safety gates, and kinematics."""
        if desc.connector_type == "mock":
            conn: RobotConnector = MockRobotConnector(desc, custom_robots_dir=self.importer.packages_dir)
        elif desc.connector_type == "ros2_bridge":
            conn = Ros2BridgeConnector(desc, desc.ros2_config, custom_robots_dir=self.importer.packages_dir)
            conn.start_background()
        elif desc.connector_type == "r1pro":
            conn = R1ProBridgeConnector(desc, desc.endpoint)
        elif desc.connector_type == "piper_lelab":
            conn = PiperLeLabConnector(desc, desc.endpoint)
        elif desc.connector_type == "dex_w1":
            conn = DexW1ReadOnlyConnector(desc, desc.endpoint)
        else:
            # Mainline EdgeConnector (or generic HTTP bridge defaulting to readonly)
            conn = EdgeConnector(desc, desc.endpoint)

        gate = RobotSafetyGate(conn.descriptor)
        rid = desc.id
        gate.on_timeout = lambda reason, rid=rid: self.stop_robot(rid, f"watchdog:{reason}")
        self.connectors[rid] = conn
        self.gates[rid] = gate
        try:
            self.kinematics[rid] = RobotKinematics(conn.descriptor, custom_robots_dir=self.importer.packages_dir)
        except Exception:
            self.kinematics.pop(rid, None)
        return conn

    def unregister_robot(self, robot_id: str) -> None:
        if robot_id in BUILTIN_ROBOT_IDS:
            raise ValueError(f"内置机型 '{robot_id}' 不可删除")
        if robot_id in self.connectors:
            try:
                self.stop_robot(robot_id, "robot_deleted")
            except Exception:
                pass
        if robot_id in self.gates:
            self.gates[robot_id].release_lease()
        old_conn = self.connectors.pop(robot_id, None)
        if isinstance(old_conn, Ros2BridgeConnector):
            try:
                loop = asyncio.get_running_loop()
                loop.create_task(old_conn.stop_background())
            except RuntimeError:
                pass
        self.gates.pop(robot_id, None)
        self.kinematics.pop(robot_id, None)

    def stop_robot(self, robot_id: str, reason: str) -> None:
        self.connectors[robot_id].safe_stop(reason)
        self.runner.abort_robot_jobs(robot_id, reason)

    def _load_config_connectors(self, config_path: Path | None) -> None:
        if not config_path or not config_path.exists():
            return
        cfg = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
        for entry in cfg.get("robots") or []:
            if not isinstance(entry, dict) or not entry.get("id"):
                continue
            ctype = str(entry.get("connector", "edge"))
            rid = str(entry["id"])
            base_url = str(entry.get("base_url", ""))
            readonly = bool(entry.get("readonly_only", True))
            mode = str(entry.get("mode", "real"))
            name = str(entry.get("name", rid))
            notes = str(entry.get("notes", ""))

            if ctype == "edge":
                desc = _piper_descriptor().model_copy(
                    update={"id": rid, "name": name, "mode": mode, "connector_type": "edge", "readonly_only": readonly, "network_note": notes}
                )
                self.connectors[rid] = EdgeConnector(desc, base_url)
            elif ctype == "r1pro":
                desc = _r1pro_descriptor().model_copy(
                    update={"id": rid, "name": name, "mode": mode, "connector_type": "r1pro", "readonly_only": readonly, "network_note": notes}
                )
                self.connectors[rid] = R1ProBridgeConnector(desc, base_url)
            elif ctype == "piper_lelab":
                desc = _piper_descriptor().model_copy(
                    update={"id": rid, "name": name, "mode": mode, "connector_type": "piper_lelab", "readonly_only": readonly, "network_note": notes}
                )
                self.connectors[rid] = PiperLeLabConnector(desc, base_url, str(entry.get("policy_url", "")))
            elif ctype == "ros2_bridge":
                from .schemas import Ros2BridgeConfig
                r2_cfg = Ros2BridgeConfig.model_validate(entry.get("ros2_config") or {"ws_url": base_url or "ws://127.0.0.1:9090"})
                base_desc = _r1pro_descriptor() if entry.get("robot_type") == "r1pro" else _w1_descriptor() if entry.get("robot_type") == "dexforce_w1" else _piper_descriptor()
                desc = base_desc.model_copy(
                    update={"id": rid, "name": name, "mode": mode, "connector_type": "ros2_bridge", "readonly_only": readonly, "endpoint": r2_cfg.ws_url, "ros2_config": r2_cfg, "network_note": notes}
                )
                self.connectors[rid] = Ros2BridgeConnector(desc, r2_cfg, custom_robots_dir=self.importer.packages_dir)
            elif ctype == "dex_w1":
                desc = _w1_descriptor().model_copy(
                    update={"id": rid, "name": name, "mode": mode, "connector_type": "dex_w1", "readonly_only": True, "network_note": notes}
                )
                self.connectors[rid] = DexW1ReadOnlyConnector(desc, base_url)

    def get_robot(self, robot_id: str) -> tuple[RobotConnector, RobotSafetyGate]:
        if robot_id not in self.connectors:
            raise HTTPException(status_code=404, detail=f"Unknown robot_id: {robot_id}")
        return self.connectors[robot_id], self.gates[robot_id]

    async def background_loop(self) -> None:
        dt = 0.05  # 20 Hz simulation + watchdog loop
        mirror_tick = 0
        while True:
            for rid, conn in list(self.connectors.items()):
                conn.step(dt)
                gate = self.gates.get(rid)
                if gate is None:
                    continue

                def _on_timeout(reason: str, _c: RobotConnector = conn, _rid: str = rid) -> None:
                    _c.safe_stop(f"watchdog:{reason}")
                    self.runner.abort_robot_jobs(_rid, reason)

                gate.check_watchdogs(on_timeout=_on_timeout)

            mirror_tick += 1
            if mirror_tick % 2 == 0 and self.motrix_bridge.client_count > 0:
                default_rid = next(iter(self.connectors.keys()))
                for target_rid in self.motrix_bridge.subscribed_robots(default_rid):
                    target_conn = self.connectors.get(target_rid)
                    if target_conn is not None:
                        obs = target_conn.get_observation()
                        await self.motrix_bridge.broadcast_observation(obs.robot_id, obs.joint_positions, obs.mode)

            await asyncio.sleep(dt)


def create_app(data_dir: Path | None = None, config_path: Path | None = None) -> FastAPI:
    ctx = AppContext(data_dir=data_dir, config_path=config_path)

    @asynccontextmanager
    async def lifespan(_app: FastAPI):
        for conn in list(ctx.connectors.values()):
            if isinstance(conn, Ros2BridgeConnector):
                conn.start_background()
        ctx._bg_task = asyncio.create_task(ctx.background_loop())
        try:
            yield
        finally:
            if ctx._bg_task:
                ctx._bg_task.cancel()
                try:
                    await ctx._bg_task
                except asyncio.CancelledError:
                    pass
            for conn in list(ctx.connectors.values()):
                if isinstance(conn, Ros2BridgeConnector):
                    await conn.stop_background()

    app = FastAPI(title="Motphys Robot Control & Perception Web", version="0.1.0", lifespan=lifespan)
    app.state.ctx = ctx

    @app.exception_handler(SafetyError)
    async def safety_error_handler(_req: Request, exc: SafetyError):
        return JSONResponse(status_code=exc.status_code, content={"code": exc.code, "detail": exc.message})

    @app.exception_handler(RequestValidationError)
    async def validation_error_handler(_req: Request, exc: RequestValidationError):
        def _clean(obj: Any) -> Any:
            if isinstance(obj, float) and not math.isfinite(obj):
                return str(obj)
            if isinstance(obj, dict):
                return {k: _clean(v) for k, v in obj.items()}
            if isinstance(obj, (list, tuple)):
                return [_clean(v) for v in obj]
            return obj

        return JSONResponse(status_code=422, content={"detail": _clean(exc.errors())})

    # ---- Health & Robot Registry ----
    @app.get("/api/health")
    def health() -> dict[str, Any]:
        return {
            "status": "ok",
            "service": "motrix-robot-web",
            "timestamp": utc_iso(),
            "robots": list(ctx.connectors.keys()),
            "motrix_bridge": ctx.motrix_bridge.assert_driver_isolation(),
        }

    @app.get("/api/robots")
    def list_robots() -> dict[str, Any]:
        items = []
        for rid, conn in list(ctx.connectors.items()):
            d = conn.descriptor
            auth = ctx.gates[rid].snapshot()
            obs = conn.get_observation()
            items.append(
                {
                    "id": d.id,
                    "name": d.name,
                    "robot_type": d.robot_type,
                    "mode": d.mode,
                    "integration_status": d.integration_status,
                    "connector_type": d.connector_type,
                    "readonly_only": d.readonly_only,
                    "is_custom": d.is_custom,
                    "endpoint": d.endpoint,
                    "connected": obs.connected,
                    "lease_active": auth.lease_active,
                    "estop_latched": auth.estop_latched,
                    "dof": len(d.joints),
                    "capabilities": d.capabilities.model_dump(),
                }
            )
        return {"robots": items}

    # ---- Dynamic Robot Package Import & 6-Gate Preflight Wizard ----
    @app.get("/api/robots/import/template")
    def download_robot_template() -> Response:
        zip_bytes = build_sample_robot_package_zip("motphys-arm6-demo")
        return Response(
            content=zip_bytes,
            media_type="application/zip",
            headers={
                "Content-Disposition": 'attachment; filename="motphys_robot_template.zip"',
                "Cache-Control": "no-store",
            },
        )

    MAX_UPLOAD_ZIP_BYTES = 64 * 1024 * 1024  # 64 MB compressed upload cap

    @app.post("/api/robots/import/inspect")
    async def inspect_robot_import(request: Request, sample: bool = Query(False)) -> dict[str, Any]:
        """Accept raw .zip bytes (or ?sample=true built-in 6-DOF cobot sample), stage & run 6 preflight gates."""
        try:
            if sample:
                zip_bytes = build_sample_robot_package_zip("motphys-arm6-demo")
            else:
                cl = request.headers.get("content-length")
                if cl and cl.isdigit() and int(cl) > MAX_UPLOAD_ZIP_BYTES:
                    raise HTTPException(status_code=413, detail=f"上传压缩包超过上限 ({MAX_UPLOAD_ZIP_BYTES // (1024 * 1024)} MB)")
                chunks: list[bytes] = []
                total = 0
                async for chunk in request.stream():
                    total += len(chunk)
                    if total > MAX_UPLOAD_ZIP_BYTES:
                        raise HTTPException(status_code=413, detail=f"上传压缩包超过上限 ({MAX_UPLOAD_ZIP_BYTES // (1024 * 1024)} MB)")
                    chunks.append(chunk)
                zip_bytes = b"".join(chunks)
            return ctx.importer.inspect_zip(zip_bytes, ctx.connectors)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    @app.post("/api/robots/import/validate")
    def validate_robot_import(body: dict[str, Any]) -> dict[str, Any]:
        stage_id = str(body.get("stage_id") or "").strip()
        manifest = body.get("manifest")
        if not stage_id or not isinstance(manifest, dict):
            raise HTTPException(status_code=400, detail="缺少 stage_id 或 manifest 配置")
        try:
            return ctx.importer.validate_staged(stage_id, manifest, ctx.connectors)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    @app.post("/api/robots/import/commit")
    def commit_robot_import(body: dict[str, Any]) -> dict[str, Any]:
        stage_id = str(body.get("stage_id") or "").strip()
        manifest = body.get("manifest")
        if not stage_id or not isinstance(manifest, dict):
            raise HTTPException(status_code=400, detail="缺少 stage_id 或 manifest 配置")
        try:
            desc = ctx.importer.commit_staged(stage_id, manifest, ctx.connectors)
            ctx.register_robot(desc)
            return {"ok": True, "robot": desc.model_dump()}
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    @app.post("/api/robots/ros2/probe")
    async def ros2_probe(body: dict[str, Any]) -> dict[str, Any]:
        """Non-intrusive read-only connectivity test against a rosbridge_server WebSocket endpoint."""
        ws_url = str(body.get("ws_url") or "").strip()
        timeout_s = max(0.5, min(5.0, float(body.get("timeout_s", 2.0))))
        return await probe_ros2_bridge(ws_url, timeout_s=timeout_s)

    @app.delete("/api/robots/{robot_id}")
    def delete_custom_robot(robot_id: str) -> dict[str, Any]:
        if robot_id in BUILTIN_ROBOT_IDS:
            raise HTTPException(status_code=400, detail=f"内置机型 '{robot_id}' 受保护，不可删除")
        if robot_id not in ctx.connectors:
            raise HTTPException(status_code=404, detail=f"Unknown robot_id: {robot_id}")
        ctx.unregister_robot(robot_id)
        ctx.importer.delete_custom_robot(robot_id)
        return {"deleted": True, "robot_id": robot_id}

    @app.get("/api/robots/{robot_id}/descriptor")
    def get_descriptor(robot_id: str) -> dict[str, Any]:
        conn, _ = ctx.get_robot(robot_id)
        return conn.descriptor.model_dump()

    def _build_state_payload(robot_id: str) -> dict[str, Any]:
        conn, gate = ctx.get_robot(robot_id)
        obs = conn.get_observation()
        obs.motrix_mirror_clients = ctx.motrix_bridge.client_count
        auth = gate.snapshot()
        active_jobs = [j.model_dump() for j in ctx.store.list_jobs(robot_id=robot_id)[:12]]
        return {
            "descriptor": conn.descriptor.model_dump(),
            "observation": obs.model_dump(),
            "authority": auth.model_dump(),
            "motrix": ctx.motrix_bridge.assert_driver_isolation(),
            "recent_jobs": active_jobs,
            "connectors": [
                {
                    "id": c.descriptor.id,
                    "name": c.descriptor.name,
                    "type": c.descriptor.connector_type,
                    "mode": c.descriptor.mode,
                    "readonly_only": c.descriptor.readonly_only,
                    "mapping": c.contract_mapping() if hasattr(c, "contract_mapping") else {"transport": "in-process mock"},
                }
                for c in ctx.connectors.values()
            ],
        }

    @app.get("/api/robots/{robot_id}/state")
    def get_robot_state(robot_id: str) -> dict[str, Any]:
        return _build_state_payload(robot_id)

    @app.get("/api/events")
    async def sse_events(request: Request, robot_id: str = Query("r1pro-mock")):
        ctx.get_robot(robot_id)

        async def event_stream():
            while True:
                if await request.is_disconnected():
                    break
                if robot_id not in ctx.connectors:
                    yield f"event: robot_removed\ndata: {json.dumps({'robot_id': robot_id})}\n\n"
                    break
                try:
                    payload = _build_state_payload(robot_id)
                except HTTPException:
                    break
                yield f"data: {json.dumps(payload, ensure_ascii=False)}\n\n"
                await asyncio.sleep(0.12)

        return StreamingResponse(event_stream(), media_type="text/event-stream")

    @app.get("/api/robots/{robot_id}/camera/{stream_key}")
    def get_camera_frame(robot_id: str, stream_key: str) -> Response:
        conn, _ = ctx.get_robot(robot_id)
        svg = conn.get_camera_svg(stream_key)
        return Response(
            content=svg,
            media_type="image/svg+xml",
            headers={
                "Cache-Control": "no-store",
                "Content-Security-Policy": "default-src 'none'; sandbox",
                "X-Content-Type-Options": "nosniff",
            },
        )

    # ---- Authority: High-Level Permission vs Underlying Lease & Lease-Free E-Stop ----
    @app.post("/api/robots/{robot_id}/permission")
    def set_permission(robot_id: str, body: dict[str, Any]) -> dict[str, Any]:
        conn, gate = ctx.get_robot(robot_id)
        perm: TaskPermission = body.get("task_permission", "operator")
        if perm not in ("viewer", "operator", "admin"):
            raise SafetyError(400, "invalid_permission", f"无效高层任务权限：{perm}")
        if perm == "viewer":
            ctx.stop_robot(robot_id, "permission_downgrade")
        auth = gate.set_task_permission(perm, body.get("operator_name"))
        return {"authority": auth.model_dump()}

    @app.post("/api/robots/{robot_id}/lease/acquire")
    def acquire_lease(robot_id: str, req: LeaseRequest) -> dict[str, Any]:
        conn, gate = ctx.get_robot(robot_id)
        source = (
            "ros2_bridge_local_lease"
            if conn.descriptor.connector_type == "ros2_bridge"
            else "mock_local_lease_not_edge"
        )
        auth = gate.acquire_lease(
            operator_name=req.operator_name,
            task_permission=req.task_permission,
            confirm_safety=req.confirm_safety,
            lease_source=source,
        )
        for other_id, other_gate in ctx.gates.items():
            if other_id != robot_id and other_gate.snapshot().lease_active:
                ctx.stop_robot(other_id, "robot_switch")
                other_gate.release_lease()
        return {"authority": auth.model_dump()}

    @app.post("/api/robots/{robot_id}/lease/release")
    def release_lease(robot_id: str) -> dict[str, Any]:
        conn, gate = ctx.get_robot(robot_id)
        conn.safe_stop("lease_released")
        ctx.runner.abort_robot_jobs(robot_id, "lease_released")
        auth = gate.release_lease()
        return {"authority": auth.model_dump()}

    @app.post("/api/robots/{robot_id}/lease/heartbeat")
    def lease_heartbeat(robot_id: str, body: dict[str, Any] | None = None) -> dict[str, Any]:
        _, gate = ctx.get_robot(robot_id)
        lease_id = (body or {}).get("lease_id")
        auth = gate.heartbeat(lease_id=lease_id)
        return {"authority": auth.model_dump()}

    @app.post("/api/robots/{robot_id}/estop")
    def trigger_estop(robot_id: str, req: EstopRequest | None = None) -> dict[str, Any]:
        """Always allowed WITHOUT lease or task permission."""
        conn, gate = ctx.get_robot(robot_id)
        payload = req or EstopRequest()
        conn.safe_stop(f"estop:{payload.reason}")
        ctx.runner.abort_robot_jobs(robot_id, f"estop:{payload.reason}")
        auth = gate.trigger_estop(source=payload.source, reason=payload.reason)
        return {"accepted": True, "bypass_lease": True, "scope": "local_mock" if conn.descriptor.connector_type == "mock" else "local_latch_only", "remote_stop_confirmed": False, "authority": auth.model_dump()}

    @app.post("/api/robots/{robot_id}/estop/clear")
    def clear_estop(robot_id: str, body: dict[str, Any] | None = None) -> dict[str, Any]:
        conn, gate = ctx.get_robot(robot_id)
        operator = str((body or {}).get("operator_name") or "operator@motphys")
        auth = gate.clear_estop(operator_name=operator)
        # Also allow clearing FAULT nav state back to MANUAL cleanly
        if conn.descriptor.capabilities.navigation:
            try:
                conn.set_navigation_mode("MANUAL")
            except SafetyError:
                pass
        return {"authority": auth.model_dump()}

    # ---- Motion: Joints, 6D EE, Arms-Only Teach ----
    @app.post("/api/robots/{robot_id}/joints/targets")
    def send_joint_targets(robot_id: str, req: JointTargetsRequest) -> dict[str, Any]:
        conn, gate = ctx.get_robot(robot_id)
        obs = conn.get_observation()
        validated = gate.validate_joint_targets(req.targets, obs)
        res = conn.send_joint_targets(validated, duration_s=req.duration_s)
        return {"result": res, "authority": gate.snapshot().model_dump()}

    @app.post("/api/robots/{robot_id}/ee/step")
    def send_ee_step(robot_id: str, req: EEStepRequest) -> dict[str, Any]:
        conn, gate = ctx.get_robot(robot_id)
        gate.require_control_authority(conn.get_observation())
        res = conn.send_ee_step(req.arm, req.dx, req.dy, req.dz, req.droll_deg, req.dpitch_deg, req.dyaw_deg)
        return {"result": res}

    # ---- Read-Only URDF FK/IK & Quintic Trajectory Planning (Zero Lease, Zero Motion!) ----
    @app.post("/api/robots/{robot_id}/kinematics/fk")
    def kinematics_fk(robot_id: str, req: KinematicsFKRequest | None = None) -> dict[str, Any]:
        conn, _ = ctx.get_robot(robot_id)
        kin = ctx.kinematics.get(robot_id)
        if kin is None:
            raise HTTPException(status_code=400, detail=f"Robot '{robot_id}' has no URDF kinematics configured")
        obs = conn.get_observation()
        joints = (req.joints if req and req.joints else obs.joint_positions)
        if req and req.arm:
            return {"fk": {req.arm: kin.fk(req.arm, joints)}, "base_frame": kin.base_frame}
        return {"fk": kin.fk_all_arms(joints), "base_frame": kin.base_frame}

    @app.post("/api/robots/{robot_id}/kinematics/ik")
    def kinematics_ik(robot_id: str, req: KinematicsIKRequest) -> dict[str, Any]:
        conn, _ = ctx.get_robot(robot_id)
        kin = ctx.kinematics.get(robot_id)
        if kin is None:
            raise HTTPException(status_code=400, detail=f"Robot '{robot_id}' has no URDF kinematics configured")
        obs = conn.get_observation()
        seed = req.seed_joints if req.seed_joints else obs.joint_positions
        cur_fk = kin.fk(req.arm, seed)
        if req.xyz_m is not None and len(req.xyz_m) >= 3:
            tgt_xyz = [float(req.xyz_m[0]), float(req.xyz_m[1]), float(req.xyz_m[2])]
        else:
            d = req.delta_xyz_m or [0.0, 0.0, 0.0]
            tgt_xyz = [cur_fk["position"][i] + float(d[i]) for i in range(3)]
        if req.rpy_deg is not None and len(req.rpy_deg) >= 3:
            tgt_rpy: list[float] | None = [float(req.rpy_deg[0]), float(req.rpy_deg[1]), float(req.rpy_deg[2])]
        elif req.delta_rpy_deg is not None and len(req.delta_rpy_deg) >= 3:
            tgt_rpy = [cur_fk["rpy_deg"][i] + float(req.delta_rpy_deg[i]) for i in range(3)]
        else:
            tgt_rpy = cur_fk["rpy_deg"]
        if req.plan_trajectory:
            return kin.plan_cartesian_ik(
                req.arm,
                seed,
                tgt_xyz,
                tgt_rpy,
                position_only=req.position_only,
                waypoints=req.waypoints,
                duration_s=req.duration_s,
                max_vel=req.max_vel,
            )
        ik_res = kin.solve_ik(req.arm, tgt_xyz, tgt_rpy, seed, position_only=req.position_only)
        return {"ik": ik_res, "plan": None}

    @app.post("/api/robots/{robot_id}/kinematics/plan")
    def kinematics_plan(robot_id: str, req: TrajectoryPlanRequest) -> dict[str, Any]:
        conn, _ = ctx.get_robot(robot_id)
        kin = ctx.kinematics.get(robot_id)
        if kin is None:
            raise HTTPException(status_code=400, detail=f"Robot '{robot_id}' has no URDF kinematics configured")
        obs = conn.get_observation()
        start = req.start_joints if req.start_joints else obs.joint_positions
        goal = {**start, **req.targets}
        plan = kin.build_joint_plan(start, goal, max_vel=req.max_vel, duration_s=req.duration_s)
        return {"ok": True, "plan": plan}

    @app.post("/api/robots/{robot_id}/teach/check")
    def check_teach_groups(robot_id: str, body: dict[str, Any]) -> dict[str, Any]:
        conn, gate = ctx.get_robot(robot_id)
        gate.require_control_authority(conn.get_observation())
        groups = list(body.get("groups") or [])
        gate.validate_teach_request(groups)
        return {"allowed": True, "groups": groups, "policy": "arms_only"}

    # ---- Navigation & Chassis ----
    @app.get("/api/robots/{robot_id}/navigation/map")
    def get_nav_map(robot_id: str) -> dict[str, Any]:
        """Read-only real recorded SLAM map / occupancy metadata. Never acquires a lease or starts motion."""
        conn, _gate = ctx.get_robot(robot_id)
        desc = conn.descriptor
        if not desc.capabilities.navigation:
            return {
                "supported": False,
                "reason": f"{desc.name} 无移动底盘导航能力，不提供占用栅格或点云，也不使用其他机型地图替代。",
                "pointcloud": {"available": False, "live": False, "source": None},
                "maps": [],
            }
        if desc.is_custom:
            if not desc.extra_maps:
                return {
                    "supported": False,
                    "reason": f"{desc.name} 导入包未包含 2D/3D 导航地图文件（map.png + map.yaml），不替代其他机型地图。",
                    "pointcloud": {"available": False, "live": False, "source": None},
                    "maps": [],
                }
            first_map = desc.extra_maps[0]
            return {
                "supported": True,
                "default_map_id": first_map["id"],
                "kind": first_map["kind"],
                "image_url": first_map["image_url"],
                "yaml": first_map["yaml"],
                "pointcloud": first_map["pointcloud"],
                "maps": desc.extra_maps,
            }
        fast_slam_map = {
            "id": "fast_slam_20260914",
            "name": "R1 Pro Fast-SLAM Plus 高精地图 (bag_20260914 · 0.05m/pix + 98.1万点 3D 点云)",
            "kind": "recorded_slam_map",
            "image_url": "/assets/maps/fast_slam_20260914/bag_20260914_map.png",
            "yaml_url": "/assets/maps/fast_slam_20260914/bag_20260914_map.yaml",
            "yaml": {"resolution": 0.05, "origin": [-14.8, -29.1, 0.0], "width": 709, "height": 755},
            "pointcloud": {
                "available": True,
                "live": False,
                "source": "recorded_lidar_pcd",
                "pcd_url": "/assets/maps/fast_slam_20260914/virtual_machine_map.pcd",
                "point_count": 981005,
                "frame": "map",
                "label": "真实 R1 Pro Fast-SLAM Plus 高密建图点云 (virtual_machine_map.pcd · 981,005 点 · 离线静态地图，非实时扫描)",
            },
        }
        mx_map = {
            "id": "mx_map",
            "name": "R1 Pro pc10 Web 现场 2D 地图 (mx_map · 0.05m/pix)",
            "kind": "occupancy_grid",
            "image_url": "/assets/mx_map.png",
            "yaml_url": "/assets/mx_map.yaml",
            "yaml": {"resolution": 0.05, "origin": [-9.19, -7.85, 0.0], "width": 321, "height": 349},
            "pointcloud": {
                "available": True,
                "live": False,
                "source": "mock_from_occupancy",
                "pcd_url": None,
                "frame": "map",
                "label": "Mock 示例点云，由本地 2D 占用栅格生成，不是真实激光雷达或现场采集。",
            },
        }
        robust_slam_map = {
            "id": "r1pro_20260918",
            "name": "R1 Pro Robust-SLAM 压缩对照图 (r1pro_20260918 · 0.15m/pix + 21.5万点)",
            "kind": "recorded_slam_map",
            "image_url": "/assets/maps/r1pro_20260918/r1pro_map.png",
            "yaml_url": "/assets/maps/r1pro_20260918/r1pro_map.yaml",
            "yaml": {"resolution": 0.15, "origin": [-28.05, -21.75, 0.0], "width": 274, "height": 317},
            "pointcloud": {
                "available": True,
                "live": False,
                "source": "recorded_lidar_pcd",
                "pcd_url": "/assets/maps/r1pro_20260918/r1pro_map.pcd",
                "point_count": 215046,
                "area_m2": 1404.0,
                "frame": "map",
                "label": "真实 R1 Pro Robust-SLAM 压缩建图点云 (r1pro_20260918 · 215,046 点 · 离线静态地图，非实时扫描)",
            },
        }
        return {
            "supported": True,
            "default_map_id": "fast_slam_20260914",
            "kind": fast_slam_map["kind"],
            "image_url": fast_slam_map["image_url"],
            "yaml": fast_slam_map["yaml"],
            "pointcloud": fast_slam_map["pointcloud"],
            "maps": [fast_slam_map, mx_map, robust_slam_map],
        }

    @app.get("/api/navigation/waypoints")
    def list_waypoints(robot_id: str | None = None, map_id: str | None = None) -> dict[str, Any]:
        return {"waypoints": [w.model_dump() for w in ctx.store.list_waypoints(robot_id=robot_id, map_id=map_id)]}

    @app.post("/api/navigation/waypoints")
    def save_waypoint(body: dict[str, Any]) -> dict[str, Any]:
        wp = ctx.store.upsert_waypoint(body)
        return {"waypoint": wp.model_dump()}

    @app.delete("/api/navigation/waypoints/{waypoint_id}")
    def delete_waypoint(waypoint_id: str) -> dict[str, Any]:
        if not ctx.store.delete_waypoint(waypoint_id):
            raise HTTPException(status_code=404, detail="Waypoint not found")
        return {"deleted": True, "id": waypoint_id}

    @app.post("/api/robots/{robot_id}/navigation/mode")
    def set_nav_mode(robot_id: str, body: dict[str, Any]) -> dict[str, Any]:
        conn, gate = ctx.get_robot(robot_id)
        gate.require_control_authority(conn.get_observation())
        return conn.set_navigation_mode(str(body.get("mode", "MANUAL")))

    @app.post("/api/robots/{robot_id}/navigation/goal")
    def send_nav_goal(robot_id: str, req: NavGoalRequest) -> dict[str, Any]:
        conn, gate = ctx.get_robot(robot_id)
        gate.require_control_authority(conn.get_observation())
        return conn.send_navigation_goal(req.x, req.y, req.yaw)

    @app.post("/api/robots/{robot_id}/navigation/cancel")
    def cancel_nav(robot_id: str) -> dict[str, Any]:
        conn, _ = ctx.get_robot(robot_id)
        return conn.cancel_navigation()

    @app.post("/api/robots/{robot_id}/navigation/chassis")
    def send_chassis(robot_id: str, req: ChassisVelRequest) -> dict[str, Any]:
        conn, gate = ctx.get_robot(robot_id)
        gate.require_control_authority(conn.get_observation())
        return conn.send_chassis_velocity(req.vx, req.vy, req.omega, req.duration_s)

    @app.post("/api/robots/{robot_id}/navigation/spin")
    def nav_spin(robot_id: str, body: dict[str, Any]) -> dict[str, Any]:
        conn, gate = ctx.get_robot(robot_id)
        gate.require_control_authority(conn.get_observation())
        return conn.start_localization_spin(bool(body.get("active", True)))

    # ---- Perception (Cameras, Head Depth, Strict 3-step YOLO->Latch->FoundationPose, ArUco) ----
    @app.post("/api/robots/{robot_id}/perception/{action}")
    def perception_action(robot_id: str, action: str, body: dict[str, Any] | None = None) -> dict[str, Any]:
        conn, gate = ctx.get_robot(robot_id)
        # Reading/picking 3D point or inspecting perception is allowed in read-only;
        # chassis fine approach or starting GPU workers can run under operator permission/lease.
        stop_actions = {"stop_yolo", "stop_foundation_pose", "aruco_stop_pose"}
        stopping_fine = action == "aruco_set_fine" and not bool((body or {}).get("enabled"))
        if action not in stop_actions | {"pick_3d"} and not stopping_fine:
            gate.require_control_authority(conn.get_observation())
        return conn.control_perception(action, body or {})

    # ---- Policy, Dynamic Checkpoints, Offline Replay, Episode Tagging ----
    @app.post("/api/robots/{robot_id}/policy/{action}")
    def policy_action(robot_id: str, action: str, body: dict[str, Any] | None = None) -> dict[str, Any]:
        conn, gate = ctx.get_robot(robot_id)
        if action != "stop_infer":
            gate.require_control_authority(conn.get_observation())
        return conn.control_policy(action, body or {})

    # ---- Fault Injection (Mock Interlock Drills) ----
    @app.post("/api/robots/{robot_id}/faults/{fault_type}")
    def inject_fault(robot_id: str, fault_type: str) -> dict[str, Any]:
        conn, gate = ctx.get_robot(robot_id)
        # Clearing an injected Mock fault still requires an active lease & operator role,
        # but must not be blocked by the very interlock/stale-feedback fault being cleared.
        obs_for_gate = None if fault_type == "clear" else conn.get_observation()
        gate.require_control_authority(obs_for_gate)
        return conn.inject_fault(fault_type)

    # ---- Atomic Actions & R1 Pro JSON Compatibility ----
    @app.get("/api/actions")
    def list_actions(robot_id: str | None = None) -> dict[str, Any]:
        items = []
        for a in ctx.store.list_actions(robot_id):
            d = a.model_dump()
            d["produces"] = sorted(action_produces(a))
            d["requires"] = sorted(action_requires(a))
            items.append(d)
        return {"actions": items}

    @app.post("/api/actions")
    def save_action(body: dict[str, Any]) -> dict[str, Any]:
        act = ctx.store.upsert_action(body)
        return {"action": act.model_dump()}

    @app.delete("/api/actions/{action_id}")
    def delete_action(action_id: str) -> dict[str, Any]:
        if not ctx.store.delete_action(action_id):
            raise HTTPException(status_code=404, detail="Action not found")
        return {"deleted": True, "id": action_id}

    @app.post("/api/actions/{action_id}/execute")
    async def execute_atomic_action(action_id: str, body: dict[str, Any] | None = None) -> dict[str, Any]:
        """Execute a single AtomicAction through WorkflowRunner (identical SafetyGate + Lease + audit trail)."""
        from .schemas import WorkflowStep

        act = ctx.store.actions.get(action_id)
        if not act:
            raise HTTPException(status_code=404, detail="Action not found")
        conn, gate = ctx.get_robot(act.robot_id)
        dry_run = bool((body or {}).get("dry_run", False))
        operator = str((body or {}).get("operator") or gate.snapshot().operator_name)
        duration = float(act.params.get("duration_s", 1.2))
        ephemeral = WorkflowDefinition(
            id=f"single-{act.id}",
            name=f"action:{act.name}",
            robot_id=act.robot_id,
            description=act.description,
            kind="Manual",
            steps=[WorkflowStep(id="st-1", action_name=act.name, delay_s=0.0, duration_s=max(0.2, min(15.0, duration)))],
        )
        job = ctx.runner.start_workflow(ephemeral, conn, gate, dry_run=dry_run, operator=operator)
        job.workflow_id = None
        ctx.store.save_job(job)
        return {"job": job.model_dump()}

    @app.post("/api/actions/import-r1pro")
    def import_r1pro_actions(body: dict[str, Any], robot_id: str = Query("r1pro-mock")) -> dict[str, Any]:
        imported = ctx.store.import_r1pro_actions(body, robot_id=robot_id)
        return {"imported_count": len(imported), "actions": [a.model_dump() for a in imported[:10]]}

    @app.get("/api/actions/export-r1pro")
    def export_r1pro_bundle(robot_id: str = Query("r1pro-mock")) -> dict[str, Any]:
        return ctx.store.export_r1pro_bundle(robot_id)

    # ---- Workflows & Loop-Style Execution Jobs ----
    @app.get("/api/workflows")
    def list_workflows(robot_id: str | None = None) -> dict[str, Any]:
        out = []
        for w in ctx.store.list_workflows(robot_id):
            d = w.model_dump()
            d["validation"] = ctx.runner.validate_workflow(w)
            out.append(d)
        return {"workflows": out}

    @app.post("/api/workflows/validate")
    def validate_workflow_route(body: dict[str, Any]) -> dict[str, Any]:
        wf_id = body.get("workflow_id") or body.get("id")
        if wf_id and not body.get("steps") and wf_id in ctx.store.workflows:
            wf = ctx.store.workflows[wf_id]
        else:
            wf = WorkflowDefinition.model_validate({
                "id": str(body.get("id") or "draft-validate"),
                "name": str(body.get("name") or "draft"),
                "robot_id": str(body.get("robot_id") or "r1pro-mock"),
                "description": str(body.get("description") or ""),
                "kind": body.get("kind") or "Workflow",
                "steps": body.get("steps") or [],
            })
        return ctx.runner.validate_workflow(wf)

    @app.post("/api/workflows/{workflow_id}/preview")
    def preview_workflow(workflow_id: str) -> dict[str, Any]:
        """Read-only 3D trajectory + parameter-passing preview for any workflow (zero Lease, zero motion)."""
        wf = ctx.store.workflows.get(workflow_id)
        if not wf:
            raise HTTPException(status_code=404, detail="Workflow not found")
        conn, _ = ctx.get_robot(wf.robot_id)
        return ctx.runner.preview_workflow_trajectory(wf, conn)

    @app.post("/api/workflows")
    def save_workflow(body: dict[str, Any]) -> dict[str, Any]:
        wf = ctx.store.upsert_workflow(body)
        return {"workflow": wf.model_dump(), "validation": ctx.runner.validate_workflow(wf)}

    @app.delete("/api/workflows/{workflow_id}")
    def delete_workflow(workflow_id: str) -> dict[str, Any]:
        if not ctx.store.delete_workflow(workflow_id):
            raise HTTPException(status_code=404, detail="Workflow not found")
        return {"deleted": True, "id": workflow_id}

    @app.post("/api/workflows/import-r1pro")
    def import_r1pro_sequences(body: dict[str, Any], robot_id: str = Query("r1pro-mock")) -> dict[str, Any]:
        imported = ctx.store.import_r1pro_sequences(body, robot_id=robot_id)
        return {"imported_count": len(imported), "workflows": [w.model_dump() for w in imported]}

    @app.post("/api/workflows/{workflow_id}/run")
    async def run_workflow(workflow_id: str, body: dict[str, Any] | None = None) -> dict[str, Any]:
        wf = ctx.store.workflows.get(workflow_id)
        if not wf:
            raise HTTPException(status_code=404, detail="Workflow not found")
        conn, gate = ctx.get_robot(wf.robot_id)
        dry_run = bool((body or {}).get("dry_run", False))
        step_mode = bool((body or {}).get("step_mode", False))
        operator = str((body or {}).get("operator") or gate.snapshot().operator_name)
        job = ctx.runner.start_workflow(wf, conn, gate, dry_run=dry_run, operator=operator, step_mode=step_mode)
        return {"job": job.model_dump()}

    @app.get("/api/jobs")
    def list_jobs(
        robot_id: str | None = None,
        state: str | None = None,
        kind: str | None = None,
    ) -> dict[str, Any]:
        return {"jobs": [j.model_dump() for j in ctx.store.list_jobs(robot_id=robot_id, state=state, kind=kind)]}

    @app.get("/api/jobs/{job_id}")
    def get_job(job_id: str) -> dict[str, Any]:
        job = ctx.store.get_job(job_id)
        if not job:
            raise HTTPException(status_code=404, detail="Job not found")
        return {"job": job.model_dump()}

    @app.post("/api/jobs/{job_id}/pause")
    async def pause_job(job_id: str) -> dict[str, Any]:
        job = ctx.runner.pause_job(job_id)
        if not job:
            raise HTTPException(status_code=404, detail="Job not found")
        return {"job": job.model_dump()}

    @app.post("/api/jobs/{job_id}/resume")
    async def resume_job(job_id: str) -> dict[str, Any]:
        job = ctx.runner.resume_job(job_id, single_step=False)
        if not job:
            raise HTTPException(status_code=404, detail="Job not found")
        return {"job": job.model_dump()}

    @app.post("/api/jobs/{job_id}/step")
    async def step_job(job_id: str) -> dict[str, Any]:
        job = ctx.runner.resume_job(job_id, single_step=True)
        if not job:
            raise HTTPException(status_code=404, detail="Job not found")
        return {"job": job.model_dump()}

    @app.post("/api/jobs/{job_id}/retry")
    async def retry_job(job_id: str, body: dict[str, Any] | None = None) -> dict[str, Any]:
        old = ctx.store.get_job(job_id)
        if not old or not old.workflow_id:
            raise HTTPException(status_code=404, detail="Job or source workflow not found")
        wf = ctx.store.workflows.get(old.workflow_id)
        if not wf:
            raise HTTPException(status_code=404, detail="Workflow no longer exists")
        conn, gate = ctx.get_robot(wf.robot_id)
        dry_run = bool((body or {}).get("dry_run", not gate.snapshot().lease_active))
        step_mode = bool((body or {}).get("step_mode", False))
        job = ctx.runner.start_workflow(wf, conn, gate, dry_run=dry_run, operator=old.operator, step_mode=step_mode)
        return {"job": job.model_dump()}

    @app.post("/api/jobs/{job_id}/cancel")
    async def cancel_job(job_id: str) -> dict[str, Any]:
        existing = ctx.store.get_job(job_id)
        if existing and existing.state in ("Running", "Paused") and existing.lease_id:
            ctx.stop_robot(existing.robot_id, "job_cancel")
        job = ctx.runner.cancel_job(job_id)
        if not job:
            raise HTTPException(status_code=404, detail="Job not found")
        return {"job": job.model_dump()}

    # ---- Loop Object Chain Manual Registry ----
    @app.get("/api/loop/records")
    def list_loop_records(robot_id: str | None = None) -> dict[str, Any]:
        return {"records": [r.model_dump() for r in ctx.store.list_loop_records(robot_id=robot_id)]}

    @app.post("/api/loop/records")
    def save_loop_record(body: dict[str, Any]) -> dict[str, Any]:
        rec = ctx.store.upsert_loop_record(body)
        return {"record": rec.model_dump()}

    @app.delete("/api/loop/records/{record_id}")
    def delete_loop_record(record_id: str) -> dict[str, Any]:
        if not ctx.store.delete_loop_record(record_id):
            raise HTTPException(status_code=404, detail="Loop record not found")
        return {"deleted": True, "record_id": record_id}

    # ---- Strictly Read-Only Motrix Mirror Bridge ----
    @app.get("/api/motrix/state")
    def motrix_state() -> dict[str, Any]:
        return ctx.motrix_bridge.assert_driver_isolation()

    @app.websocket("/ws/motrix")
    async def ws_motrix(ws: WebSocket, robot_id: str | None = Query(None)):
        accepted = await ctx.motrix_bridge.register(ws, robot_id=robot_id)
        if not accepted:
            return
        try:
            while True:
                raw = await ws.receive_text()
                resp = ctx.motrix_bridge.handle_incoming_message(raw, ws=ws)
                await ws.send_json(resp)
        except WebSocketDisconnect:
            pass
        finally:
            ctx.motrix_bridge.unregister(ws)

    @app.middleware("http")
    async def revalidate_frontend(request, call_next):
        response = await call_next(request)
        if request.url.path == "/" or request.url.path.startswith(("/js/", "/css/", "/assets/urdf/", "/custom-robots/")):
            response.headers["Cache-Control"] = "no-cache, must-revalidate"
        return response

    # ---- Static Frontend, Public Custom Robot Packages (never _staging or registry.json) & Offline Vendor Mounts ----
    app.mount("/custom-robots", StaticFiles(directory=str(ctx.importer.packages_dir)), name="custom-robots")
    if (FRONTEND_DIR / "vendor").exists():
        app.mount("/vendor", StaticFiles(directory=str(FRONTEND_DIR / "vendor")), name="vendor")
    if (FRONTEND_DIR / "assets").exists():
        app.mount("/assets", StaticFiles(directory=str(FRONTEND_DIR / "assets")), name="assets")
    if (FRONTEND_DIR / "css").exists():
        app.mount("/css", StaticFiles(directory=str(FRONTEND_DIR / "css")), name="css")
    if (FRONTEND_DIR / "js").exists():
        app.mount("/js", StaticFiles(directory=str(FRONTEND_DIR / "js")), name="js")

    @app.get("/favicon.ico", include_in_schema=False)
    def favicon():
        return FileResponse(FRONTEND_DIR / "assets/motphys-icon.svg", media_type="image/svg+xml")

    @app.get("/")
    def index():
        return FileResponse(FRONTEND_DIR / "index.html")

    return app


app = create_app()
