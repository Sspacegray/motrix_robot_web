from __future__ import annotations

import asyncio
import json
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

import yaml
from fastapi import FastAPI, HTTPException, Query, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles

from .connectors import (
    DexW1ReadOnlyConnector,
    EdgeConnector,
    PiperLeLabConnector,
    R1ProBridgeConnector,
    RobotConnector,
    build_default_mock_connectors,
)
from .connectors.mock import _piper_descriptor, _r1pro_descriptor, _w1_descriptor
from .motrix_bridge import MotrixReadOnlyBridge
from .safety import RobotSafetyGate, SafetyError, utc_iso
from .schemas import (
    ChassisVelRequest,
    EEStepRequest,
    EstopRequest,
    JointTargetsRequest,
    LeaseRequest,
    NavGoalRequest,
    TaskPermission,
)
from .storage import LocalStore
from .workflow_engine import WorkflowRunner

PROJECT_ROOT = Path(__file__).resolve().parents[2]
FRONTEND_DIR = PROJECT_ROOT / "frontend"
DEFAULT_DATA_DIR = PROJECT_ROOT / "data"


class AppContext:
    def __init__(self, data_dir: Path | None = None, config_path: Path | None = None) -> None:
        self.store = LocalStore(data_dir or DEFAULT_DATA_DIR)
        self.motrix_bridge = MotrixReadOnlyBridge()
        self.connectors: dict[str, RobotConnector] = build_default_mock_connectors()
        self._load_config_connectors(config_path)

        # Enforce Piper single-CAN-writer invariant at startup
        PiperLeLabConnector.assert_no_dual_can_writer(
            [
                (c.descriptor.robot_type, c.descriptor.connector_type, c.descriptor.readonly_only)
                for c in self.connectors.values()
            ]
        )

        self.gates: dict[str, RobotSafetyGate] = {
            rid: RobotSafetyGate(conn.descriptor) for rid, conn in self.connectors.items()
        }
        self.runner = WorkflowRunner(self.store)
        self._bg_task: asyncio.Task[Any] | None = None

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
            for rid, conn in self.connectors.items():
                conn.step(dt)
                gate = self.gates[rid]

                def _on_timeout(reason: str, _c: RobotConnector = conn, _rid: str = rid) -> None:
                    _c.safe_stop(f"watchdog:{reason}")
                    self.runner.abort_robot_jobs(_rid, reason)

                gate.check_watchdogs(on_timeout=_on_timeout)

            mirror_tick += 1
            if mirror_tick % 2 == 0 and self.motrix_bridge.client_count > 0:
                first_conn = next(iter(self.connectors.values()))
                obs = first_conn.get_observation()
                await self.motrix_bridge.broadcast_observation(obs.robot_id, obs.joint_positions, obs.mode)

            await asyncio.sleep(dt)


def create_app(data_dir: Path | None = None, config_path: Path | None = None) -> FastAPI:
    ctx = AppContext(data_dir=data_dir, config_path=config_path)

    @asynccontextmanager
    async def lifespan(_app: FastAPI):
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

    app = FastAPI(title="Motphys Robot Control & Perception Web", version="0.1.0", lifespan=lifespan)
    app.state.ctx = ctx

    @app.exception_handler(SafetyError)
    async def safety_error_handler(_req: Request, exc: SafetyError):
        return JSONResponse(status_code=exc.status_code, content={"code": exc.code, "detail": exc.message})

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
        for rid, conn in ctx.connectors.items():
            d = conn.descriptor
            auth = ctx.gates[rid].snapshot()
            obs = conn.get_observation()
            items.append(
                {
                    "id": d.id,
                    "name": d.name,
                    "robot_type": d.robot_type,
                    "mode": d.mode,
                    "connector_type": d.connector_type,
                    "readonly_only": d.readonly_only,
                    "connected": obs.connected,
                    "lease_active": auth.lease_active,
                    "estop_latched": auth.estop_latched,
                    "dof": len(d.joints),
                    "capabilities": d.capabilities.model_dump(),
                }
            )
        return {"robots": items}

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
                payload = _build_state_payload(robot_id)
                yield f"data: {json.dumps(payload, ensure_ascii=False)}\n\n"
                await asyncio.sleep(0.12)

        return StreamingResponse(event_stream(), media_type="text/event-stream")

    @app.get("/api/robots/{robot_id}/camera/{stream_key}")
    def get_camera_frame(robot_id: str, stream_key: str) -> Response:
        conn, _ = ctx.get_robot(robot_id)
        svg = conn.get_camera_svg(stream_key)
        return Response(content=svg, media_type="image/svg+xml", headers={"Cache-Control": "no-store"})

    # ---- Authority: High-Level Permission vs Underlying Lease & Lease-Free E-Stop ----
    @app.post("/api/robots/{robot_id}/permission")
    def set_permission(robot_id: str, body: dict[str, Any]) -> dict[str, Any]:
        _, gate = ctx.get_robot(robot_id)
        perm: TaskPermission = body.get("task_permission", "operator")
        if perm not in ("viewer", "operator", "admin"):
            raise SafetyError(400, "invalid_permission", f"无效高层任务权限：{perm}")
        auth = gate.set_task_permission(perm, body.get("operator_name"))
        return {"authority": auth.model_dump()}

    @app.post("/api/robots/{robot_id}/lease/acquire")
    def acquire_lease(robot_id: str, req: LeaseRequest) -> dict[str, Any]:
        conn, gate = ctx.get_robot(robot_id)
        source = f"{conn.descriptor.connector_type}_session"
        auth = gate.acquire_lease(
            operator_name=req.operator_name,
            task_permission=req.task_permission,
            confirm_safety=req.confirm_safety,
            lease_source=source,
        )
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
        return {"accepted": True, "bypass_lease": True, "authority": auth.model_dump()}

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

    @app.post("/api/robots/{robot_id}/teach/check")
    def check_teach_groups(robot_id: str, body: dict[str, Any]) -> dict[str, Any]:
        conn, gate = ctx.get_robot(robot_id)
        gate.require_control_authority(conn.get_observation())
        groups = list(body.get("groups") or [])
        gate.validate_teach_request(groups)
        return {"allowed": True, "groups": groups, "policy": "arms_only"}

    # ---- Navigation & Chassis ----
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
        if action == "aruco_set_fine" and bool((body or {}).get("enabled")):
            gate.require_control_authority(conn.get_observation())
        return conn.control_perception(action, body or {})

    # ---- Policy, Dynamic Checkpoints, Offline Replay, Episode Tagging ----
    @app.post("/api/robots/{robot_id}/policy/{action}")
    def policy_action(robot_id: str, action: str, body: dict[str, Any] | None = None) -> dict[str, Any]:
        conn, gate = ctx.get_robot(robot_id)
        if action == "start_infer":
            gate.require_control_authority(conn.get_observation())
        return conn.control_policy(action, body or {})

    # ---- Fault Injection (Mock Interlock Drills) ----
    @app.post("/api/robots/{robot_id}/faults/{fault_type}")
    def inject_fault(robot_id: str, fault_type: str) -> dict[str, Any]:
        conn, _ = ctx.get_robot(robot_id)
        return conn.inject_fault(fault_type)

    # ---- Atomic Actions & R1 Pro JSON Compatibility ----
    @app.get("/api/actions")
    def list_actions(robot_id: str | None = None) -> dict[str, Any]:
        return {"actions": [a.model_dump() for a in ctx.store.list_actions(robot_id)]}

    @app.post("/api/actions")
    def save_action(body: dict[str, Any]) -> dict[str, Any]:
        act = ctx.store.upsert_action(body)
        return {"action": act.model_dump()}

    @app.delete("/api/actions/{action_id}")
    def delete_action(action_id: str) -> dict[str, Any]:
        if not ctx.store.delete_action(action_id):
            raise HTTPException(status_code=404, detail="Action not found")
        return {"deleted": True, "id": action_id}

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
        return {"workflows": [w.model_dump() for w in ctx.store.list_workflows(robot_id)]}

    @app.post("/api/workflows")
    def save_workflow(body: dict[str, Any]) -> dict[str, Any]:
        wf = ctx.store.upsert_workflow(body)
        return {"workflow": wf.model_dump()}

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
        operator = str((body or {}).get("operator") or gate.snapshot().operator_name)
        job = ctx.runner.start_workflow(wf, conn, gate, dry_run=dry_run, operator=operator)
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
        job = ctx.runner.start_workflow(wf, conn, gate, dry_run=dry_run, operator=old.operator)
        return {"job": job.model_dump()}

    @app.post("/api/jobs/{job_id}/cancel")
    def cancel_job(job_id: str) -> dict[str, Any]:
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
    async def ws_motrix(ws: WebSocket):
        await ctx.motrix_bridge.register(ws)
        try:
            while True:
                raw = await ws.receive_text()
                resp = ctx.motrix_bridge.handle_incoming_message(raw)
                await ws.send_json(resp)
        except WebSocketDisconnect:
            ctx.motrix_bridge.unregister(ws)

    # ---- Static Frontend & Offline Vendor Mounts ----
    if (FRONTEND_DIR / "vendor").exists():
        app.mount("/vendor", StaticFiles(directory=str(FRONTEND_DIR / "vendor")), name="vendor")
    if (FRONTEND_DIR / "assets").exists():
        app.mount("/assets", StaticFiles(directory=str(FRONTEND_DIR / "assets")), name="assets")
    if (FRONTEND_DIR / "css").exists():
        app.mount("/css", StaticFiles(directory=str(FRONTEND_DIR / "css")), name="css")
    if (FRONTEND_DIR / "js").exists():
        app.mount("/js", StaticFiles(directory=str(FRONTEND_DIR / "js")), name="js")

    @app.get("/")
    def index():
        return FileResponse(FRONTEND_DIR / "index.html")

    return app


app = create_app()
