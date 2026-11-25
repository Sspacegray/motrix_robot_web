from __future__ import annotations

import json
import math
import time
from pathlib import Path

from fastapi.testclient import TestClient

from motrix_robot_web.app import create_app
from motrix_robot_web.motrix_bridge import MotrixReadOnlyBridge
from motrix_robot_web.schemas import ExecutionJob


def test_camera_svg_escapes_xss_and_sets_sandbox_csp(tmp_path: Path) -> None:
    app = create_app(data_dir=tmp_path)
    with TestClient(app) as client:
        # Reflected XSS attempt via stream_key (single URL path segment)
        res = client.get("/api/robots/r1pro-mock/camera/%3Csvg%20onload=alert(1)%3E")
        assert res.status_code == 200
        assert res.headers["content-type"].startswith("image/svg+xml")
        assert "sandbox" in res.headers.get("content-security-policy", "")
        assert res.headers.get("x-content-type-options") == "nosniff"
        assert "<svg onload=" not in res.text
        assert "&lt;svg onload=alert(1)&gt;" in res.text

        # Stored XSS attempt via yolo_prompt
        lease = client.post(
            "/api/robots/r1pro-mock/lease/acquire",
            json={"operator_name": "sec", "task_permission": "operator", "confirm_safety": True},
        )
        assert lease.status_code == 200
        assert client.post("/api/robots/r1pro-mock/perception/set_head_depth", json={"enabled": True}).status_code == 200
        assert client.post("/api/robots/r1pro-mock/perception/start_yolo", json={"prompt": "<img src=x onerror=alert(1)>"}).status_code == 200
        cam = client.get("/api/robots/r1pro-mock/camera/head_left_rgb")
        assert cam.status_code == 200
        assert "<img " not in cam.text
        assert "&lt;img " in cam.text


def test_robot_import_rejects_oversized_upload_before_buffering(tmp_path: Path) -> None:
    app = create_app(data_dir=tmp_path)
    with TestClient(app) as client:
        res = client.post(
            "/api/robots/import/inspect",
            content=b"tiny",
            headers={"Content-Length": str(70 * 1024 * 1024), "Content-Type": "application/zip"},
        )
        assert res.status_code == 413


def test_concurrent_live_workflows_blocked_and_chassis_bounds_enforced(tmp_path: Path) -> None:
    app = create_app(data_dir=tmp_path)
    with TestClient(app) as client:
        assert client.post(
            "/api/robots/r1pro-mock/lease/acquire",
            json={"operator_name": "op", "task_permission": "operator", "confirm_safety": True},
        ).status_code == 200

        # 1. Start a Live workflow -> 200 Running; starting a second Live workflow concurrently -> 409!
        r1 = client.post("/api/workflows/wf-r1-box-transport/run", json={"dry_run": False})
        assert r1.status_code == 200
        job_id = r1.json()["job"]["job_id"]

        r2 = client.post("/api/workflows/wf-r1-box-transport/run", json={"dry_run": False})
        assert r2.status_code == 409
        assert r2.json()["code"] == "workflow_already_running"

        # Dry-run is still allowed while Live is running
        dry = client.post("/api/workflows/wf-r1-box-transport/run", json={"dry_run": True})
        assert dry.status_code == 200

        # Cancel the first Live job cleanly
        client.post(f"/api/jobs/{job_id}/cancel")
        time.sleep(0.15)

        # 2. Workflow action with dangerous chassis_velocity vx=50.0 must fail inside SafetyGate dispatch
        bad_act = client.post(
            "/api/actions",
            json={
                "name": "unsafe_fast_chassis",
                "robot_id": "r1pro-mock",
                "kind": "chassis_velocity",
                "params": {"vx": 50.0, "vy": 0.0, "omega": 0.0, "duration_s": 0.5},
            },
        ).json()["action"]
        exec_res = client.post(f"/api/actions/{bad_act['id']}/execute", json={"dry_run": False})
        assert exec_res.status_code == 200
        bad_job_id = exec_res.json()["job"]["job_id"]
        for _ in range(30):
            j = client.get(f"/api/jobs/{bad_job_id}").json()["job"]
            if j["state"] in ("Failed", "Succeeded", "Canceled"):
                break
            time.sleep(0.05)
        assert j["state"] == "Failed"
        assert any("安全上限" in log for log in j["logs"])


def test_storage_atomic_write_nan_sanitization_and_job_cap(tmp_path: Path) -> None:
    app = create_app(data_dir=tmp_path)
    store = app.state.ctx.store

    # 1. NaN/Inf in R1 Pro import payload must be sanitized so persisted actions.json stays strict JSON
    imported = store.import_r1pro_actions(
        {
            "actions": [
                {
                    "name": "nan_safe_pose",
                    "type": "joint_targets",
                    "params": {"targets": [{"name": "torso[1]", "value": float("nan"), "unit": "rad"}]},
                }
            ]
        },
        robot_id="r1pro-mock",
    )
    assert len(imported) == 1
    assert math.isfinite(imported[0].params["targets"]["torso_1"])
    # Strict JSON parse check (no NaN tokens on disk)
    raw_text = store.actions_path.read_text(encoding="utf-8")
    assert "NaN" not in raw_text
    json.loads(raw_text)

    # 2. In-memory self.jobs stays bounded at <= 200
    for i in range(215):
        store.save_job(
            ExecutionJob(
                job_id=f"cap-job-{i:03d}",
                run_id=f"r-cap-{i:03d}",
                flow_name="cap_test",
                state="Succeeded",
                robot_id="r1pro-mock",
                created_at=f"2026-11-25T00:00:{i % 60:02d}Z",
                started_at="2026-11-25T00:00:00Z",
                finished_at="2026-11-25T00:00:01Z",
                exit_code=0,
            )
        )
    assert len(store.jobs) <= 200


def test_motrix_ws_handles_non_dict_json_oversized_frames_and_cleans_up(tmp_path: Path) -> None:
    bridge = MotrixReadOnlyBridge()
    assert bridge.handle_incoming_message("null")["code"] == "invalid_json"
    assert bridge.handle_incoming_message("[1, 2, 3]")["code"] == "invalid_json"
    assert bridge.handle_incoming_message("x" * 20000)["code"] == "payload_too_large"

    app = create_app(data_dir=tmp_path)
    with TestClient(app) as client:
        with client.websocket_connect("/ws/motrix?robot_id=r1pro-mock") as ws:
            hello = ws.receive_json()
            assert hello["read_only"] is True
            ws.send_text("null")
            resp1 = ws.receive_json()
            assert resp1["code"] == "invalid_json"
            ws.send_text("[1, 2]")
            resp2 = ws.receive_json()
            assert resp2["code"] == "invalid_json"
        # After disconnect, client_count MUST return to 0 (no leaked dead sockets)
        assert app.state.ctx.motrix_bridge.client_count == 0
