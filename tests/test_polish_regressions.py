"""Polish v6 backend regressions: fault recovery, favicon, Motrix per-robot WS, ArUco & workflow CRUD."""
from __future__ import annotations

import pytest
from fastapi.testclient import TestClient

from motrix_robot_web.app import create_app


@pytest.fixture
def client(tmp_path):
    with TestClient(create_app(data_dir=tmp_path)) as c:
        yield c


def acquire(client: TestClient, rid: str = "r1pro-mock") -> dict:
    res = client.post(f"/api/robots/{rid}/lease/acquire", json={"operator_name": "tester", "confirm_safety": True})
    assert res.status_code == 200
    return res.json()["authority"]


def test_favicon_returns_200_svg(client: TestClient):
    res = client.get("/favicon.ico")
    assert res.status_code == 200
    assert "image/svg+xml" in res.headers["content-type"]
    assert b"<svg" in res.content


@pytest.mark.parametrize("fault_type", ["overheat", "motor_error", "stale_feedback"])
def test_fault_injection_blocks_motion_and_clear_recovers(client: TestClient, fault_type: str):
    acquire(client, "r1pro-mock")

    # Inject fault
    inj = client.post(f"/api/robots/r1pro-mock/faults/{fault_type}")
    assert inj.status_code == 200

    # Motion is strictly blocked while fault is active
    blocked = client.post(
        "/api/robots/r1pro-mock/joints/targets",
        json={"targets": {"torso_1": 0.1}, "duration_s": 1.0},
    )
    assert blocked.status_code == 409

    # Clear fault MUST succeed under active lease (not deadlocked by the injected fault)
    clr = client.post("/api/robots/r1pro-mock/faults/clear")
    assert clr.status_code == 200
    assert clr.json()["cleared"] is True

    # Motion works again after clearing
    ok = client.post(
        "/api/robots/r1pro-mock/joints/targets",
        json={"targets": {"torso_1": 0.1}, "duration_s": 1.0},
    )
    assert ok.status_code == 200


def test_motrix_ws_robot_subscription_and_write_rejection(client: TestClient):
    with client.websocket_connect("/ws/motrix?robot_id=dex-w1-mock") as ws:
        hello = ws.receive_json()
        assert hello["type"] == "hello"
        assert hello["read_only"] is True
        assert hello["robot_id"] == "dex-w1-mock"

        ws.send_json({"type": "subscribe", "robot_id": "dex-w1-mock"})
        ack = ws.receive_json()
        assert ack["type"] == "ack"
        assert ack["read_only"] is True

        ws.send_json({"type": "set_joints", "targets": {"WAIST_YAW": 0.5}})
        rej = ws.receive_json()
        assert rej["type"] == "rejected"
        assert rej["code"] == "readonly_mirror_violation"


def test_aruco_pose_and_fine_alignment_lifecycle(client: TestClient):
    acquire(client, "r1pro-mock")

    # Fine align before starting pose -> 409
    pre = client.post("/api/robots/r1pro-mock/perception/aruco_set_fine", json={"enabled": True})
    assert pre.status_code == 409
    assert pre.json()["code"] == "aruco_pose_required"

    # Start ArUco pose -> detects marker #1
    started = client.post("/api/robots/r1pro-mock/perception/aruco_start_pose", json={})
    assert started.status_code == 200
    perc = started.json()["perception"]
    assert perc["aruco_pose_running"] is True
    assert len(perc["aruco_markers"]) >= 1

    # Enable fine align -> succeeds and narrows lateral error
    fine = client.post("/api/robots/r1pro-mock/perception/aruco_set_fine", json={"enabled": True})
    assert fine.status_code == 200
    assert fine.json()["perception"]["aruco_fine_enabled"] is True

    # Stop ArUco pose (allowed even without write check)
    stopped = client.post("/api/robots/r1pro-mock/perception/aruco_stop_pose", json={})
    assert stopped.status_code == 200
    assert stopped.json()["perception"]["aruco_pose_running"] is False


def test_workflow_crud_and_visual_step_order(client: TestClient):
    created = client.post(
        "/api/workflows",
        json={
            "name": "custom_visual_flow",
            "robot_id": "r1pro-mock",
            "kind": "Workflow",
            "description": "Created from visual builder",
            "steps": [
                {"id": "s1", "action_name": "r1_safe_home", "delay_s": 0.0, "duration_s": 0.5},
                {"id": "s2", "action_name": "r1_pregrasp_left", "delay_s": 0.1, "duration_s": 0.6},
            ],
        },
    )
    assert created.status_code == 200
    wf = created.json()["workflow"]
    wf_id = wf["id"]
    assert len(wf["steps"]) == 2

    # Update workflow keeping same ID with reordered steps
    updated = client.post(
        "/api/workflows",
        json={
            "id": wf_id,
            "name": "custom_visual_flow_v2",
            "robot_id": "r1pro-mock",
            "kind": "Workflow",
            "steps": [
                {"id": "s2", "action_name": "r1_pregrasp_left", "delay_s": 0.0, "duration_s": 0.4},
                {"id": "s1", "action_name": "r1_safe_home", "delay_s": 0.0, "duration_s": 0.4},
            ],
        },
    )
    assert updated.status_code == 200
    assert updated.json()["workflow"]["id"] == wf_id
    assert updated.json()["workflow"]["steps"][0]["action_name"] == "r1_pregrasp_left"

    # Delete workflow
    deleted = client.delete(f"/api/workflows/{wf_id}")
    assert deleted.status_code == 200
    assert deleted.json()["deleted"] is True
