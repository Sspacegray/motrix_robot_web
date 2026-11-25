import pytest
from fastapi.testclient import TestClient
from motrix_robot_web.app import create_app


@pytest.fixture
def client(tmp_path):
    app = create_app(data_dir=tmp_path)
    with TestClient(app) as c:
        yield c


def test_api_health_and_robots(client):
    res = client.get("/api/health")
    assert res.status_code == 200
    data = res.json()
    assert data["status"] == "ok"
    assert data["motrix_bridge"]["read_only"] is True

    res = client.get("/api/robots")
    assert res.status_code == 200
    robots = res.json()["robots"]
    assert len(robots) == 3
    ids = {r["id"] for r in robots}
    assert ids == {"r1pro-mock", "piper-dual-mock", "dex-w1-mock"}


def test_api_state_and_events(client):
    res = client.get("/api/robots/r1pro-mock/state")
    assert res.status_code == 200
    state = res.json()
    assert state["descriptor"]["id"] == "r1pro-mock"
    assert "joint_positions" in state["observation"]
    assert "lease_active" in state["authority"]
    assert state["authority"]["lease_active"] is False


def test_api_lease_and_motion_security(client):
    # 1. Joint target without lease -> 403
    res = client.post(
        "/api/robots/r1pro-mock/joints/targets",
        json={"targets": {"torso_1": 0.0}, "duration_s": 1.0},
    )
    assert res.status_code == 403
    assert res.json()["code"] == "lease_required"

    # 2. Acquire lease without confirm -> 400
    res = client.post(
        "/api/robots/r1pro-mock/lease/acquire",
        json={"operator_name": "op1", "confirm_safety": False},
    )
    assert res.status_code == 400

    # 3. Acquire lease with confirm -> 200
    res = client.post(
        "/api/robots/r1pro-mock/lease/acquire",
        json={"operator_name": "op1", "confirm_safety": True},
    )
    assert res.status_code == 200
    auth = res.json()["authority"]
    assert auth["lease_active"] is True
    assert auth["lease_id"] is not None

    # 4. Joint target with lease -> 200
    res = client.post(
        "/api/robots/r1pro-mock/joints/targets",
        json={"targets": {"torso_1": 0.15}, "duration_s": 1.0},
    )
    assert res.status_code == 200

    # 5. Heartbeat -> 200
    res = client.post(
        "/api/robots/r1pro-mock/lease/heartbeat",
        json={"lease_id": auth["lease_id"]},
    )
    assert res.status_code == 200

    # 6. E-Stop bypass -> 200 and latched
    res = client.post(
        "/api/robots/r1pro-mock/estop",
        json={"source": "api_test", "reason": "test stop"},
    )
    assert res.status_code == 200
    assert res.json()["authority"]["estop_latched"] is True

    # 7. Clear E-Stop -> 200
    res = client.post(
        "/api/robots/r1pro-mock/estop/clear",
        json={"operator_name": "op1"},
    )
    assert res.status_code == 200
    assert res.json()["authority"]["estop_latched"] is False


def test_api_workflow_run_and_jobs(client):
    # Run workflow dry-run
    res = client.post("/api/workflows/wf-piper-plate-eval/run", json={"dry_run": True})
    assert res.status_code == 200
    job = res.json()["job"]
    assert job["flow_name"] == "piper_set_the_plates_eval_flow"
    assert job["kind"] == "Manual"

    # List jobs
    res = client.get("/api/jobs")
    assert res.status_code == 200
    jobs = res.json()["jobs"]
    assert len(jobs) >= 1
    assert any(j["job_id"] == job["job_id"] for j in jobs)


def test_api_r1pro_actions_import(client):
    payload = {
        "actions": [
            {
                "name": "test_import_pose",
                "type": "joint_targets",
                "params": {"targets": [{"group": "torso", "name": "torso[1]", "unit": "deg", "value": 15.0}]},
            }
        ]
    }
    res = client.post("/api/actions/import-r1pro?robot_id=r1pro-mock", json=payload)
    assert res.status_code == 200
    assert res.json()["imported_count"] == 1


def test_api_camera_svg(client):
    res = client.get("/api/robots/r1pro-mock/camera/head_left_rgb")
    assert res.status_code == 200
    assert "image/svg+xml" in res.headers["content-type"]
    assert b"<svg" in res.content


def test_static_frontend_and_assets(client):
    # 1. Index page
    res = client.get("/")
    assert res.status_code == 200
    assert b"Motphys Robot Web" in res.content
    assert b"tabPaneOverview" in res.content

    # 2. Main CSS & JS
    assert client.get("/css/style.css").status_code == 200
    assert client.get("/js/app.js").status_code == 200
    assert client.get("/js/api.js").status_code == 200
    assert client.get("/js/drawer.js").status_code == 200
    assert client.get("/js/tabs/overview.js").status_code == 200
    assert client.get("/js/tabs/joints.js").status_code == 200
    assert client.get("/js/tabs/actions.js").status_code == 200
    assert client.get("/js/tabs/navigation.js").status_code == 200
    assert client.get("/js/tabs/perception.js").status_code == 200
    assert client.get("/js/tabs/policy.js").status_code == 200
    assert client.get("/js/tabs/diagnostics.js").status_code == 200

    # 3. Vendor Three.js & URDFLoader
    assert client.get("/vendor/three/three.module.js").status_code == 200
    assert client.get("/vendor/urdf/URDFLoader.js").status_code == 200

    # 4. Map assets
    assert client.get("/assets/mx_map.png").status_code == 200
    assert client.get("/assets/mx_map.yaml").status_code == 200


def test_navigation_api_lifecycle(client):
    # Acquire lease first
    res = client.post("/api/robots/r1pro-mock/lease/acquire", json={"operator_name": "tester", "confirm_safety": True})
    assert res.status_code == 200

    # Mode switch to MANUAL
    res = client.post("/api/robots/r1pro-mock/navigation/mode", json={"mode": "MANUAL"})
    assert res.status_code == 200
    assert res.json()["mode"] == "MANUAL"

    # Send chassis manual velocity
    res = client.post(
        "/api/robots/r1pro-mock/navigation/chassis",
        json={"vx": 0.2, "vy": 0.0, "omega": 0.1, "duration_s": 0.4},
    )
    assert res.status_code == 200
    assert res.json()["accepted"] is True

    # Switch to NAVIGATION and send goal
    res = client.post("/api/robots/r1pro-mock/navigation/mode", json={"mode": "NAVIGATION"})
    assert res.status_code == 200
    assert res.json()["mode"] == "NAVIGATION"

    res = client.post("/api/robots/r1pro-mock/navigation/goal", json={"x": 1.5, "y": 0.8, "yaw": 0.5})
    assert res.status_code == 200
    assert res.json()["accepted"] is True

    # Cancel nav
    res = client.post("/api/robots/r1pro-mock/navigation/cancel")
    assert res.status_code == 200

    # Spin toggle
    res = client.post("/api/robots/r1pro-mock/navigation/spin", json={"active": True})
    assert res.status_code == 200


def test_perception_api_lifecycle(client):
    client.post("/api/robots/r1pro-mock/lease/acquire", json={"confirm_safety": True})
    # 1. Toggle depth ready first
    res = client.post("/api/robots/r1pro-mock/perception/set_head_depth", json={"enabled": True})
    assert res.status_code == 200
    assert res.json()["perception"]["head_depth_ready"] is True

    # 2. Pick point in 3D
    res = client.post("/api/robots/r1pro-mock/perception/pick_3d", json={"u": 240, "v": 135, "stream_key": "head_left_rgb"})
    assert res.status_code == 200
    assert "pick" in res.json()
    assert "base_link_xyz_m" in res.json()["pick"]

    # 3. Strict 3-step: start yolo -> latch segment -> foundation pose 6d
    res = client.post("/api/robots/r1pro-mock/perception/start_yolo", json={"camera": "head_left_rgb", "prompt": "bottle"})
    assert res.status_code == 200
    assert res.json()["perception"]["yolo_running"] is True

    res = client.post("/api/robots/r1pro-mock/perception/latch_yolo", json={})
    assert res.status_code == 200
    assert res.json()["perception"]["latched_mask"]["verified"] is True

    res = client.post("/api/robots/r1pro-mock/perception/start_foundation_pose", json={"classes": "bottle"})
    assert res.status_code == 200
    assert res.json()["perception"]["foundation_pose_running"] is True

    res = client.post("/api/robots/r1pro-mock/perception/stop_foundation_pose")
    assert res.status_code == 200

    res = client.post("/api/robots/r1pro-mock/perception/stop_yolo")
    assert res.status_code == 200


def test_policy_and_loop_records_api(client):
    # 1. Acquire lease
    client.post("/api/robots/r1pro-mock/lease/acquire", json={"operator_name": "tester", "confirm_safety": True})

    # 2. Policy start & stop
    res = client.post("/api/robots/r1pro-mock/policy/start_infer", json={})
    assert res.status_code == 200
    assert res.json()["infer_running"] is True

    res = client.post("/api/robots/r1pro-mock/policy/stop_infer")
    assert res.status_code == 200
    assert res.json()["infer_running"] is False

    # 3. Episode tagging (testing zero_success / no_result)
    res = client.post("/api/robots/r1pro-mock/policy/tag_episode", json={"outcome": "zero_success", "frames": 240})
    assert res.status_code == 200
    assert res.json()["episode"]["outcome"] == "zero_success"

    # 4. Loop records CRUD
    rec_payload = {
        "task_name": "api_test_task",
        "source_type": "training_run",
        "source_ref": "tr-202611-test-01",
        "model_artifact": "art-safetensors-test",
        "eval_plan": "plan-eval-10eps",
        "eval_status": "passed",
        "evidence_run_id": "s3://loop/test.mcap",
        "next_action": "Promote to Prod",
        "robot_id": "r1pro-mock",
    }
    res = client.post("/api/loop/records", json=rec_payload)
    assert res.status_code == 200
    rec_id = res.json()["record"]["record_id"]

    res = client.get("/api/loop/records?robot_id=r1pro-mock")
    assert res.status_code == 200
    records = res.json()["records"]
    assert any(r["record_id"] == rec_id for r in records)

    res = client.delete(f"/api/loop/records/{rec_id}")
    assert res.status_code == 200


