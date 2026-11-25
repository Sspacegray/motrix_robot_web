"""Regression tests for Priority #1:
- Unified Preview == Live Dispatch (including ee_axial via URDF 6D IK)
- Online IK convergence guard (reject unreachable poses before motion)
- Real closed-loop settlement (joints & navigation) + per-step timeout abort
- Pause / Step-by-Step breakpoint / Resume / Cancel-while-paused / E-Stop-while-paused
"""
import asyncio
import time
import pytest
from fastapi.testclient import TestClient

from motrix_robot_web.app import create_app
from motrix_robot_web.connectors.mock import build_default_mock_connectors
from motrix_robot_web.kinematics import RobotKinematics
from motrix_robot_web.safety import RobotSafetyGate
from motrix_robot_web.storage import LocalStore
from motrix_robot_web.workflow_engine import WorkflowRunner


def _acquire_lease(client: TestClient, robot_id: str = "r1pro-mock") -> str:
    resp = client.post(
        f"/api/robots/{robot_id}/lease/acquire",
        json={"operator_name": "tester@motphys", "task_permission": "operator", "confirm_safety": True},
    )
    assert resp.status_code == 200
    return resp.json()["authority"]["lease_id"]


def _wait_job_state(client: TestClient, job_id: str, targets: set[str], timeout_s: float = 4.0) -> dict:
    deadline = time.monotonic() + timeout_s
    last = None
    while time.monotonic() < deadline:
        last = client.get(f"/api/jobs/{job_id}").json()["job"]
        if last["state"] in targets:
            return last
        time.sleep(0.06)
    raise AssertionError(f"Job {job_id} stayed in state={last['state'] if last else None}, expected {targets}")


def test_ee_axial_preview_and_live_dispatch_use_same_urdf_ik_and_settle_closed_loop(tmp_path):
    with TestClient(create_app(data_dir=tmp_path)) as client:
        wf = client.post(
            "/api/workflows",
            json={
                "robot_id": "r1pro-mock",
                "name": "axial_parity_check",
                "steps": [
                    {
                        "id": "ax1",
                        "action_name": "ee_axial_pour_template",
                        "delay_s": 0.0,
                        "duration_s": 0.8,
                        "overrides": {"arm": "right", "delta_xyz_m": [0.04, 0.0, 0.03], "end_angle_deg": 25.0},
                    }
                ],
            },
        ).json()["workflow"]

        prev = client.post(f"/api/workflows/{wf['id']}/preview")
        assert prev.status_code == 200
        preview_goal = prev.json()["plan"]["goal"]

        _acquire_lease(client, "r1pro-mock")
        run = client.post(f"/api/workflows/{wf['id']}/run", json={"dry_run": False})
        assert run.status_code == 200
        job = _wait_job_state(client, run.json()["job"]["job_id"], {"Succeeded", "Failed"}, timeout_s=4.0)
        assert job["state"] == "Succeeded", f"unexpected logs: {job['logs']}"
        assert any("IK ee_axial (right)" in line for line in job["logs"])
        assert any("Settled (joints max_err=" in line for line in job["logs"])
        assert "闭环到位" in job["steps"][1]["detail"]

        # Verify actual settled right-arm joints match preview IK goal tightly!
        obs = client.get("/api/robots/r1pro-mock/state").json()["observation"]
        for i in range(1, 8):
            jn = f"right_arm_{i}"
            assert abs(obs["joint_positions"][jn] - preview_goal[jn]) <= 0.03


def test_online_ik_convergence_guard_blocks_unreachable_pose_before_dispatch(tmp_path):
    with TestClient(create_app(data_dir=tmp_path)) as client:
        wf = client.post(
            "/api/workflows",
            json={
                "robot_id": "r1pro-mock",
                "name": "unreachable_ik_guard_test",
                "steps": [
                    {
                        "id": "bad_ik",
                        "action_name": "ee_pose_template",
                        "delay_s": 0.0,
                        "duration_s": 1.0,
                        "ik_max_err_mm": 15.0,
                        "overrides": {"arm": "right", "xyz_source": "manual", "xyz_m": [2.8, -2.5, 2.5], "offset_xyz_m": [0.0, 0.0, 0.0]},
                    }
                ],
            },
        ).json()["workflow"]

        prev = client.post(f"/api/workflows/{wf['id']}/preview")
        assert prev.status_code == 200
        assert prev.json()["steps"][0]["ik_ok"] is False
        assert prev.json()["step_segments"][0]["ik_ok"] is False

        before_q = dict(client.get("/api/robots/r1pro-mock/state").json()["observation"]["joint_positions"])
        _acquire_lease(client, "r1pro-mock")
        run = client.post(f"/api/workflows/{wf['id']}/run", json={"dry_run": False})
        assert run.status_code == 200
        job = _wait_job_state(client, run.json()["job"]["job_id"], {"Failed", "Succeeded"}, timeout_s=3.0)
        assert job["state"] == "Failed"
        assert job["exit_code"] == 1
        assert any("在线 IK 复核未收敛" in line for line in job["logs"])

        after_q = client.get("/api/robots/r1pro-mock/state").json()["observation"]["joint_positions"]
        assert max(abs(after_q[k] - before_q[k]) for k in before_q) < 1e-4


@pytest.mark.asyncio
async def test_closed_loop_settlement_timeout_aborts_stuck_actuator_before_next_step(tmp_path):
    store = LocalStore(tmp_path)
    conn = build_default_mock_connectors()["r1pro-mock"]
    gate = RobotSafetyGate(conn.descriptor)
    kin_map = {"r1pro-mock": RobotKinematics(conn.descriptor)}
    runner = WorkflowRunner(store, kinematics_map=kin_map)

    wf = store.upsert_workflow(
        {
            "robot_id": "r1pro-mock",
            "name": "stuck_actuator_timeout_test",
            "steps": [
                {"id": "s1", "action_name": "钩箱躯干姿态_v1", "delay_s": 0.0, "duration_s": 0.5, "timeout_s": 0.55},
                {"id": "s2", "action_name": "r1_safe_home", "delay_s": 0.0, "duration_s": 0.5, "timeout_s": 2.0},
            ],
        }
    )

    gate.acquire_lease("tester@motphys", confirm_safety=True)
    # Intentionally DO NOT call conn.step(): joint positions never move -> must time out on step 1!
    job = runner.start_workflow(wf, conn, gate, dry_run=False, operator="tester@motphys")
    await asyncio.wait_for(runner._tasks[job.job_id], timeout=2.5)

    final_job = store.get_job(job.job_id)
    assert final_job.state == "Failed"
    assert final_job.exit_code == 1
    assert final_job.steps[1].phase == "failed"
    assert final_job.steps[2].phase == "canceled"  # Never advanced blindly to step 2!
    assert any("闭环到位超时" in line for line in final_job.logs)


def test_step_by_step_breakpoint_step_resume_and_cancel_while_paused(tmp_path):
    with TestClient(create_app(data_dir=tmp_path)) as client:
        _acquire_lease(client, "r1pro-mock")

        # 1. Start 4-step workflow in step_mode=True
        run = client.post(
            "/api/workflows/wf-r1-fp-grasp-param-pass/run",
            json={"dry_run": False, "step_mode": True},
        )
        assert run.status_code == 200
        jid = run.json()["job"]["job_id"]

        # Step 1 finishes -> automatically pauses before Step 2
        paused1 = _wait_job_state(client, jid, {"Paused", "Failed", "Succeeded"}, timeout_s=3.0)
        assert paused1["state"] == "Paused", f"expected Paused after step 1, got {paused1['state']}: {paused1['logs']}"
        assert paused1["steps"][1]["phase"] == "succeeded"
        assert paused1["steps"][2]["phase"] == "pending"
        assert "单步断点" in paused1["paused_reason"]

        # Concurrent Live workflow is blocked even while first job is Paused!
        dup = client.post("/api/workflows/wf-r1-box-transport/run", json={"dry_run": False})
        assert dup.status_code == 409
        assert dup.json()["code"] == "workflow_already_running"

        # 2. Click "Next Step" (/step) -> runs ONLY Step 2, settles closed-loop, pauses before Step 3!
        step_resp = client.post(f"/api/jobs/{jid}/step")
        assert step_resp.status_code == 200
        paused2 = _wait_job_state(client, jid, {"Paused", "Failed", "Succeeded"}, timeout_s=4.0)
        assert paused2["state"] == "Paused"
        assert paused2["steps"][2]["phase"] == "succeeded"
        assert "闭环到位" in paused2["steps"][2]["detail"]
        assert paused2["steps"][3]["phase"] == "pending"

        # 3. Click "Resume All" (/resume) -> completes Steps 3 & 4 continuously to Succeeded!
        res_resp = client.post(f"/api/jobs/{jid}/resume")
        assert res_resp.status_code == 200
        done = _wait_job_state(client, jid, {"Succeeded", "Failed"}, timeout_s=5.0)
        assert done["state"] == "Succeeded", f"unexpected logs: {done['logs']}"
        assert all(st["phase"] == "succeeded" for st in done["steps"])

        # 4. Verify Canceling a Paused job works instantaneously with clean teardown
        run2 = client.post(
            "/api/workflows/wf-r1-fp-grasp-param-pass/run",
            json={"dry_run": False, "step_mode": True},
        )
        jid2 = run2.json()["job"]["job_id"]
        _wait_job_state(client, jid2, {"Paused"}, timeout_s=3.0)
        cancel_resp = client.post(f"/api/jobs/{jid2}/cancel")
        assert cancel_resp.status_code == 200
        assert cancel_resp.json()["job"]["state"] == "Canceled"
        assert cancel_resp.json()["job"]["exit_code"] == 130

        # 5. Verify E-Stop while Paused aborts the job cleanly
        run3 = client.post(
            "/api/workflows/wf-r1-fp-grasp-param-pass/run",
            json={"dry_run": False, "step_mode": True},
        )
        jid3 = run3.json()["job"]["job_id"]
        _wait_job_state(client, jid3, {"Paused"}, timeout_s=3.0)
        est = client.post("/api/robots/r1pro-mock/estop", json={"source": "test", "reason": "estop_while_paused"})
        assert est.status_code == 200
        aborted = _wait_job_state(client, jid3, {"Canceled", "Failed"}, timeout_s=3.0)
        assert aborted["state"] == "Canceled"
