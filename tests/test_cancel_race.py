"""Deterministic short-job cancellation boundaries; no devices, network, or Lease."""
import asyncio
from unittest.mock import patch
import pytest
from fastapi.testclient import TestClient
from motrix_robot_web.app import create_app
from motrix_robot_web.connectors.mock import build_default_mock_connectors
from motrix_robot_web.safety import RobotSafetyGate, SafetyError
from motrix_robot_web.storage import LocalStore
from motrix_robot_web.workflow_engine import WorkflowRunner


def fixture(tmp_path, steps=1):
    store = LocalStore(tmp_path)
    connector = build_default_mock_connectors()["dex-w1-mock"]
    gate = RobotSafetyGate(connector.descriptor)
    store.upsert_action({"name":"short_pose", "robot_id":connector.descriptor.id, "kind":"joint_targets", "params":{"targets":{"ANKLE_PITCH":-.65}}})
    wf = store.upsert_workflow({"name":"short_cancel_race", "robot_id":connector.descriptor.id, "steps":[{"action_name":"short_pose", "duration_s":.01} for _ in range(steps)]})
    return store, connector, gate, wf, WorkflowRunner(store)


@pytest.mark.asyncio
async def test_cancel_before_worker_first_turn_is_persisted_and_never_overwritten(tmp_path):
    store, conn, gate, wf, runner = fixture(tmp_path)
    with patch.object(runner, "_dispatch_action", side_effect=AssertionError("motion")) as dispatch, patch.object(conn, "safe_stop", side_effect=AssertionError("dry-run stop")) as stop, patch.object(gate,"acquire_lease",side_effect=AssertionError("Lease")) as lease:
        job = runner.start_workflow(wf, conn, gate, dry_run=True)
        worker = runner._tasks[job.job_id]
        result = runner.cancel_job(job.job_id)
        assert result.state == "Canceled" and result.exit_code == 130
        assert LocalStore(tmp_path).get_job(job.job_id).state == "Canceled"
        await worker
        assert job.state == "Canceled" and job.steps[-1].phase == "canceled"
        assert any("Cancel requested" in log for log in job.logs)
        assert any("Canceled" in log for log in job.logs)
        assert not any("Finished cleanly" in log for log in job.logs)
        assert dispatch.call_count == stop.call_count == lease.call_count == 0
        assert not gate.snapshot().lease_active
        assert runner._cancel_events == {} and runner._cancel_flags == set()


@pytest.mark.asyncio
async def test_cancel_at_last_await_of_short_job_wins_over_success_commit(tmp_path):
    store, conn, gate, wf, runner = fixture(tmp_path)
    original_wait = runner._wait_or_cancel
    calls = 0
    async def cancel_at_final_wait(job, seconds):
        nonlocal calls
        calls += 1
        await original_wait(job, seconds)
        # One short action has two 100ms chunks. This reproduces the old hole:
        # Cancel arrives at the final await, just before success is committed.
        if calls == 2:
            assert runner.cancel_job(job.job_id).state == "Canceled"
    with patch.object(runner, "_wait_or_cancel", side_effect=cancel_at_final_wait), patch.object(runner, "_dispatch_action", side_effect=AssertionError("motion")) as dispatch:
        job = runner.start_workflow(wf, conn, gate, dry_run=True)
        await runner._tasks[job.job_id]
    assert calls == 2
    assert store.get_job(job.job_id).state == "Canceled"
    assert job.exit_code == 130 and job.steps[-1].phase == "canceled"
    assert not any("Finished cleanly" in log for log in job.logs)
    assert dispatch.call_count == 0 and not gate.snapshot().lease_active


@pytest.mark.asyncio
async def test_cancel_interrupts_wait_and_skips_following_actions(tmp_path):
    store, conn, gate, wf, runner = fixture(tmp_path, steps=2)
    entered = asyncio.Event()
    original_wait = runner._wait_or_cancel
    async def observe_wait(job, seconds):
        entered.set()
        await original_wait(job, 30)  # would hang unless cancellation wakes it
    with patch.object(runner, "_wait_or_cancel", side_effect=observe_wait), patch.object(runner, "_dispatch_action", side_effect=AssertionError("motion")) as dispatch:
        job = runner.start_workflow(wf, conn, gate, dry_run=True)
        worker = runner._tasks[job.job_id]
        await entered.wait()
        assert runner.cancel_job(job.job_id).state == "Canceled"
        await asyncio.wait_for(worker, timeout=.5)
    assert job.steps[2].phase == "canceled"
    assert not job.steps[2].started_at
    assert dispatch.call_count == 0


@pytest.mark.asyncio
async def test_finished_job_is_not_falsely_reported_as_canceled(tmp_path):
    store, conn, gate, wf, runner = fixture(tmp_path)
    job = runner.start_workflow(wf, conn, gate, dry_run=True)
    await runner._tasks[job.job_id]
    with pytest.raises(SafetyError) as error:
        runner.cancel_job(job.job_id)
    assert error.value.status_code == 409 and error.value.code == "job_already_finished"
    assert job.state == "Succeeded" and runner._cancel_flags == set()


@pytest.mark.parametrize("delay", [0, .08, .16])
def test_api_short_running_cancel_response_and_persisted_list_are_canceled(tmp_path, delay):
    app = create_app(data_dir=tmp_path)
    with TestClient(app) as client, patch.object(app.state.ctx.runner, "_dispatch_action", side_effect=AssertionError("motion")) as dispatch:
        rid="dex-w1-mock"
        action=client.post("/api/actions",json={"name":"short_pose", "robot_id":rid, "kind":"joint_targets", "params":{"targets":{"ANKLE_PITCH":-.65}}}).json()["action"]
        wf=client.post("/api/workflows",json={"name":"short_api", "robot_id":rid, "steps":[{"action_name":action["name"], "duration_s":.01}]}).json()["workflow"]
        job=client.post(f"/api/workflows/{wf['id']}/run",json={"dry_run":True}).json()["job"]
        client.portal.call(asyncio.sleep, delay)
        response=client.post(f"/api/jobs/{job['job_id']}/cancel")
        assert response.status_code == 200
        assert response.json()["job"]["state"] == "Canceled"
        client.portal.call(asyncio.sleep, .25)
        final=client.get(f"/api/jobs/{job['job_id']}").json()["job"]
        assert final["state"] == "Canceled" and final["exit_code"] == 130
        assert any(j["job_id"]==job["job_id"] for j in client.get("/api/jobs?state=Canceled").json()["jobs"])
        assert client.post(f"/api/jobs/{job['job_id']}/cancel").json()["job"]["state"] == "Canceled"
        assert dispatch.call_count == 0
        assert not client.get(f"/api/robots/{rid}/state").json()["authority"]["lease_active"]


def test_api_cancel_after_completion_returns_explicit_409_not_fake_acceptance(tmp_path):
    app=create_app(data_dir=tmp_path)
    with TestClient(app) as client, patch.object(app.state.ctx.runner,"_dispatch_action",side_effect=AssertionError("motion")) as dispatch:
        rid="dex-w1-mock"
        client.post("/api/actions",json={"name":"short_pose", "robot_id":rid, "kind":"joint_targets", "params":{"targets":{"ANKLE_PITCH":-.65}}})
        wf=client.post("/api/workflows",json={"name":"already_done", "robot_id":rid, "steps":[{"action_name":"short_pose", "duration_s":.01}]}).json()["workflow"]
        job=client.post(f"/api/workflows/{wf['id']}/run",json={"dry_run":True}).json()["job"]
        client.portal.call(asyncio.sleep,.3)
        response=client.post(f"/api/jobs/{job['job_id']}/cancel")
        assert response.status_code==409 and response.json()["code"]=="job_already_finished"
        assert client.get(f"/api/jobs/{job['job_id']}").json()["job"]["state"]=="Succeeded"
        assert app.state.ctx.runner._cancel_flags==set()
        assert dispatch.call_count==0
