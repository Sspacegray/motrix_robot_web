import asyncio
import pytest
from motrix_robot_web.connectors.mock import build_default_mock_connectors
from motrix_robot_web.safety import RobotSafetyGate
from motrix_robot_web.storage import LocalStore
from motrix_robot_web.workflow_engine import WorkflowRunner


@pytest.mark.asyncio
async def test_workflow_runner_dry_run_and_live(tmp_path):
    store = LocalStore(tmp_path)
    runner = WorkflowRunner(store)
    conns = build_default_mock_connectors()
    r1 = conns["r1pro-mock"]
    gate = RobotSafetyGate(r1.descriptor)

    # Background stepping task to simulate 20Hz robot telemetry loop
    stop_stepping = False

    async def step_loop():
        while not stop_stepping:
            r1.step(0.05)
            await asyncio.sleep(0.05)

    step_task = asyncio.create_task(step_loop())

    try:
        wf = store.workflows["wf-r1-box-transport"]

        # 1. Dry run without lease should succeed and create an execution job with logs
        job_dry = runner.start_workflow(wf, r1, gate, dry_run=True, operator="test_operator")
        assert job_dry.kind == "Manual"
        assert job_dry.state == "Running"

        # Wait for dry run execution to finish
        await asyncio.sleep(1.8)
        job_dry = store.get_job(job_dry.job_id)
        assert job_dry.state == "Succeeded"
        assert job_dry.exit_code == 0
        assert any("Preflight check OK" in log for log in job_dry.logs)
        assert job_dry.steps[-1].phase == "succeeded"

        # 2. Live run without lease -> Rejected
        with pytest.raises(Exception):
            runner.start_workflow(wf, r1, gate, dry_run=False, operator="test_operator")

        # 3. Live run with lease -> Succeeded
        gate.acquire_lease("test_operator", confirm_safety=True)
        job_live = runner.start_workflow(wf, r1, gate, dry_run=False, operator="test_operator")
        assert job_live.kind == "Workflow"
        assert job_live.lease_id is not None

        await asyncio.sleep(1.8)
        job_live = store.get_job(job_live.job_id)
        assert job_live.state == "Succeeded"
        assert job_live.exit_code == 0
        assert any("Dispatched" in log for log in job_live.logs)
    finally:
        stop_stepping = True
        await step_task


@pytest.mark.asyncio
async def test_workflow_runner_cancellation_on_estop(tmp_path):
    store = LocalStore(tmp_path)
    runner = WorkflowRunner(store)
    conns = build_default_mock_connectors()
    r1 = conns["r1pro-mock"]
    gate = RobotSafetyGate(r1.descriptor)

    stop_stepping = False

    async def step_loop():
        while not stop_stepping:
            r1.step(0.05)
            await asyncio.sleep(0.05)

    step_task = asyncio.create_task(step_loop())

    try:
        gate.acquire_lease("test_operator", confirm_safety=True)
        wf = store.workflows["wf-r1-box-transport"]
        job = runner.start_workflow(wf, r1, gate, dry_run=False, operator="test_operator")

        # Trigger E-Stop mid-workflow
        await asyncio.sleep(0.3)
        gate.trigger_estop(source="test", reason="emergency abort")
        runner.abort_robot_jobs("r1pro-mock", "estop_triggered")

        await asyncio.sleep(0.8)
        final_job = store.get_job(job.job_id)
        assert final_job.state in ("Canceled", "Failed")
        assert any("Aborted" in log or "Canceled" in log for log in final_job.logs)
    finally:
        stop_stepping = True
        await step_task
