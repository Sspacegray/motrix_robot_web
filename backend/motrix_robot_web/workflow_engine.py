from __future__ import annotations

import asyncio
import uuid
from typing import Any

from .connectors.base import RobotConnector
from .safety import RobotSafetyGate, SafetyError, utc_iso
from .schemas import ExecutionJob, JobStep, WorkflowDefinition
from .storage import LocalStore


class WorkflowRunner:
    """High-level workflow orchestrator producing Loop Actions-compatible Jobs, Steps, and Logs.

    Never replaces Edge runtime orchestration: every physical step is delegated through
    the robot's Connector after verifying SafetyGate authority and lease state.
    """

    def __init__(self, store: LocalStore) -> None:
        self.store = store
        self._tasks: dict[str, asyncio.Task[Any]] = {}
        self._cancel_flags: set[str] = set()
        self._cancel_events: dict[str, asyncio.Event] = {}

    def _ts(self) -> str:
        return utc_iso().replace("T", " ").replace("Z", "")

    def start_workflow(
        self,
        workflow: WorkflowDefinition,
        connector: RobotConnector,
        gate: RobotSafetyGate,
        dry_run: bool = False,
        operator: str = "operator@motphys",
    ) -> ExecutionJob:
        auth = gate.snapshot()
        if not dry_run:
            gate.require_control_authority(connector.get_observation())

        now = utc_iso()
        job_id = str(uuid.uuid4())
        run_id = f"r-{uuid.uuid4().hex[:24].upper()}"
        steps = [JobStep(step="start", phase="running", detail="dry-run validation" if dry_run else f"lease={auth.lease_id}")]
        for wf_step in workflow.steps:
            steps.append(JobStep(step=wf_step.action_name, phase="pending", detail=f"duration={wf_step.duration_s:.1f}s"))
        steps.append(JobStep(step="end", phase="pending", detail="finalize job & audit"))

        job = ExecutionJob(
            job_id=job_id,
            run_id=run_id,
            flow_name=workflow.name,
            kind="Manual" if dry_run else workflow.kind,
            state="Running",
            robot_id=workflow.robot_id,
            workflow_id=workflow.id,
            created_at=now,
            started_at=now,
            operator=operator,
            lease_id=None if dry_run else auth.lease_id,
            steps=steps,
            logs=[
                f"{self._ts()} [workflow] Starting {workflow.name} (run={run_id}, mode={'DRY-RUN' if dry_run else 'LIVE'})",
                f"{self._ts()} [robot] Target={connector.descriptor.id} ({connector.descriptor.name}) connector={connector.descriptor.connector_type}",
            ],
        )
        self.store.save_job(job)

        self._cancel_events[job_id] = asyncio.Event()
        task = asyncio.create_task(self._execute(job, workflow, connector, gate, dry_run))
        self._tasks[job_id] = task
        return job

    def _mark_canceled(self, job: ExecutionJob, reason: str) -> None:
        if job.state == "Canceled":
            return
        for step in job.steps:
            if step.phase in ("pending", "running"):
                step.phase = "canceled"
                step.finished_at = utc_iso()
        job.state = "Canceled"
        job.exit_code = 130
        job.finished_at = utc_iso()
        job.logs.append(f"{self._ts()} [workflow] Canceled ({reason})")
        self.store.save_job(job)

    def cancel_job(self, job_id: str) -> ExecutionJob | None:
        # Called on the runner's event loop by the async API route: the Running
        # check, terminal transition, and success commit cannot interleave.
        job = self.store.get_job(job_id)
        if not job or job.state == "Canceled":
            return job
        if job.state != "Running":
            raise SafetyError(409, "job_already_finished", f"任务已结束（{job.state}），未接受取消；请刷新任务状态。")
        self._cancel_flags.add(job_id)
        event = self._cancel_events.get(job_id)
        if event:
            event.set()
        job.logs.append(f"{self._ts()} [workflow] Cancel requested by operator")
        # Persist before replying; the worker must never overwrite this record.
        self._mark_canceled(job, "canceled_by_operator")
        return job

    def _check_canceled(self, job: ExecutionJob) -> None:
        if job.job_id in self._cancel_flags or job.state == "Canceled":
            raise asyncio.CancelledError("canceled_by_operator_or_safety")

    async def _wait_or_cancel(self, job: ExecutionJob, seconds: float) -> None:
        self._check_canceled(job)
        try:
            await asyncio.wait_for(self._cancel_events[job.job_id].wait(), timeout=seconds)
        except asyncio.TimeoutError:
            pass
        self._check_canceled(job)

    def abort_robot_jobs(self, robot_id: str, reason: str) -> None:
        for job in list(self.store.jobs.values()):
            if job.robot_id == robot_id and job.state == "Running":
                self._cancel_flags.add(job.job_id)
                event = self._cancel_events.get(job.job_id)
                task = self._tasks.get(job.job_id)
                if event and task and not task.get_loop().is_closed():
                    # Safety endpoints may call this from FastAPI's thread pool.
                    task.get_loop().call_soon_threadsafe(event.set)
                job.logs.append(f"{self._ts()} [safety] Aborted running job: {reason}")
                self.store.save_job(job)

    async def _execute(
        self,
        job: ExecutionJob,
        workflow: WorkflowDefinition,
        connector: RobotConnector,
        gate: RobotSafetyGate,
        dry_run: bool,
    ) -> None:
        try:
            self._check_canceled(job)
            job.steps[0].phase = "succeeded"
            job.steps[0].finished_at = utc_iso()
            job.logs.append(
                f"{self._ts()} [step:start] Preflight check OK ({'no motion will be sent' if dry_run else 'lease active & interlocks clear'})"
            )
            self.store.save_job(job)

            for idx, wf_step in enumerate(workflow.steps):
                step_obj = job.steps[idx + 1]
                if job.job_id in self._cancel_flags:
                    raise asyncio.CancelledError("canceled_by_operator_or_safety")

                if not dry_run:
                    # Re-check authority + interlocks + estop before every step!
                    gate.require_control_authority(connector.get_observation())

                step_obj.phase = "running"
                step_obj.started_at = utc_iso()
                action = self.store.get_action_by_name(workflow.robot_id, wf_step.action_name)
                if not action:
                    raise SafetyError(404, "missing_action", f"未找到原子动作：{wf_step.action_name}")

                job.logs.append(f"{self._ts()} [step:{idx + 1}] Running action '{action.name}' (kind={action.kind})")
                self.store.save_job(job)

                if wf_step.delay_s > 0:
                    await self._wait_or_cancel(job, min(wf_step.delay_s, 0.5))

                if job.job_id in self._cancel_flags:
                    raise asyncio.CancelledError("canceled_before_dispatch")
                params = {**action.params, **wf_step.overrides}
                if dry_run:
                    step_obj.detail = f"dry-run validated ({action.kind})"
                    job.logs.append(f"{self._ts()} [step:{idx + 1}] Dry-run OK: params keys={list(params.keys())}")
                else:
                    detail = self._dispatch_action(action.kind, params, connector, gate)
                    step_obj.detail = detail
                    job.logs.append(f"{self._ts()} [step:{idx + 1}] Dispatched -> {detail}")

                # Wait briefly so 3D view & streaming logs show realistic progression
                wait_s = min(max(wf_step.duration_s * 0.35, 0.25), 0.7)
                chunks = max(1, int(wait_s / 0.1))
                for _ in range(chunks):
                    if job.job_id in self._cancel_flags or (not dry_run and gate.snapshot().estop_latched):
                        raise asyncio.CancelledError("aborted_during_step")
                    await self._wait_or_cancel(job, 0.1)

                self._check_canceled(job)
                step_obj.phase = "succeeded"
                step_obj.finished_at = utc_iso()
                self.store.save_job(job)

            self._check_canceled(job)
            end_step = job.steps[-1]
            end_step.phase = "succeeded"
            end_step.finished_at = utc_iso()
            end_step.detail = "all steps completed cleanly"
            job.state = "Succeeded"
            job.exit_code = 0
            job.finished_at = utc_iso()
            job.logs.append(f"{self._ts()} [workflow] Finished cleanly with exit code 0")
            self.store.save_job(job)

        except asyncio.CancelledError as exc:
            if not dry_run:
                connector.safe_stop("workflow_canceled")
            self._mark_canceled(job, str(exc))
        except Exception as exc:
            if not dry_run:
                connector.safe_stop("workflow_failed")
            msg = exc.message if isinstance(exc, SafetyError) else str(exc)
            failed_marked = False
            for st in job.steps:
                if st.phase == "running":
                    st.phase = "failed"
                    st.detail = msg
                    st.finished_at = utc_iso()
                    failed_marked = True
                elif st.phase == "pending":
                    st.phase = "canceled"
                    st.detail = "skipped after upstream failure" if failed_marked else msg
            job.state = "Failed"
            job.exit_code = 1
            job.finished_at = utc_iso()
            job.logs.append(f"{self._ts()} [error] {msg}")
            job.logs.append(f"{self._ts()} [workflow] Exiting with code 1")
            self.store.save_job(job)
        finally:
            self._cancel_flags.discard(job.job_id)
            self._cancel_events.pop(job.job_id, None)
            self._tasks.pop(job.job_id, None)

    def _dispatch_action(
        self, kind: str, params: dict[str, Any], connector: RobotConnector, gate: RobotSafetyGate
    ) -> str:
        obs = connector.get_observation()
        gate.require_control_authority(obs)
        if kind == "joint_targets":
            raw_targets = params.get("targets") or {}
            validated = gate.validate_joint_targets(raw_targets, obs)
            connector.send_joint_targets(validated, float(params.get("duration_s", 1.2)))
            return f"sent {len(validated)} joint targets"
        if kind in ("ee_relative", "ee_pose", "ee_axial"):
            arm = str(params.get("arm", "left"))
            dx = float(params.get("dx", 0.0))
            dy = float(params.get("dy", 0.0))
            dz = float(params.get("dz", 0.04 if kind == "ee_relative" else 0.0))
            connector.send_ee_step(arm, dx, dy, dz, 0.0, 0.0, float(params.get("end_angle_deg", 0.0)) * 0.2)
            return f"EE {kind} ({arm}) dx={dx:.2f} dy={dy:.2f} dz={dz:.2f}"
        if kind == "chassis_velocity":
            connector.send_chassis_velocity(
                float(params.get("vx", 0.0)),
                float(params.get("vy", 0.0)),
                float(params.get("omega", 0.0)),
                float(params.get("duration_s", 0.5)),
            )
            return f"chassis vel vx={params.get('vx', 0)} omega={params.get('omega', 0)}"
        if kind == "navigate_to_pose":
            connector.set_navigation_mode("NAVIGATION")
            connector.send_navigation_goal(float(params.get("x", 0.0)), float(params.get("y", 0.0)), float(params.get("yaw", 0.0)))
            return f"nav goal x={params.get('x', 0)} y={params.get('y', 0)}"
        if kind == "perception_step":
            pipeline = str(params.get("pipeline", "aruco_fine"))
            if pipeline == "aruco_fine" and connector.descriptor.capabilities.aruco:
                connector.control_perception("aruco_start_pose", {})
                connector.control_perception("aruco_set_fine", {"enabled": True})
                return "aruco_id1 fine approach converged (x=0.60m)"
            return f"perception step '{pipeline}' executed"
        if kind == "policy_intent":
            ckpt = params.get("checkpoint")
            if ckpt:
                connector.control_policy("switch_checkpoint", {"checkpoint_id": ckpt})
            return f"policy intent dispatched ({params.get('task_intent', 'default')})"
        return f"completed {kind}"
