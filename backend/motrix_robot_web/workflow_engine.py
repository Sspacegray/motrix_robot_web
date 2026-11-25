from __future__ import annotations

import asyncio
import math
import time
import uuid
from typing import Any

from .connectors.base import RobotConnector
from .kinematics import RobotKinematics
from .safety import RobotSafetyGate, SafetyError, utc_iso
from .schemas import AtomicAction, ExecutionJob, JobStep, WorkflowDefinition, WorkflowStep
from .storage import LocalStore


# ---- R1 Pro Compatible Parameter-Passing Dependency Rules ----

def action_produces(action: AtomicAction | dict[str, Any], params_override: dict[str, Any] | None = None) -> set[str]:
    """Return workflow context keys produced by an action (compatible with r1pro_web_control)."""
    kind = action.kind if isinstance(action, AtomicAction) else str(action.get("kind") or action.get("type") or "")
    base_params = action.params if isinstance(action, AtomicAction) else (action.get("params") or {})
    params = {**base_params, **(params_override or {})}
    produced: set[str] = set()
    explicit = params.get("produces")
    if isinstance(explicit, list):
        produced.update(str(x) for x in explicit if x)
    if kind == "perception_step":
        pipeline = str(params.get("pipeline", ""))
        if pipeline in ("read_foundation_pose", "foundation_pose"):
            produced.update({"workflow_xyz", "workflow_pose"})
        elif pipeline in ("latch_yolo", "read_yolo", "yolo_latch"):
            produced.add("workflow_xyz")
        elif pipeline in ("aruco_fine", "read_aruco", "aruco_pose"):
            produced.update({"workflow_xyz", "workflow_pose", "aruco_pose"})
        elif pipeline == "read_ee_pose":
            produced.add("latched_ee_pose")
    return produced


def action_requires(action: AtomicAction | dict[str, Any], params_override: dict[str, Any] | None = None) -> set[str]:
    """Return workflow context keys required by an action before execution."""
    kind = action.kind if isinstance(action, AtomicAction) else str(action.get("kind") or action.get("type") or "")
    base_params = action.params if isinstance(action, AtomicAction) else (action.get("params") or {})
    params = {**base_params, **(params_override or {})}
    required: set[str] = set()
    explicit = params.get("requires")
    if isinstance(explicit, list):
        required.update(str(x) for x in explicit if x)
    if kind == "ee_pose":
        source = str(params.get("xyz_source", "manual"))
        if source in ("workflow_xyz", "workflow_pose", "aruco_pose"):
            required.add(source)
        elif source == "latched_yolo":
            required.add("workflow_xyz")
    elif kind == "ee_relative":
        source = str(params.get("reference_source", "current"))
        if source in ("workflow_pose", "latched_ee_pose"):
            required.add(source)
    return required


class WorkflowRunner:
    """High-level workflow orchestrator producing Loop Actions-compatible Jobs, Steps, and Logs.

    Supports R1 Pro style step-to-step parameter passing (produces/requires/workflow_values/overrides)
    and real URDF IK/trajectory preview without ever bypassing SafetyGate or Edge leases.
    """

    def __init__(self, store: LocalStore, kinematics_map: dict[str, RobotKinematics] | None = None) -> None:
        self.store = store
        self.kinematics_map: dict[str, RobotKinematics] = kinematics_map if kinematics_map is not None else {}
        self._tasks: dict[str, asyncio.Task[Any]] = {}
        self._cancel_flags: set[str] = set()
        self._cancel_events: dict[str, asyncio.Event] = {}
        self._pause_requested: set[str] = set()
        self._step_mode_jobs: set[str] = set()
        self._resume_events: dict[str, asyncio.Event] = {}
        self._job_connectors: dict[str, RobotConnector] = {}
        self._job_gates: dict[str, RobotSafetyGate] = {}

    def _ts(self) -> str:
        return utc_iso().replace("T", " ").replace("Z", "")

    def validate_workflow(self, workflow: WorkflowDefinition) -> dict[str, Any]:
        """Static dependency & parameter-passing validator (aligned with r1pro_web_control validate_sequence)."""
        if not workflow.steps:
            return {"ok": False, "message": "工作流不包含任何步骤", "step_contracts": []}
        available: set[str] = set()
        step_contracts: list[dict[str, Any]] = []
        for idx, st in enumerate(workflow.steps):
            act = self.store.get_action_by_name(workflow.robot_id, st.action_name)
            if act is None:
                return {
                    "ok": False,
                    "message": f"步骤 #{idx + 1}: 未找到原子动作 '{st.action_name}'",
                    "step_contracts": step_contracts,
                }
            merged = {**act.params, **(st.overrides or {})}
            req = action_requires(act, st.overrides)
            prod = action_produces(act, st.overrides)
            missing = sorted(req - available)
            step_contracts.append(
                {
                    "step_index": idx + 1,
                    "step_id": st.id,
                    "action_name": act.name,
                    "kind": act.kind,
                    "requires": sorted(req),
                    "produces": sorted(prod),
                    "missing": missing,
                    "overrides": dict(st.overrides or {}),
                    "resolved_source": merged.get("xyz_source") or merged.get("reference_source") or merged.get("pipeline") or "",
                }
            )
            if missing:
                return {
                    "ok": False,
                    "message": f"步骤 #{idx + 1} ({act.name}) 缺少上游参数依赖: {', '.join(missing)}（请在之前添加对应感知/读取步骤）",
                    "step_contracts": step_contracts,
                }
            available.update(prod)
        return {
            "ok": True,
            "message": f"依赖校验通过（共 {len(workflow.steps)} 步，产出上下文键: {', '.join(sorted(available)) or '无'}）",
            "available_outputs": sorted(available),
            "step_contracts": step_contracts,
        }

    def _resolve_perception_context(
        self, pipeline: str, params: dict[str, Any], connector: RobotConnector, workflow_values: dict[str, Any]
    ) -> str:
        obs = connector.get_observation()
        perc = obs.perception
        if pipeline in ("read_foundation_pose", "foundation_pose"):
            fp = perc.foundation_pose_result
            if fp and isinstance(fp.get("xyz_m"), list):
                xyz = [round(float(v), 4) for v in fp["xyz_m"][:3]]
                rpy = [round(float(v), 2) for v in (fp.get("rpy_deg") or [0.0, 0.0, 0.0])[:3]]
                src = "live_foundation_pose"
            else:
                # Physically reachable tabletop grasp pose in R1 Pro right-arm URDF frame
                xyz = [0.38, -0.36, 0.94]
                rpy = [-20.0, -35.0, 10.0]
                src = "mock_foundation_pose_snapshot"
            pose_obj = {"frame": "base_link", "xyz_m": xyz, "rpy_deg": rpy, "source": src}
            workflow_values["workflow_xyz"] = xyz
            workflow_values["workflow_pose"] = pose_obj
            return f"produces workflow_xyz={xyz}, workflow_pose (src={src})"

        if pipeline in ("latch_yolo", "read_yolo", "yolo_latch"):
            if perc.last_3d_pick and isinstance(perc.last_3d_pick.get("xyz_m"), list):
                xyz = [round(float(v), 4) for v in perc.last_3d_pick["xyz_m"][:3]]
                src = "last_3d_pick"
            else:
                xyz = [0.40, -0.34, 0.86]
                src = "mock_yolo_3d"
            workflow_values["workflow_xyz"] = xyz
            return f"produces workflow_xyz={xyz} (src={src})"

        if pipeline in ("aruco_fine", "read_aruco", "aruco_pose"):
            marker_xyz = [0.60, 0.0, 0.78]
            if perc.aruco_markers:
                m0 = perc.aruco_markers[0]
                if isinstance(m0.get("xyz_m"), list):
                    marker_xyz = [round(float(v), 4) for v in m0["xyz_m"][:3]]
            pose_obj = {"frame": "base_link", "xyz_m": marker_xyz, "rpy_deg": [0.0, 0.0, -90.0], "source": "aruco_id1"}
            workflow_values["workflow_xyz"] = marker_xyz
            workflow_values["workflow_pose"] = pose_obj
            workflow_values["aruco_pose"] = pose_obj
            return f"produces workflow_xyz={marker_xyz}, aruco_pose"

        if pipeline == "read_ee_pose":
            arm = str(params.get("arm", "right"))
            ee = obs.ee_poses.get(arm)
            pose_obj = {
                "frame": ee.frame if ee else "base_link",
                "xyz_m": [ee.x, ee.y, ee.z] if ee else [0.35, -0.35, 0.90],
                "rpy_deg": [ee.roll_deg, ee.pitch_deg, ee.yaw_deg] if ee else [0.0, 0.0, 0.0],
                "source": f"ee_{arm}",
            }
            workflow_values["latched_ee_pose"] = pose_obj
            return f"produces latched_ee_pose ({arm})={pose_obj['xyz_m']}"

        return f"perception '{pipeline}'"

    def _resolve_ee_target(
        self,
        kind: str,
        params: dict[str, Any],
        current_joints: dict[str, float],
        workflow_values: dict[str, Any],
        kin: RobotKinematics | None,
    ) -> tuple[str, list[float], list[float] | None, str]:
        """Single source of truth for ee_pose / ee_relative / ee_axial target resolution (Preview == Dry-run == Live)."""
        arm = str(params.get("arm", "right"))

        if kind == "ee_axial":
            raw_dxyz = params.get("delta_xyz_m")
            if isinstance(raw_dxyz, (list, tuple)) and len(raw_dxyz) >= 3:
                dxyz = [float(raw_dxyz[0]), float(raw_dxyz[1]), float(raw_dxyz[2])]
            else:
                dxyz = [float(params.get("dx", 0.0)), float(params.get("dy", 0.0)), float(params.get("dz", 0.0))]
            start_ang = float(params.get("start_angle_deg", 0.0))
            end_ang = float(params.get("end_angle_deg", 30.0))
            d_ang = end_ang - start_ang
            if not all(math.isfinite(v) for v in (*dxyz, start_ang, end_ang)):
                raise SafetyError(400, "invalid_number", "ee_axial 参数包含非有限值 (NaN/Inf)。")
            if max(abs(v) for v in dxyz) > 0.35 or abs(d_ang) > 180.0:
                raise SafetyError(400, "ee_step_exceeded", "ee_axial 位移或转角超出单步安全上限 (±0.35m / ±180°)。")
            if kin is not None:
                fk0 = kin.fk(arm, current_joints)
                base_xyz = list(fk0["position"])
                base_rpy = list(fk0["rpy_deg"])
            else:
                base_xyz = [0.38, -0.35, 0.90]
                base_rpy = [0.0, 0.0, 0.0]
            tgt_xyz = [round(base_xyz[i] + dxyz[i], 5) for i in range(3)]
            tgt_rpy = [round(base_rpy[0], 3), round(base_rpy[1], 3), round(base_rpy[2] + d_ang, 3)]
            note = f"axial rot {d_ang:+.1f}° + delta={[round(v, 3) for v in dxyz]} -> {tgt_xyz}"
            return arm, tgt_xyz, tgt_rpy, note

        offset = params.get("offset_xyz_m") or [
            float(params.get("dx", 0.0)),
            float(params.get("dy", 0.0)),
            float(params.get("dz", 0.0)),
        ]
        off_xyz = [float(offset[0]), float(offset[1]), float(offset[2])]
        if not all(math.isfinite(v) for v in off_xyz):
            raise SafetyError(400, "invalid_number", "末端偏置参数包含非有限值 (NaN/Inf)。")

        if kind == "ee_pose":
            src = str(params.get("xyz_source", "manual"))
            base_xyz = list(params.get("xyz_m") or [0.38, -0.35, 0.90])
            base_rpy = params.get("rpy_deg")
            if src == "workflow_pose" and isinstance(workflow_values.get("workflow_pose"), dict):
                wp = workflow_values["workflow_pose"]
                base_xyz = list(wp["xyz_m"])
                base_rpy = wp.get("rpy_deg", base_rpy)
            elif src == "aruco_pose" and isinstance(workflow_values.get("aruco_pose"), dict):
                ap = workflow_values["aruco_pose"]
                base_xyz = list(ap["xyz_m"])
                base_rpy = ap.get("rpy_deg", base_rpy)
            elif src in ("workflow_xyz", "latched_yolo") and isinstance(workflow_values.get("workflow_xyz"), list):
                base_xyz = list(workflow_values["workflow_xyz"])
            if not all(math.isfinite(float(v)) for v in base_xyz[:3]):
                raise SafetyError(400, "invalid_number", "ee_pose 目标坐标包含非有限值 (NaN/Inf)。")
            tgt_xyz = [round(float(base_xyz[i]) + off_xyz[i], 5) for i in range(3)]
            tgt_rpy = [float(v) for v in base_rpy[:3]] if isinstance(base_rpy, list) and len(base_rpy) >= 3 else None
            if tgt_rpy is not None and not all(math.isfinite(v) for v in tgt_rpy):
                raise SafetyError(400, "invalid_number", "ee_pose 姿态角包含非有限值 (NaN/Inf)。")
            note = f"src={src} base={[round(float(v), 3) for v in base_xyz]} + off={off_xyz} -> {tgt_xyz}"
            return arm, tgt_xyz, tgt_rpy, note

        # ee_relative
        ref_src = str(params.get("reference_source", "current"))
        if ref_src == "workflow_pose" and isinstance(workflow_values.get("workflow_pose"), dict):
            ref = workflow_values["workflow_pose"]
            base_xyz = list(ref["xyz_m"])
            base_rpy = ref.get("rpy_deg")
        elif ref_src == "latched_ee_pose" and isinstance(workflow_values.get("latched_ee_pose"), dict):
            ref = workflow_values["latched_ee_pose"]
            base_xyz = list(ref["xyz_m"])
            base_rpy = ref.get("rpy_deg")
        elif kin is not None:
            fk = kin.fk(arm, current_joints)
            base_xyz = fk["position"]
            base_rpy = fk["rpy_deg"]
        else:
            base_xyz = [0.38, -0.35, 0.90]
            base_rpy = [0.0, 0.0, 0.0]
        tgt_xyz = [round(float(base_xyz[i]) + off_xyz[i], 5) for i in range(3)]
        tgt_rpy = [float(v) for v in base_rpy[:3]] if isinstance(base_rpy, list) and len(base_rpy) >= 3 else None
        note = f"ref={ref_src} base={[round(float(v), 3) for v in base_xyz]} + delta={off_xyz} -> {tgt_xyz}"
        return arm, tgt_xyz, tgt_rpy, note

    def preview_workflow_trajectory(self, workflow: WorkflowDefinition, connector: RobotConnector) -> dict[str, Any]:
        """Compute full read-only multi-step joint/IK trajectory & parameter-passing preview for 3D Ghost + Trail."""
        validation = self.validate_workflow(workflow)
        if not validation["ok"]:
            raise SafetyError(422, "workflow_invalid", validation["message"])

        kin = self.kinematics_map.get(workflow.robot_id)
        if kin is None:
            raise SafetyError(400, "kinematics_unavailable", f"机型 {workflow.robot_id} 未配置 URDF 运动学求解器。")

        obs = connector.get_observation()
        cur_q = {j.name: float(obs.joint_positions.get(j.name, j.default)) for j in connector.descriptor.joints}
        start_q = dict(cur_q)
        workflow_values: dict[str, Any] = {}

        samples: list[dict[str, Any]] = [{"t": 0.0, "q": dict(cur_q), "step_index": 0}]
        elapsed = 0.0
        step_summaries: list[dict[str, Any]] = []
        step_segments: list[dict[str, Any]] = []
        last_ik_meta: dict[str, Any] | None = None
        step_palette = [
            ("#10b981", "#059669"),  # Emerald
            ("#00e5ff", "#0284c7"),  # Cyan / Azure
            ("#ff2e93", "#db2777"),  # Neon Magenta
            ("#f59e0b", "#d97706"),  # Solar Amber
            ("#a855f7", "#7c3aed"),  # Electric Violet
            ("#38bdf8", "#2563eb"),  # Sky / Royal Blue
        ]

        for idx, wf_step in enumerate(workflow.steps):
            act = self.store.get_action_by_name(workflow.robot_id, wf_step.action_name)
            if act is None:
                continue
            params = {**act.params, **(wf_step.overrides or {})}
            seg_dur = max(0.4, float(wf_step.duration_s or params.get("duration_s", 1.0)))
            t_start = round(elapsed, 4)
            col_dark, col_light = step_palette[idx % len(step_palette)]
            step_num = idx + 1

            if act.kind == "perception_step":
                pipeline = str(params.get("pipeline", "aruco_fine"))
                note = self._resolve_perception_context(pipeline, params, connector, workflow_values)
                elapsed += 0.35
                samples.append({"t": round(elapsed, 4), "q": dict(cur_q), "step_index": step_num})
                step_summaries.append({"step": step_num, "action": act.name, "kind": act.kind, "note": note})
                step_segments.append({
                    "step_index": step_num,
                    "step_id": wf_step.id,
                    "action_name": act.name,
                    "kind": act.kind,
                    "t_start": t_start,
                    "t_end": round(elapsed, 4),
                    "duration_s": round(elapsed - t_start, 4),
                    "has_motion": False,
                    "color": col_dark,
                    "light_color": col_light,
                    "note": note,
                })
                continue

            if act.kind == "joint_targets":
                goal_q = dict(cur_q)
                for k, v in (params.get("targets") or {}).items():
                    if k in goal_q:
                        sp = kin.desc_by_name[k]
                        goal_q[k] = max(sp.lower, min(sp.upper, float(v)))
                sub = kin.build_joint_plan(cur_q, goal_q, duration_s=seg_dur, sample_count=12)
                for sp_item in sub["samples"][1:]:
                    samples.append({"t": round(elapsed + sp_item["t"], 4), "q": sp_item["q"], "step_index": step_num})
                elapsed += sub["duration"]
                cur_q = dict(sub["goal"])
                note = f"关节运动 {len(sub['moving'])} 轴"
                step_summaries.append({"step": step_num, "action": act.name, "kind": act.kind, "moving": len(sub["moving"])})
                step_segments.append({
                    "step_index": step_num,
                    "step_id": wf_step.id,
                    "action_name": act.name,
                    "kind": act.kind,
                    "t_start": t_start,
                    "t_end": round(elapsed, 4),
                    "duration_s": round(elapsed - t_start, 4),
                    "has_motion": len(sub["moving"]) > 0,
                    "color": col_dark,
                    "light_color": col_light,
                    "note": note,
                })
                continue

            if act.kind in ("ee_pose", "ee_relative", "ee_axial"):
                arm, tgt_xyz, tgt_rpy, note = self._resolve_ee_target(act.kind, params, cur_q, workflow_values, kin)
                ik_plan = kin.plan_cartesian_ik(arm, cur_q, tgt_xyz, tgt_rpy, waypoints=10, duration_s=seg_dur)
                for sp_item in ik_plan["plan"]["samples"][1:]:
                    samples.append({"t": round(elapsed + sp_item["t"], 4), "q": sp_item["q"], "step_index": step_num})
                elapsed += ik_plan["plan"]["duration"]
                cur_q = dict(ik_plan["plan"]["goal"])
                last_ik_meta = ik_plan["ik"]
                err_mm = round(ik_plan["ik"]["pos_error_m"] * 1000.0, 2)
                max_err = max(1.0, float(params.get("ik_max_err_mm", wf_step.ik_max_err_mm or 15.0)))
                ik_ok = bool(ik_plan["ik"]["ok"]) and err_mm <= max_err
                step_summaries.append({
                    "step": step_num,
                    "action": act.name,
                    "kind": act.kind,
                    "note": note,
                    "ik_pos_err_mm": err_mm,
                    "ik_ok": ik_ok,
                    "ik_max_err_mm": max_err,
                })
                warn_tag = "" if ik_ok else f" [⚠ 超出阈值 {max_err:.0f}mm]"
                step_segments.append({
                    "step_index": step_num,
                    "step_id": wf_step.id,
                    "action_name": act.name,
                    "kind": act.kind,
                    "t_start": t_start,
                    "t_end": round(elapsed, 4),
                    "duration_s": round(elapsed - t_start, 4),
                    "has_motion": len(ik_plan["plan"].get("moving") or []) > 0,
                    "ik_ok": ik_ok,
                    "color": col_dark,
                    "light_color": col_light,
                    "note": f"{arm} IK err={err_mm:.1f}mm{warn_tag} · {note}",
                })
                continue

            # Non-arm step (e.g. nav/policy/chassis): hold arm pose cleanly in joint preview
            elapsed += 0.4
            samples.append({"t": round(elapsed, 4), "q": dict(cur_q), "step_index": step_num})
            step_summaries.append({"step": step_num, "action": act.name, "kind": act.kind, "note": "底盘/策略步骤（保持臂姿）"})
            step_segments.append({
                "step_index": step_num,
                "step_id": wf_step.id,
                "action_name": act.name,
                "kind": act.kind,
                "t_start": t_start,
                "t_end": round(elapsed, 4),
                "duration_s": round(elapsed - t_start, 4),
                "has_motion": False,
                "color": col_dark,
                "light_color": col_light,
                "note": "底盘/策略步骤（保持臂姿）",
            })

        # Compute true union of ALL joints moved across ANY step (never drops round-trip joints!)
        all_moving: list[str] = []
        max_span = 0.0
        for j in connector.descriptor.joints:
            vals = [float(sp["q"].get(j.name, j.default)) for sp in samples]
            if vals:
                span = max(vals) - min(vals)
                if span > 0.008:
                    all_moving.append(j.name)
                if span > max_span:
                    max_span = span

        total_dur = round(max(0.4, elapsed), 4)
        plan = kin.build_joint_plan(
            start_q,
            cur_q,
            duration_s=total_dur,
            kind="workflow_preview",
            ik_meta=last_ik_meta,
            sample_count=len(samples),
        )
        plan["samples"] = samples
        plan["duration"] = total_dur
        plan["moving"] = all_moving
        plan["max_delta"] = round(max_span, 5)
        plan["step_segments"] = step_segments
        plan["workflow_id"] = workflow.id
        plan["workflow_name"] = workflow.name
        return {
            "ok": True,
            "validation": validation,
            "workflow_context": workflow_values,
            "steps": step_summaries,
            "step_segments": step_segments,
            "plan": plan,
        }

    def start_workflow(
        self,
        workflow: WorkflowDefinition,
        connector: RobotConnector,
        gate: RobotSafetyGate,
        dry_run: bool = False,
        operator: str = "operator@motphys",
        step_mode: bool = False,
    ) -> ExecutionJob:
        # Reject unsatisfied parameter-passing dependencies synchronously before creating a job;
        # missing action names still flow into _execute so the Failed job is recorded for audit.
        val = self.validate_workflow(workflow)
        if not val["ok"] and any(c.get("missing") for c in val.get("step_contracts", [])):
            raise SafetyError(422, "workflow_dependency_error", val["message"])

        auth = gate.snapshot()
        if not dry_run:
            gate.require_control_authority(connector.get_observation())
            active_live = next(
                (
                    j
                    for j in self.store.jobs.values()
                    if j.robot_id == workflow.robot_id and j.state in ("Running", "Paused") and j.lease_id is not None
                ),
                None,
            )
            if active_live is not None:
                raise SafetyError(
                    409,
                    "workflow_already_running",
                    f"机器人 {workflow.robot_id} 已有正在运行或暂停中的受控任务 ({active_live.flow_name})，请先完成或取消该任务。",
                )

        now = utc_iso()
        job_id = str(uuid.uuid4())
        run_id = f"r-{uuid.uuid4().hex[:24].upper()}"
        mode_tag = "DRY-RUN" if dry_run else "LIVE"
        step_tag = " · STEP-BY-STEP" if step_mode else ""
        steps = [JobStep(step="start", phase="running", detail="dry-run validation" if dry_run else f"lease={auth.lease_id}{step_tag}")]
        for wf_step in workflow.steps:
            ov_tag = f" overrides={list(wf_step.overrides.keys())}" if wf_step.overrides else ""
            bp_tag = " [断点]" if wf_step.pause_after else ""
            steps.append(JobStep(step=wf_step.action_name, phase="pending", detail=f"duration={wf_step.duration_s:.1f}s timeout={wf_step.timeout_s:.0f}s{ov_tag}{bp_tag}"))
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
            step_mode=bool(step_mode),
            paused_reason="",
            current_step_index=0,
            workflow_context={},
            steps=steps,
            logs=[
                f"{self._ts()} [workflow] Starting {workflow.name} (run={run_id}, mode={mode_tag}{step_tag})",
                f"{self._ts()} [robot] Target={connector.descriptor.id} ({connector.descriptor.name}) connector={connector.descriptor.connector_type}",
            ],
        )
        self.store.save_job(job)

        self._cancel_events[job_id] = asyncio.Event()
        self._resume_events[job_id] = asyncio.Event()
        self._job_connectors[job_id] = connector
        self._job_gates[job_id] = gate
        if step_mode:
            self._step_mode_jobs.add(job_id)
        task = asyncio.create_task(self._execute(job, workflow, connector, gate, dry_run))
        self._tasks[job_id] = task
        return job

    def _mark_canceled(self, job: ExecutionJob, reason: str) -> None:
        if job.state == "Canceled":
            return
        for step in job.steps:
            if step.phase in ("pending", "running", "paused"):
                step.phase = "canceled"
                step.finished_at = utc_iso()
        job.state = "Canceled"
        job.paused_reason = ""
        job.exit_code = 130
        job.finished_at = utc_iso()
        job.logs.append(f"{self._ts()} [workflow] Canceled ({reason})")
        self.store.save_job(job)

    def pause_job(self, job_id: str, reason: str = "操作员手动暂停（点击「下一步」单步执行或「继续全部」）") -> ExecutionJob | None:
        job = self.store.get_job(job_id)
        if not job:
            return None
        if job.state == "Paused":
            return job
        if job.state != "Running":
            raise SafetyError(409, "job_not_running", f"任务当前状态为 {job.state}，无法暂停。")
        self._pause_requested.add(job_id)
        resume_ev = self._resume_events.get(job_id)
        if resume_ev:
            resume_ev.clear()
        if job.lease_id is not None:
            conn = self._job_connectors.get(job_id)
            if conn is not None:
                conn.safe_stop("paused_by_operator")
        job.state = "Paused"
        job.paused_reason = reason
        job.logs.append(f"{self._ts()} [pause] Pause requested by operator")
        self.store.save_job(job)
        return job

    def resume_job(self, job_id: str, single_step: bool = False) -> ExecutionJob | None:
        job = self.store.get_job(job_id)
        if not job:
            return None
        if job.state != "Paused":
            raise SafetyError(409, "job_not_paused", f"任务当前状态为 {job.state}，仅 Paused 状态支持继续或单步执行。")
        if job.lease_id is not None:
            gate = self._job_gates.get(job_id)
            conn = self._job_connectors.get(job_id)
            if gate is not None:
                gate.require_control_authority(conn.get_observation() if conn is not None else None)
        self._pause_requested.discard(job_id)
        if single_step:
            self._step_mode_jobs.add(job_id)
            job.step_mode = True
            mode_note = "单步执行下一动作 (Step)"
        else:
            self._step_mode_jobs.discard(job_id)
            job.step_mode = False
            mode_note = "恢复连续执行 (Resume)"
        job.state = "Running"
        job.paused_reason = ""
        job.logs.append(f"{self._ts()} [resume] {mode_note}")
        self.store.save_job(job)
        resume_ev = self._resume_events.get(job_id)
        if resume_ev:
            resume_ev.set()
        return job

    def cancel_job(self, job_id: str) -> ExecutionJob | None:
        job = self.store.get_job(job_id)
        if not job or job.state == "Canceled":
            return job
        if job.state not in ("Running", "Paused"):
            raise SafetyError(409, "job_already_finished", f"任务已结束（{job.state}），未接受取消；请刷新任务状态。")
        self._cancel_flags.add(job_id)
        self._pause_requested.discard(job_id)
        event = self._cancel_events.get(job_id)
        if event:
            event.set()
        resume_ev = self._resume_events.get(job_id)
        if resume_ev:
            resume_ev.set()
        job.logs.append(f"{self._ts()} [workflow] Cancel requested by operator")
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

    async def _enter_pause_gate(
        self,
        job: ExecutionJob,
        connector: RobotConnector,
        gate: RobotSafetyGate,
        dry_run: bool,
        reason: str,
        step_obj: JobStep | None = None,
    ) -> None:
        """Park the workflow cleanly in Paused state until Resume/Step or Cancel/E-Stop wakes it."""
        self._check_canceled(job)
        resume_ev = self._resume_events.get(job.job_id)
        cancel_ev = self._cancel_events.get(job.job_id)
        if resume_ev is None or cancel_ev is None:
            return
        # If resume already cleared pause before worker reached gate, skip parking
        if resume_ev.is_set() and job.job_id not in self._pause_requested and job.state == "Running":
            resume_ev.clear()
            return
        self._pause_requested.discard(job.job_id)
        resume_ev.clear()
        if step_obj is not None and step_obj.phase == "running":
            step_obj.phase = "paused"
        if job.state != "Paused" or job.paused_reason != reason:
            job.state = "Paused"
            job.paused_reason = reason
            job.logs.append(f"{self._ts()} [pause] {reason}")
            self.store.save_job(job)

        wait_resume = asyncio.create_task(resume_ev.wait())
        wait_cancel = asyncio.create_task(cancel_ev.wait())
        try:
            done, pending = await asyncio.wait(
                {wait_resume, wait_cancel}, return_when=asyncio.FIRST_COMPLETED
            )
            for p in pending:
                p.cancel()
        finally:
            if not wait_resume.done():
                wait_resume.cancel()
            if not wait_cancel.done():
                wait_cancel.cancel()

        self._check_canceled(job)
        if not dry_run:
            if gate.snapshot().estop_latched:
                raise asyncio.CancelledError("estop_latched_while_paused")
            gate.require_control_authority(connector.get_observation())
        resume_ev.clear()
        if step_obj is not None and step_obj.phase == "paused":
            step_obj.phase = "running"
        job.state = "Running"
        job.paused_reason = ""
        self.store.save_job(job)

    async def _wait_for_settlement(
        self,
        job: ExecutionJob,
        step_idx: int,
        wf_step: WorkflowStep,
        action: AtomicAction,
        step_obj: JobStep,
        params: dict[str, Any],
        dispatch_info: dict[str, Any],
        connector: RobotConnector,
        gate: RobotSafetyGate,
        workflow_values: dict[str, Any],
        kin: RobotKinematics | None,
    ) -> None:
        """Closed-loop arrival verification with per-step timeout guard and mid-step pause/resume."""
        settlement = dispatch_info.get("settlement") or {"kind": "duration", "duration_s": 0.25}
        base_detail = str(dispatch_info.get("detail") or "")
        s_kind = str(settlement.get("kind") or "duration")
        timeout_s = max(0.5, min(120.0, float(wf_step.timeout_s or 15.0)))
        poll_dt = 0.05
        active_elapsed = 0.0
        settled_ticks = 0
        last_err_str = ""
        last_save_elapsed = 0.0

        while True:
            self._check_canceled(job)
            if gate.snapshot().estop_latched:
                raise asyncio.CancelledError("aborted_during_step")
            gate.check_watchdogs()
            if not gate.snapshot().lease_active:
                raise asyncio.CancelledError("lease_lost_during_step")

            # Mid-step pause requested: stop live motion immediately, park, then re-dispatch target on resume
            if job.job_id in self._pause_requested or job.state == "Paused":
                connector.safe_stop("paused_by_operator")
                await self._enter_pause_gate(
                    job,
                    connector,
                    gate,
                    dry_run=False,
                    reason=f"步骤 #{step_idx + 1} ({action.name}) 运行中暂停，已执行 safe_stop；继续后将平滑补发目标",
                    step_obj=step_obj,
                )
                # Re-dispatch current step smoothly after resume
                resumed = self._dispatch_action(
                    action.kind, params, connector, gate, workflow_values, kin, ik_max_err_mm=wf_step.ik_max_err_mm
                )
                dispatch_info.update(resumed)
                settlement = dispatch_info.get("settlement") or settlement
                base_detail = str(dispatch_info.get("detail") or base_detail)
                s_kind = str(settlement.get("kind") or s_kind)
                settled_ticks = 0
                job.logs.append(f"{self._ts()} [step:{step_idx + 1}] Resumed & re-dispatched -> {base_detail}")
                self.store.save_job(job)

            obs = connector.get_observation()
            gate.require_control_authority(obs)

            if s_kind == "joints":
                targets: dict[str, float] = settlement.get("targets") or {}
                tol = max(0.002, float(wf_step.settle_tol_rad or 0.025))
                if not targets:
                    step_obj.detail = f"{base_detail} · 闭环到位 (0.00s)"
                    return
                max_err = max(abs(float(obs.joint_positions.get(k, v)) - float(v)) for k, v in targets.items())
                max_vel = max(abs(float(obs.joint_velocities.get(k, 0.0))) for k in targets.keys())
                last_err_str = f"max_joint_err={max_err:.4f}rad"
                if max_err <= tol:
                    settled_ticks += 1
                    if max_vel <= 0.25 or settled_ticks >= 2:
                        step_obj.detail = f"{base_detail} · 闭环到位 err={max_err:.4f}rad ({active_elapsed:.2f}s)"
                        job.logs.append(
                            f"{self._ts()} [step:{step_idx + 1}] Settled (joints max_err={max_err:.4f}rad <= {tol:.3f}, {active_elapsed:.2f}s)"
                        )
                        return
                else:
                    settled_ticks = 0
                    if active_elapsed - last_save_elapsed >= 0.25:
                        step_obj.detail = f"{base_detail} · 闭环收敛中 err={max_err:.3f}rad"
                        self.store.save_job(job)
                        last_save_elapsed = active_elapsed

            elif s_kind == "navigation":
                nav = obs.navigation
                if nav.mode == "FAULT":
                    raise SafetyError(409, "navigation_fault", f"步骤 #{step_idx + 1} 底盘导航进入 FAULT 状态，中止执行。")
                gx = float(settlement.get("x", 0.0))
                gy = float(settlement.get("y", 0.0))
                gyaw = float(settlement.get("yaw", 0.0))
                tol_m = max(0.02, float(wf_step.nav_tol_m or 0.08))
                px = float(nav.pose_map.get("x", 0.0))
                py = float(nav.pose_map.get("y", 0.0))
                pyaw = float(nav.pose_map.get("yaw", 0.0))
                dist = math.hypot(gx - px, gy - py)
                dyaw = abs(math.atan2(math.sin(gyaw - pyaw), math.cos(gyaw - pyaw)))
                last_err_str = f"nav_dist={dist:.3f}m, dyaw={math.degrees(dyaw):.1f}°"
                if (nav.goal_map is None and nav.mode == "MANUAL") or (dist <= tol_m and dyaw <= 0.15):
                    step_obj.detail = f"{base_detail} · 导航到位 dist={dist:.3f}m ({active_elapsed:.2f}s)"
                    job.logs.append(
                        f"{self._ts()} [step:{step_idx + 1}] Settled (nav dist={dist:.3f}m, dyaw={math.degrees(dyaw):.1f}°, {active_elapsed:.2f}s)"
                    )
                    return
                if active_elapsed - last_save_elapsed >= 0.25:
                    step_obj.detail = f"{base_detail} · 导航行进中 距目标 {dist:.2f}m"
                    self.store.save_job(job)
                    last_save_elapsed = active_elapsed

            elif s_kind == "perception":
                pipeline = str(settlement.get("pipeline") or "")
                perc = obs.perception
                waiting_fp = (
                    pipeline in ("read_foundation_pose", "foundation_pose")
                    and perc.foundation_pose_running
                    and perc.foundation_pose_phase in ("waiting", "registering")
                )
                last_err_str = f"perception_phase={perc.foundation_pose_phase}"
                if not waiting_fp:
                    # Re-latch freshest pose after convergence
                    fresh_note = self._resolve_perception_context(pipeline, params, connector, workflow_values)
                    job.workflow_context = dict(workflow_values)
                    step_obj.detail = f"{fresh_note} · 视觉就绪 ({active_elapsed:.2f}s)"
                    job.logs.append(f"{self._ts()} [step:{step_idx + 1}] Settled ({fresh_note})")
                    return

            else:
                target_dur = max(0.05, float(settlement.get("duration_s", 0.2)))
                last_err_str = f"elapsed={active_elapsed:.2f}s/{target_dur:.2f}s"
                if active_elapsed >= target_dur:
                    step_obj.detail = f"{base_detail} · 完成 ({active_elapsed:.2f}s)"
                    return

            if active_elapsed >= timeout_s:
                raise SafetyError(
                    408,
                    "step_settlement_timeout",
                    f"步骤 #{step_idx + 1} ({action.name}) 闭环到位超时 ({timeout_s:.1f}s)，当前残差 {last_err_str}，已触发 safe_stop。",
                )

            t0 = time.monotonic()
            await self._wait_or_cancel(job, poll_dt)
            active_elapsed += max(poll_dt, time.monotonic() - t0)

    def abort_robot_jobs(self, robot_id: str, reason: str) -> None:
        for job in list(self.store.jobs.values()):
            if job.robot_id == robot_id and job.state in ("Running", "Paused"):
                self._cancel_flags.add(job.job_id)
                self._pause_requested.discard(job.job_id)
                event = self._cancel_events.get(job.job_id)
                resume_ev = self._resume_events.get(job.job_id)
                task = self._tasks.get(job.job_id)
                if task and not task.get_loop().is_closed():
                    if event:
                        task.get_loop().call_soon_threadsafe(event.set)
                    if resume_ev:
                        task.get_loop().call_soon_threadsafe(resume_ev.set)
                job.logs.append(f"{self._ts()} [safety] Aborted active job: {reason}")
                self.store.save_job(job)

    async def _execute(
        self,
        job: ExecutionJob,
        workflow: WorkflowDefinition,
        connector: RobotConnector,
        gate: RobotSafetyGate,
        dry_run: bool,
    ) -> None:
        workflow_values: dict[str, Any] = {}
        kin = self.kinematics_map.get(workflow.robot_id)
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
                job.current_step_index = idx + 1
                if job.job_id in self._cancel_flags:
                    raise asyncio.CancelledError("canceled_by_operator_or_safety")

                # Honor operator pause requested right before starting this step
                if job.job_id in self._pause_requested or job.state == "Paused":
                    await self._enter_pause_gate(
                        job,
                        connector,
                        gate,
                        dry_run=dry_run,
                        reason=f"暂停于步骤 #{idx + 1} ({wf_step.action_name}) 开始前，点击「下一步」或「继续全部」",
                    )

                if not dry_run:
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
                if wf_step.overrides:
                    job.logs.append(f"{self._ts()} [step:{idx + 1}] Applied overrides={wf_step.overrides}")

                if dry_run:
                    if action.kind == "perception_step":
                        pipeline = str(params.get("pipeline", "aruco_fine"))
                        ctx_note = self._resolve_perception_context(pipeline, params, connector, workflow_values)
                        job.workflow_context = dict(workflow_values)
                        step_obj.detail = f"dry-run: {ctx_note}"
                        job.logs.append(f"{self._ts()} [step:{idx + 1}] Dry-run context -> {ctx_note}")
                    elif action.kind in ("ee_pose", "ee_relative", "ee_axial") and kin is not None:
                        obs = connector.get_observation()
                        arm, tgt_xyz, tgt_rpy, note = self._resolve_ee_target(
                            action.kind, params, obs.joint_positions, workflow_values, kin
                        )
                        ik = kin.solve_ik(arm, tgt_xyz, tgt_rpy, obs.joint_positions)
                        err_mm = ik["pos_error_m"] * 1000.0
                        max_err = max(1.0, float(params.get("ik_max_err_mm", wf_step.ik_max_err_mm or 15.0)))
                        ok_flag = bool(ik["ok"]) and err_mm <= max_err
                        step_obj.detail = f"dry-run IK ({arm}) err={err_mm:.2f}mm ({'OK' if ok_flag else f'WARN >{max_err:.0f}mm'})"
                        job.logs.append(
                            f"{self._ts()} [step:{idx + 1}] Dry-run IK: {note} | pos_err={err_mm:.2f}mm rot_err={ik['rot_error_deg']:.2f}°"
                        )
                    else:
                        step_obj.detail = f"dry-run validated ({action.kind})"
                        job.logs.append(f"{self._ts()} [step:{idx + 1}] Dry-run OK: params keys={list(params.keys())}")

                    wait_s = min(max(wf_step.duration_s * 0.35, 0.25), 0.7)
                    chunks = max(1, int(wait_s / 0.1))
                    for _ in range(chunks):
                        if job.job_id in self._cancel_flags:
                            raise asyncio.CancelledError("aborted_during_step")
                        await self._wait_or_cancel(job, 0.1)
                else:
                    dispatch_info = self._dispatch_action(
                        action.kind,
                        params,
                        connector,
                        gate,
                        workflow_values,
                        kin,
                        ik_max_err_mm=wf_step.ik_max_err_mm,
                    )
                    # Handle legacy/mocked callers in unit tests returning a plain str
                    if isinstance(dispatch_info, str):
                        dispatch_info = {"detail": dispatch_info, "settlement": {"kind": "duration", "duration_s": 0.2}}
                    detail = str(dispatch_info.get("detail") or "")
                    job.workflow_context = dict(workflow_values)
                    step_obj.detail = detail
                    job.logs.append(f"{self._ts()} [step:{idx + 1}] Dispatched -> {detail}")
                    self.store.save_job(job)

                    if wf_step.wait_for_arrival:
                        await self._wait_for_settlement(
                            job,
                            idx,
                            wf_step,
                            action,
                            step_obj,
                            params,
                            dispatch_info,
                            connector,
                            gate,
                            workflow_values,
                            kin,
                        )
                    else:
                        wait_s = min(max(wf_step.duration_s * 0.35, 0.25), 0.7)
                        chunks = max(1, int(wait_s / 0.1))
                        for _ in range(chunks):
                            if job.job_id in self._cancel_flags or gate.snapshot().estop_latched:
                                raise asyncio.CancelledError("aborted_during_step")
                            await self._wait_or_cancel(job, 0.1)

                self._check_canceled(job)
                step_obj.phase = "succeeded"
                step_obj.finished_at = utc_iso()
                self.store.save_job(job)

                # Step-by-step breakpoint or explicit per-step pause_after or operator pause before next step
                has_next = idx < len(workflow.steps) - 1
                if has_next and (
                    job.job_id in self._step_mode_jobs
                    or wf_step.pause_after
                    or job.job_id in self._pause_requested
                    or job.state == "Paused"
                ):
                    next_step = workflow.steps[idx + 1]
                    reason = (
                        f"单步断点：步 {idx + 1}/{len(workflow.steps)} ({action.name}) 已到位，确认安全后点击「下一步」执行步 {idx + 2} ({next_step.action_name})"
                        if job.job_id in self._step_mode_jobs
                        else f"断点暂停：步 {idx + 1}/{len(workflow.steps)} ({action.name}) 已完成，等待确认执行步 {idx + 2} ({next_step.action_name})"
                    )
                    await self._enter_pause_gate(job, connector, gate, dry_run=dry_run, reason=reason)

            self._check_canceled(job)
            end_step = job.steps[-1]
            end_step.phase = "succeeded"
            end_step.finished_at = utc_iso()
            end_step.detail = "all steps settled & completed cleanly"
            job.state = "Succeeded"
            job.paused_reason = ""
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
                if st.phase in ("running", "paused"):
                    st.phase = "failed"
                    st.detail = msg
                    st.finished_at = utc_iso()
                    failed_marked = True
                elif st.phase == "pending":
                    st.phase = "canceled"
                    st.detail = "skipped after upstream failure" if failed_marked else msg
            job.state = "Failed"
            job.paused_reason = ""
            job.exit_code = 1
            job.finished_at = utc_iso()
            job.logs.append(f"{self._ts()} [error] {msg}")
            job.logs.append(f"{self._ts()} [workflow] Exiting with code 1")
            self.store.save_job(job)
        finally:
            self._cancel_flags.discard(job.job_id)
            self._cancel_events.pop(job.job_id, None)
            self._pause_requested.discard(job.job_id)
            self._step_mode_jobs.discard(job.job_id)
            self._resume_events.pop(job.job_id, None)
            self._job_connectors.pop(job.job_id, None)
            self._job_gates.pop(job.job_id, None)
            self._tasks.pop(job.job_id, None)

    def _dispatch_action(
        self,
        kind: str,
        params: dict[str, Any],
        connector: RobotConnector,
        gate: RobotSafetyGate,
        workflow_values: dict[str, Any],
        kin: RobotKinematics | None,
        ik_max_err_mm: float = 15.0,
    ) -> dict[str, Any]:
        obs = connector.get_observation()
        gate.require_control_authority(obs)
        if kind == "joint_targets":
            raw_targets = params.get("targets") or {}
            validated = gate.validate_joint_targets(raw_targets, obs)
            dur = float(params.get("duration_s", 1.2))
            connector.send_joint_targets(validated, dur)
            return {
                "detail": f"sent {len(validated)} joint targets",
                "settlement": {"kind": "joints", "targets": validated},
            }
        if kind in ("ee_relative", "ee_pose", "ee_axial") and kin is not None:
            arm, tgt_xyz, tgt_rpy, note = self._resolve_ee_target(kind, params, obs.joint_positions, workflow_values, kin)
            ik = kin.solve_ik(arm, tgt_xyz, tgt_rpy, obs.joint_positions)
            err_mm = ik["pos_error_m"] * 1000.0
            max_err = max(1.0, float(params.get("ik_max_err_mm", ik_max_err_mm or 15.0)))
            if (not ik["ok"]) or err_mm > max_err:
                raise SafetyError(
                    422,
                    "ik_convergence_failed",
                    f"在线 IK 复核未收敛 ({arm} 位置残差 {err_mm:.2f}mm > 阈值 {max_err:.1f}mm，目标 {tgt_xyz})，已拦截下发。",
                )
            validated = gate.validate_joint_targets(ik["solved_arm_joints"], obs)
            connector.send_joint_targets(validated, float(params.get("duration_s", 1.2)))
            return {
                "detail": f"IK {kind} ({arm}) {note} | err={err_mm:.2f}mm",
                "settlement": {"kind": "joints", "targets": validated},
            }
        if kind in ("ee_relative", "ee_pose", "ee_axial"):
            arm = str(params.get("arm", "left"))
            raw_dxyz = params.get("delta_xyz_m")
            if kind == "ee_axial" and isinstance(raw_dxyz, (list, tuple)) and len(raw_dxyz) >= 3:
                dx, dy, dz = float(raw_dxyz[0]), float(raw_dxyz[1]), float(raw_dxyz[2])
            else:
                dx = float(params.get("dx", 0.0))
                dy = float(params.get("dy", 0.0))
                dz = float(params.get("dz", 0.04 if kind == "ee_relative" else 0.0))
            dyaw = (float(params.get("end_angle_deg", 0.0)) - float(params.get("start_angle_deg", 0.0))) * 0.2
            if not all(math.isfinite(v) for v in (dx, dy, dz, dyaw)):
                raise SafetyError(400, "invalid_number", "末端步进参数包含非有限值 (NaN/Inf)。")
            if max(abs(dx), abs(dy), abs(dz)) > 0.25 or abs(dyaw) > 45.0:
                raise SafetyError(400, "ee_step_exceeded", "末端单步位移超过安全上限 (±0.25m / ±45°)。")
            connector.send_ee_step(arm, dx, dy, dz, 0.0, 0.0, dyaw)
            return {
                "detail": f"EE {kind} ({arm}) dx={dx:.2f} dy={dy:.2f} dz={dz:.2f}",
                "settlement": {"kind": "duration", "duration_s": 0.15},
            }
        if kind == "chassis_velocity":
            vx = float(params.get("vx", 0.0))
            vy = float(params.get("vy", 0.0))
            omega = float(params.get("omega", 0.0))
            dur = float(params.get("duration_s", 0.5))
            if not all(math.isfinite(v) for v in (vx, vy, omega, dur)):
                raise SafetyError(400, "invalid_number", "底盘速度包含非有限值 (NaN/Inf)。")
            if abs(vx) > 1.0 or abs(vy) > 1.0 or abs(omega) > 1.5 or not (0.0 < dur <= 5.0):
                raise SafetyError(400, "chassis_limit_exceeded", "底盘速度或持续时长超出安全上限 (|vx,vy|<=1.0m/s, |omega|<=1.5rad/s, dur<=5s)。")
            connector.send_chassis_velocity(vx, vy, omega, dur)
            return {
                "detail": f"chassis vel vx={vx} omega={omega} ({dur:.2f}s)",
                "settlement": {"kind": "duration", "duration_s": dur},
            }
        if kind == "navigate_to_pose":
            x = float(params.get("x", 0.0))
            y = float(params.get("y", 0.0))
            yaw = float(params.get("yaw", 0.0))
            if not all(math.isfinite(v) for v in (x, y, yaw)):
                raise SafetyError(400, "invalid_number", "导航目标包含非有限值 (NaN/Inf)。")
            connector.set_navigation_mode("NAVIGATION")
            connector.send_navigation_goal(x, y, yaw)
            return {
                "detail": f"nav goal x={x:.2f} y={y:.2f} yaw={yaw:.2f}",
                "settlement": {"kind": "navigation", "x": x, "y": y, "yaw": yaw},
            }
        if kind == "perception_step":
            pipeline = str(params.get("pipeline", "aruco_fine"))
            if pipeline == "aruco_fine" and connector.descriptor.capabilities.aruco:
                connector.control_perception("aruco_start_pose", {})
                connector.control_perception("aruco_set_fine", {"enabled": True})
            ctx_note = self._resolve_perception_context(pipeline, params, connector, workflow_values)
            return {
                "detail": f"perception '{pipeline}' -> {ctx_note}",
                "settlement": {"kind": "perception", "pipeline": pipeline},
            }
        if kind == "policy_intent":
            ckpt = params.get("checkpoint")
            if ckpt:
                connector.control_policy("switch_checkpoint", {"checkpoint_id": ckpt})
            return {
                "detail": f"policy intent dispatched ({params.get('task_intent', 'default')})",
                "settlement": {"kind": "duration", "duration_s": 0.25},
            }
        return {
            "detail": f"completed {kind}",
            "settlement": {"kind": "duration", "duration_s": 0.15},
        }
