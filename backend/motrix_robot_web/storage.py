from __future__ import annotations

import json
import math
import uuid
from pathlib import Path
from typing import Any

from .safety import utc_iso
from .schemas import AtomicAction, ExecutionJob, JobStep, LoopRecord, MapEditOverlay, NavWaypoint, WorkflowDefinition, WorkflowStep


def _safe_finite(val: Any, default: float = 0.0) -> float:
    try:
        f = float(val)
    except (TypeError, ValueError):
        return default
    return f if math.isfinite(f) else default


def _atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    """Write JSON strictly (allow_nan=False) to a unique temp file then atomically replace target."""
    raw = json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False)
    tmp = path.with_name(f"{path.name}.{uuid.uuid4().hex[:8]}.tmp")
    tmp.write_text(raw, encoding="utf-8")
    tmp.replace(path)


def _r1_joint_name_to_canonical(raw_name: str, group: str | None = None, index: int | None = None) -> str:
    """Map R1 Pro actions.json joint names like 'torso[1]', 'left_arm[2]', 'left_gripper[1]' to canonical names."""
    name = (raw_name or "").strip()
    if "[" in name and name.endswith("]"):
        prefix, idx_str = name[:-1].split("[", 1)
        if prefix in ("left_gripper", "right_gripper"):
            return prefix
        return f"{prefix}_{idx_str}"
    if group in ("left_gripper", "right_gripper"):
        return group
    if group and index is not None:
        return f"{group}_{int(index) + 1}"
    return name


def _seed_actions() -> list[AtomicAction]:
    now = utc_iso()
    return [
        AtomicAction(
            id="act-r1-home",
            name="r1_safe_home",
            robot_id="r1pro-mock",
            kind="joint_targets",
            description="R1 Pro 躯干直立与双臂安全初始位",
            params={
                "targets": {
                    "torso_1": 0.0,
                    "torso_2": 0.0,
                    "torso_3": 0.0,
                    "torso_4": 0.0,
                    "left_arm_1": 0.25,
                    "left_arm_2": 0.35,
                    "left_arm_4": -0.9,
                    "right_arm_1": -0.25,
                    "right_arm_2": -0.35,
                    "right_arm_4": -0.9,
                    "left_gripper": 0.8,
                    "right_gripper": 0.8,
                },
                "duration_s": 1.4,
            },
            tags=["home", "safe"],
            created_at=now,
            updated_at=now,
        ),
        AtomicAction(
            id="act-r1-torso-hook",
            name="钩箱躯干姿态_v1",
            robot_id="r1pro-mock",
            kind="joint_targets",
            description="仅包含 torso 1-4 的 S 形搬箱躯干姿态（兼容 r1pro_web 搬运标定）",
            params={
                "targets": {
                    "torso_1": 0.354,
                    "torso_2": -0.761,
                    "torso_3": -0.524,
                    "torso_4": 0.0,
                },
                "duration_s": 1.6,
            },
            tags=["torso", "hook_transport"],
            created_at=now,
            updated_at=now,
        ),
        AtomicAction(
            id="act-r1-nav-station",
            name="nav_to_place_station_A",
            robot_id="r1pro-mock",
            kind="navigate_to_pose",
            description="切换至导航模式，移动至工位 A 并在到位后切回手动模式",
            params={"x": 1.166, "y": -0.443, "yaw": -1.553},
            tags=["navigation", "chassis"],
            created_at=now,
            updated_at=now,
        ),
        AtomicAction(
            id="act-r1-aruco-fine",
            name="aruco_id1_chassis_fine_adjust",
            robot_id="r1pro-mock",
            kind="perception_step",
            description="基于 ArUco Marker #1 的分段底盘精对准（目标 x=0.60m, y=0, yaw=-90°）",
            params={"pipeline": "aruco_fine", "marker_id": 1, "target_x_m": 0.60},
            tags=["aruco", "perception"],
            created_at=now,
            updated_at=now,
        ),
        AtomicAction(
            id="act-r1-ee-lift",
            name="dual_arm_carry_lift",
            robot_id="r1pro-mock",
            kind="ee_relative",
            description="末端沿 base_link +Z 同步抬升 12cm",
            params={"arm": "left", "dx": 0.0, "dy": 0.0, "dz": 0.12},
            tags=["ee", "carry"],
            created_at=now,
            updated_at=now,
        ),
        AtomicAction(
            id="act-r1-axial-pour",
            name="ee_axial_pour_template",
            robot_id="r1pro-mock",
            kind="ee_axial",
            description="末端绕局部 -Z 轴旋转 45° 并同步线性微移（源自 R1 Pro 模板）",
            params={"arm": "right", "start_angle_deg": 0, "end_angle_deg": 45, "speed_deg_s": 15, "delta_xyz_m": [0.0, 0.0, 0.02]},
            tags=["ee", "axial"],
            created_at=now,
            updated_at=now,
        ),
        AtomicAction(
            id="act-r1-read-fp",
            name="read_foundation_pose_target",
            robot_id="r1pro-mock",
            kind="perception_step",
            description="读取 FoundationPose 6D 物体位姿并写入工作流上下文（produces: workflow_xyz, workflow_pose）",
            params={"pipeline": "read_foundation_pose", "frame": "base_link"},
            tags=["perception", "produces:workflow_pose"],
            created_at=now,
            updated_at=now,
        ),
        AtomicAction(
            id="act-r1-ee-pose-tpl",
            name="ee_pose_template",
            robot_id="r1pro-mock",
            kind="ee_pose",
            description="6D 绝对位姿模板：支持手动 xyz_m 或消费上游 workflow_pose/workflow_xyz + 偏置并执行 URDF IK",
            params={
                "arm": "right",
                "xyz_source": "workflow_pose",
                "xyz_m": [0.36, -0.38, 0.92],
                "offset_xyz_m": [0.0, 0.0, 0.08],
                "rpy_deg": [0.0, 0.0, 0.0],
                "duration_s": 1.4,
            },
            tags=["ee", "ik", "requires:workflow_pose"],
            created_at=now,
            updated_at=now,
        ),
        AtomicAction(
            id="act-piper-home",
            name="piper_dual_ready_pose",
            robot_id="piper-dual-mock",
            kind="joint_targets",
            description="Piper 双臂桌面摆盘预备姿态（14-DOF 软限位内）",
            params={
                "targets": {
                    "left_j1": 0.0,
                    "left_j2": 0.65,
                    "left_j3": -0.95,
                    "left_j4": 0.0,
                    "left_j5": 0.45,
                    "left_j6": 0.0,
                    "left_gripper": 0.85,
                    "right_j1": 0.0,
                    "right_j2": 0.65,
                    "right_j3": -0.95,
                    "right_j4": 0.0,
                    "right_j5": 0.45,
                    "right_j6": 0.0,
                    "right_gripper": 0.85,
                },
                "duration_s": 1.2,
            },
            tags=["home", "set_the_plates"],
            created_at=now,
            updated_at=now,
        ),
        AtomicAction(
            id="act-piper-policy",
            name="piper_run_pi05_plates",
            robot_id="piper-dual-mock",
            kind="policy_intent",
            description="经 Edge/Policy 链路下达 pi0.5 摆盘任务意图（遵守 14-DOF Action Layout）",
            params={
                "checkpoint": "pi05_real_all_mcap_20261119_base_30k_v1/030000",
                "task_intent": "set_the_plates",
                "duration_s": 1.5,
            },
            tags=["policy", "pi05"],
            created_at=now,
            updated_at=now,
        ),
        AtomicAction(
            id="act-w1-neutral",
            name="w1_upper_neutral",
            robot_id="dex-w1-mock",
            kind="joint_targets",
            description="Dexforce W1 上半身安全中立位（映射至真实 URDF 关节）",
            params={
                "targets": {
                    "WAIST_YAW": 0.0,
                    "HEAD_YAW_J1": 0.0,
                    "HEAD_PITCH_J2": 0.0,
                    "SHOULDER_PITCH_L_J1": 0.18,
                    "ELBOW_PITCH_L_J3": -0.62,
                    "SHOULDER_PITCH_R_J1": 0.18,
                    "ELBOW_PITCH_R_J3": -0.62,
                },
                "duration_s": 1.3,
            },
            tags=["home", "w1"],
            created_at=now,
            updated_at=now,
        ),
    ]


def _seed_workflows() -> list[WorkflowDefinition]:
    now = utc_iso()
    return [
        WorkflowDefinition(
            id="wf-r1-box-transport",
            name="dual_hook_box_transport_v2",
            robot_id="r1pro-mock",
            description="R1 Pro 复位 → ArUco 精对准 → 搬箱躯干姿态 → 双臂抬升 → 运送至工位 A",
            kind="Workflow",
            steps=[
                WorkflowStep(id="st-1", action_name="r1_safe_home", delay_s=0.0, duration_s=0.8),
                WorkflowStep(id="st-2", action_name="aruco_id1_chassis_fine_adjust", delay_s=0.1, duration_s=0.9),
                WorkflowStep(id="st-3", action_name="钩箱躯干姿态_v1", delay_s=0.1, duration_s=0.9),
                WorkflowStep(id="st-4", action_name="dual_arm_carry_lift", delay_s=0.1, duration_s=0.8),
                WorkflowStep(id="st-5", action_name="nav_to_place_station_A", delay_s=0.1, duration_s=1.0),
            ],
            created_at=now,
            updated_at=now,
        ),
        WorkflowDefinition(
            id="wf-r1-fp-grasp-param-pass",
            name="r1_foundationpose_ik_grasp_flow",
            robot_id="r1pro-mock",
            description="参数传递示例（源自 r1pro_web）：读取 FoundationPose 6D 位姿 → 注入预抓取(+10cm) IK → 下探抓取 IK → 相对抬升(+12cm)",
            kind="Workflow",
            steps=[
                WorkflowStep(id="st-fp-1", action_name="read_foundation_pose_target", delay_s=0.0, duration_s=0.6),
                WorkflowStep(
                    id="st-fp-2",
                    action_name="ee_pose_template",
                    delay_s=0.1,
                    duration_s=1.4,
                    overrides={"arm": "right", "xyz_source": "workflow_pose", "offset_xyz_m": [0.0, 0.0, 0.10]},
                ),
                WorkflowStep(
                    id="st-fp-3",
                    action_name="ee_pose_template",
                    delay_s=0.1,
                    duration_s=1.2,
                    overrides={"arm": "right", "xyz_source": "workflow_pose", "offset_xyz_m": [0.0, 0.0, 0.0]},
                ),
                WorkflowStep(
                    id="st-fp-4",
                    action_name="dual_arm_carry_lift",
                    delay_s=0.1,
                    duration_s=1.0,
                    overrides={"arm": "right", "reference_source": "workflow_pose", "dz": 0.12},
                ),
            ],
            created_at=now,
            updated_at=now,
        ),
        WorkflowDefinition(
            id="wf-piper-plate-eval",
            name="piper_set_the_plates_eval_flow",
            robot_id="piper-dual-mock",
            description="Piper 双臂回到安全预备位 → 下达 pi0.5 摆盘推理意图 → 自动复位",
            kind="Inference",
            steps=[
                WorkflowStep(id="st-p1", action_name="piper_dual_ready_pose", delay_s=0.0, duration_s=0.9),
                WorkflowStep(id="st-p2", action_name="piper_run_pi05_plates", delay_s=0.1, duration_s=1.1),
                WorkflowStep(id="st-p3", action_name="piper_dual_ready_pose", delay_s=0.1, duration_s=0.8),
            ],
            created_at=now,
            updated_at=now,
        ),
        WorkflowDefinition(
            id="wf-w1-check",
            name="w1_posture_self_check",
            robot_id="dex-w1-mock",
            description="Dexforce W1 上半身安全中立位姿态复核（验证联锁与 3D/Motrix 只读镜像）",
            kind="Manual",
            steps=[
                WorkflowStep(id="st-w1", action_name="w1_upper_neutral", delay_s=0.0, duration_s=0.9),
            ],
            created_at=now,
            updated_at=now,
        ),
    ]


def _seed_jobs() -> list[ExecutionJob]:
    return [
        ExecutionJob(
            job_id="8d406ead-b396-41ce-a91f-b3ee74c15e94",
            run_id="r-01M7QDY7GM4K8X726B4NSZMBPQ",
            flow_name="piper_set_the_plates_eval_flow",
            kind="Inference",
            state="Succeeded",
            robot_id="piper-dual-mock",
            workflow_id="wf-piper-plate-eval",
            created_at="2026-11-23T14:36:02Z",
            started_at="2026-11-23T14:36:02Z",
            finished_at="2026-11-23T14:36:21Z",
            exit_code=0,
            pipeline_sha="3f8eb3d165a7ca02a5aeb5a279ff92e1732cddf3",
            operator="wubinxin@motphys",
            lease_id="lease-a91f74c15e",
            steps=[
                JobStep(step="start", phase="succeeded", detail="lease active · 14-DOF Action Layout verified"),
                JobStep(step="piper_dual_ready_pose", phase="succeeded", detail="14 joints reached target within 0.012 rad"),
                JobStep(step="piper_run_pi05_plates", phase="succeeded", detail="pi05_real_all_mcap_20261119_base_30k_v1/030000 · RTC overlap=0.7"),
                JobStep(step="end", phase="succeeded", detail="linked to loop-rec-001"),
            ],
            logs=[
                "2026-11-23 14:36:02.144 [workflow] Starting piper_set_the_plates_eval_flow for operator:wubinxin",
                "2026-11-23 14:36:02.189 [safety] Lease verified: lease-a91f74c15e (hb_age=0.24s < 3.0s)",
                "2026-11-23 14:36:02.210 [edge] Observation Layout matched: DualPiper 14-DOF (left[0:7], right[7:14])",
                "2026-11-23 14:36:05.840 [step:1] piper_dual_ready_pose succeeded (max_err=0.008 rad)",
                "2026-11-23 14:36:18.920 [step:2] Policy intent completed · 0 interlock trips · max_track_err=0.031 rad",
                "2026-11-23 14:36:21.012 [workflow] Completed with exit code 0",
            ],
        ),
        ExecutionJob(
            job_id="a19c310f-4f22-4a8e-99b0-2c11d8a3f901",
            run_id="r-01M7QDTFYVDMN9N8GA6ZZ7T0J6",
            flow_name="dual_hook_box_transport_v2",
            kind="Workflow",
            state="Failed",
            robot_id="r1pro-mock",
            workflow_id="wf-r1-box-transport",
            created_at="2026-11-23T13:18:10Z",
            started_at="2026-11-23T13:18:11Z",
            finished_at="2026-11-23T13:18:24Z",
            exit_code=1,
            pipeline_sha="3f8eb3d165a7ca02a5aeb5a279ff92e1732cddf3",
            operator="wubinxin@motphys",
            lease_id="lease-2c11d8a3f9",
            steps=[
                JobStep(step="start", phase="succeeded", detail="R1 Pro 20-DOF authority checked"),
                JobStep(step="r1_safe_home", phase="succeeded", detail="torso & dual arms settled"),
                JobStep(step="aruco_id1_chassis_fine_adjust", phase="failed", detail="marker #1 occluded > 2.0s; safe_stop issued"),
                JobStep(step="钩箱躯干姿态_v1", phase="canceled", detail="skipped after upstream step failure"),
            ],
            logs=[
                "2026-11-23 13:18:11.004 [workflow] Starting dual_hook_box_transport_v2 on r1pro-mock",
                "2026-11-23 13:18:15.402 [step:1] r1_safe_home succeeded",
                "2026-11-23 13:18:24.112 [step:2] ERROR: ArUco marker #1 observation timed out (>2.0s)",
                "2026-11-23 13:18:24.115 [safety] Zero chassis velocity published; workflow aborted cleanly (exit=1)",
            ],
        ),
    ]


def _seed_loop_records() -> list[LoopRecord]:
    now = utc_iso()
    return [
        LoopRecord(
            record_id="loop-rec-001",
            robot_id="piper-dual-mock",
            task_name="set_the_plates_real_bimanual",
            owner="wubinxin",
            source_type="existing_model",
            source_ref="omah:18100 / pi05_real_all_mcap_20261119_base_30k_v1",
            model_artifact="checkpoints/pi05_real_all_mcap_20261119_base_30k_v1/030000",
            eval_plan="离线 LeRobot 回放 + 真机单机低速 10 轮评测（不虚增训练轮次）",
            eval_status="passed",
            success_rate=0.80,
            episodes_count=10,
            evidence_run_id="r-01M7QDY7GM4K8X726B4NSZMBPQ",
            notes="max_first_step=0.45, J4 track_error_hold=0.6 rad；主臂反馈尚未验收，暂不计入 DAgger 闭环",
            next_action="待 Edge M2/M4 验收后切至 Edge Connector 复用统一采集/推理链路",
            created_at=now,
            updated_at=now,
        ),
        LoopRecord(
            record_id="loop-rec-002",
            robot_id="r1pro-mock",
            task_name="r1pro_dual_hook_box_transport",
            owner="wubinxin",
            source_type="existing_model",
            source_ref="r1pro_web_control v2 workflow + FoundationPose",
            model_artifact="FoundationPose + YOLO bottle/box preset",
            eval_plan="pc10 跨网段只读接入 → 授权低速工位搬运复测",
            eval_status="no_result",
            success_rate=None,
            episodes_count=0,
            evidence_run_id="r-01M7QDTFYVDMN9N8GA6ZZ7T0J6",
            notes="尚未完成 R1 Pro 整机实测与 Edge 适配，严格标记为 no_result（不与零成功混淆）",
            next_action="完成 192.168.3.x 跨网段联通检查并确认过渡期单入口禁双写规则",
            created_at=now,
            updated_at=now,
        ),
    ]


def _seed_waypoints() -> list[NavWaypoint]:
    now = utc_iso()
    return [
        NavWaypoint(id="wp-fs-home", robot_id="r1pro-mock", map_id="fast_slam_20260914", name="建图原点 (Dock)", x=0.0, y=0.0, yaw=0.0, note="Fast-SLAM Plus 坐标系原点", created_at=now),
        NavWaypoint(id="wp-fs-rotunda", robot_id="r1pro-mock", map_id="fast_slam_20260914", name="圆形大厅中心", x=14.5, y=-1.5, yaw=0.0, note="东北侧圆形大厅可通行区", created_at=now),
        NavWaypoint(id="wp-fs-south", robot_id="r1pro-mock", map_id="fast_slam_20260914", name="南侧主通道", x=2.0, y=-21.0, yaw=1.57, note="南侧主通道工位", created_at=now),
        NavWaypoint(id="wp-mx-home", robot_id="r1pro-mock", map_id="mx_map", name="pc10 原点", x=0.0, y=0.0, yaw=0.0, note="mx_map 基准点", created_at=now),
        NavWaypoint(id="wp-mx-station-a", robot_id="r1pro-mock", map_id="mx_map", name="放置工位 A", x=1.166, y=-0.443, yaw=-1.553, note="对应 nav_to_place_station_A", created_at=now),
        NavWaypoint(id="wp-r1-home", robot_id="r1pro-mock", map_id="r1pro_20260918", name="建图原点 (Dock)", x=0.0, y=0.0, yaw=0.0, note="SLAM 坐标系原点", created_at=now),
        NavWaypoint(id="wp-r1-corridor-start", robot_id="r1pro-mock", map_id="r1pro_20260918", name="走廊起点工位", x=-4.125, y=-10.875, yaw=0.785, note="来自 r1pro_map_vis 规划起点", created_at=now),
        NavWaypoint(id="wp-r1-station-b", robot_id="r1pro-mock", map_id="r1pro_20260918", name="大厅目标工位 B", x=4.725, y=-4.875, yaw=0.0, note="来自 r1pro_map_vis 规划终点", created_at=now),
    ]


class LocalStore:
    """Persistent JSON store for actions, workflows, jobs, and Loop records (with R1 Pro JSON import/export)."""

    def __init__(self, data_dir: Path) -> None:
        self.data_dir = data_dir
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.actions_path = self.data_dir / "actions.json"
        self.workflows_path = self.data_dir / "workflows.json"
        self.jobs_path = self.data_dir / "jobs.json"
        self.loop_path = self.data_dir / "loop_records.json"
        self.waypoints_path = self.data_dir / "waypoints.json"
        self.map_edits_path = self.data_dir / "map_edits.json"

        self.actions: dict[str, AtomicAction] = {}
        self.workflows: dict[str, WorkflowDefinition] = {}
        self.jobs: dict[str, ExecutionJob] = {}
        self.loop_records: dict[str, LoopRecord] = {}
        self.waypoints: dict[str, NavWaypoint] = {}
        self.map_edits: dict[str, MapEditOverlay] = {}
        self._load_or_seed()

    def _load_or_seed(self) -> None:
        if self.actions_path.exists():
            raw = json.loads(self.actions_path.read_text(encoding="utf-8"))
            for item in raw.get("actions", []):
                act = AtomicAction.model_validate(item)
                self.actions[act.id] = act
            # Ensure newly added built-in parameter-passing template actions exist without overwriting user data
            added_act = False
            for seed_act in _seed_actions():
                if seed_act.id in ("act-r1-read-fp", "act-r1-ee-pose-tpl") and seed_act.id not in self.actions:
                    self.actions[seed_act.id] = seed_act
                    added_act = True
            if added_act:
                self._save_actions()
        else:
            for act in _seed_actions():
                self.actions[act.id] = act
            self._save_actions()

        if self.workflows_path.exists():
            raw = json.loads(self.workflows_path.read_text(encoding="utf-8"))
            for item in raw.get("workflows", []):
                wf = WorkflowDefinition.model_validate(item)
                self.workflows[wf.id] = wf
            added_wf = False
            for seed_wf in _seed_workflows():
                if seed_wf.id == "wf-r1-fp-grasp-param-pass" and seed_wf.id not in self.workflows:
                    self.workflows[seed_wf.id] = seed_wf
                    added_wf = True
            if added_wf:
                self._save_workflows()
        else:
            for wf in _seed_workflows():
                self.workflows[wf.id] = wf
            self._save_workflows()

        if self.jobs_path.exists():
            raw = json.loads(self.jobs_path.read_text(encoding="utf-8"))
            for item in raw.get("jobs", []):
                job = ExecutionJob.model_validate(item)
                self.jobs[job.job_id] = job
        else:
            for job in _seed_jobs():
                self.jobs[job.job_id] = job
            self._save_jobs()

        if self.loop_path.exists():
            raw = json.loads(self.loop_path.read_text(encoding="utf-8"))
            for item in raw.get("records", []):
                rec = LoopRecord.model_validate(item)
                self.loop_records[rec.record_id] = rec
        else:
            for rec in _seed_loop_records():
                self.loop_records[rec.record_id] = rec
            self._save_loop()

        if self.waypoints_path.exists():
            raw = json.loads(self.waypoints_path.read_text(encoding="utf-8"))
            for item in raw.get("waypoints", []):
                wp = NavWaypoint.model_validate(item)
                self.waypoints[wp.id] = wp
            added_wp = False
            for seed_wp in _seed_waypoints():
                if seed_wp.map_id == "fast_slam_20260914" and seed_wp.id not in self.waypoints:
                    self.waypoints[seed_wp.id] = seed_wp
                    added_wp = True
            if added_wp:
                self._save_waypoints()
        else:
            for wp in _seed_waypoints():
                self.waypoints[wp.id] = wp
            self._save_waypoints()

        if self.map_edits_path.exists():
            try:
                raw = json.loads(self.map_edits_path.read_text(encoding="utf-8"))
                for item in raw.get("overlays", []):
                    ov = MapEditOverlay.model_validate(item)
                    self.map_edits[ov.map_id] = ov
            except Exception:
                self.map_edits = {}

    def _save_actions(self) -> None:
        payload = {"version": 2, "actions": [a.model_dump() for a in self.actions.values()]}
        _atomic_write_json(self.actions_path, payload)

    def _save_workflows(self) -> None:
        payload = {"version": 2, "workflows": [w.model_dump() for w in self.workflows.values()]}
        _atomic_write_json(self.workflows_path, payload)

    def _save_jobs(self) -> None:
        ordered = sorted(self.jobs.values(), key=lambda j: j.created_at, reverse=True)[:100]
        payload = {"version": 1, "jobs": [j.model_dump() for j in ordered]}
        _atomic_write_json(self.jobs_path, payload)

    def _save_loop(self) -> None:
        payload = {"version": 1, "records": [r.model_dump() for r in self.loop_records.values()]}
        _atomic_write_json(self.loop_path, payload)

    def _save_waypoints(self) -> None:
        payload = {"version": 1, "waypoints": [w.model_dump() for w in self.waypoints.values()]}
        _atomic_write_json(self.waypoints_path, payload)

    def _save_map_edits(self) -> None:
        payload = {"version": 1, "overlays": [ov.model_dump() for ov in self.map_edits.values()]}
        _atomic_write_json(self.map_edits_path, payload)

    # ---- Actions ----
    def list_actions(self, robot_id: str | None = None) -> list[AtomicAction]:
        items = list(self.actions.values())
        if robot_id:
            items = [a for a in items if a.robot_id == robot_id]
        return sorted(items, key=lambda a: a.name)

    def get_action_by_name(self, robot_id: str, name: str) -> AtomicAction | None:
        for a in self.actions.values():
            if a.robot_id == robot_id and a.name == name:
                return a
        return None

    def upsert_action(self, data: dict[str, Any]) -> AtomicAction:
        now = utc_iso()
        act_id = data.get("id") or f"act-{uuid.uuid4().hex[:8]}"
        existing = self.actions.get(act_id)
        payload = {
            **data,
            "id": act_id,
            "created_at": existing.created_at if existing else now,
            "updated_at": now,
        }
        act = AtomicAction.model_validate(payload)
        self.actions[act.id] = act
        self._save_actions()
        return act

    def delete_action(self, action_id: str) -> bool:
        if action_id in self.actions:
            del self.actions[action_id]
            self._save_actions()
            return True
        return False

    # ---- R1 Pro actions.json & sequences.json Compatibility Import/Export ----
    def import_r1pro_actions(self, raw_payload: dict[str, Any], robot_id: str = "r1pro-mock") -> list[AtomicAction]:
        raw_list = raw_payload.get("actions", [])
        if isinstance(raw_list, dict):
            raw_list = list(raw_list.values())
        imported: list[AtomicAction] = []
        type_map = {
            "joint_targets": "joint_targets",
            "ee_plan": "ee_pose",
            "ee_pose": "ee_pose",
            "ee_plan_relative": "ee_relative",
            "ee_relative": "ee_relative",
            "ee_axial_motion": "ee_axial",
            "ee_axial": "ee_axial",
            "chassis_velocity": "chassis_velocity",
            "navigate_to_pose": "navigate_to_pose",
            "switch_camera": "perception_step",
            "yolo_open": "perception_step",
            "latch_yolo": "perception_step",
            "yolo_close": "perception_step",
            "read_yolo_detections": "perception_step",
            "grasp": "ee_pose",
            "read_ee_pose": "perception_step",
            "code_workflow": "policy_intent",
        }
        now = utc_iso()
        for item in raw_list:
            if not isinstance(item, dict) or not item.get("name"):
                continue
            src_type = item.get("type") or item.get("kind") or "joint_targets"
            kind = type_map.get(src_type, "perception_step")
            src_params = dict(item.get("params") or {})
            params: dict[str, Any] = dict(src_params)

            if kind == "joint_targets" and isinstance(src_params.get("targets"), list):
                converted: dict[str, float] = {}
                for t in src_params["targets"]:
                    if not isinstance(t, dict):
                        continue
                    jname = _r1_joint_name_to_canonical(t.get("name", ""), t.get("group"), t.get("index"))
                    val = _safe_finite(t.get("value", 0.0), 0.0)
                    unit = t.get("unit", "rad")
                    if unit == "deg":
                        val = round(math.radians(val), 5)
                    elif unit == "open_0_100":
                        val = round(max(0.0, min(1.0, val / 100.0)), 4)
                    converted[jname] = val
                params["targets"] = converted
                params.setdefault("duration_s", _safe_finite(src_params.get("stable_s") or 1.2, 1.2))
            elif kind == "navigate_to_pose":
                params["x"] = _safe_finite(src_params.get("x_m", src_params.get("x", 0.0)), 0.0)
                params["y"] = _safe_finite(src_params.get("y_m", src_params.get("y", 0.0)), 0.0)
                if "yaw_deg" in src_params:
                    params["yaw"] = round(math.radians(_safe_finite(src_params["yaw_deg"], 0.0)), 5)
                else:
                    params["yaw"] = _safe_finite(src_params.get("yaw", 0.0), 0.0)
            elif kind == "chassis_velocity":
                params["vx"] = _safe_finite(src_params.get("vx_mps", src_params.get("vx", 0.0)), 0.0)
                params["vy"] = _safe_finite(src_params.get("vy_mps", src_params.get("vy", 0.0)), 0.0)
                params["omega"] = _safe_finite(src_params.get("omega_radps", src_params.get("omega", 0.0)), 0.0)
                params["duration_s"] = _safe_finite(src_params.get("duration_s", 1.0), 1.0)

            act = AtomicAction(
                id=str(item.get("id") or f"r1-{uuid.uuid4().hex[:8]}"),
                name=str(item["name"]),
                robot_id=robot_id,
                kind=kind,  # type: ignore[arg-type]
                description=str(item.get("description") or f"Imported from R1 Pro ({src_type})"),
                params=params,
                tags=["r1pro_imported", src_type],
                created_at=str(item.get("created_at") or now),
                updated_at=now,
            )
            self.actions[act.id] = act
            imported.append(act)
        if imported:
            self._save_actions()
        return imported

    def import_r1pro_sequences(self, raw_payload: dict[str, Any], robot_id: str = "r1pro-mock") -> list[WorkflowDefinition]:
        raw_list = raw_payload.get("sequences", raw_payload.get("workflows", []))
        if isinstance(raw_list, dict):
            raw_list = list(raw_list.values())
        imported: list[WorkflowDefinition] = []
        now = utc_iso()
        for seq in raw_list:
            if not isinstance(seq, dict) or not seq.get("name"):
                continue
            steps: list[WorkflowStep] = []
            for idx, st in enumerate(seq.get("steps", [])):
                if not isinstance(st, dict):
                    continue
                act_names = st.get("actions") or ([st["action_name"]] if st.get("action_name") else [])
                for sub_idx, aname in enumerate(act_names):
                    overrides = (st.get("overrides") or {}).get(aname, st.get("overrides") or {})
                    steps.append(
                        WorkflowStep(
                            id=f"{st.get('id', f'st-{idx}')}-{sub_idx}",
                            action_name=str(aname),
                            delay_s=float(st.get("delay_s") or 0.0),
                            duration_s=float(st.get("duration_override_s") or st.get("duration_s") or 1.0),
                            overrides=overrides if isinstance(overrides, dict) else {},
                        )
                    )
            wf = WorkflowDefinition(
                id=str(seq.get("id") or f"wf-r1-{uuid.uuid4().hex[:8]}"),
                name=str(seq["name"]),
                robot_id=robot_id,
                description=str(seq.get("description") or "Imported from R1 Pro sequences.json"),
                kind="Workflow",
                steps=steps,
                created_at=str(seq.get("created_at") or now),
                updated_at=now,
            )
            self.workflows[wf.id] = wf
            imported.append(wf)
        if imported:
            self._save_workflows()
        return imported

    def export_r1pro_bundle(self, robot_id: str) -> dict[str, Any]:
        acts = self.list_actions(robot_id)
        wfs = self.list_workflows(robot_id)
        return {
            "actions_json": {
                "version": 2,
                "actions": [
                    {
                        "id": a.id,
                        "name": a.name,
                        "type": a.kind,
                        "description": a.description,
                        "params": a.params,
                        "created_at": a.created_at,
                        "updated_at": a.updated_at,
                    }
                    for a in acts
                ],
            },
            "sequences_json": {
                "version": 2,
                "sequences": [
                    {
                        "id": w.id,
                        "name": w.name,
                        "description": w.description,
                        "steps": [
                            {
                                "id": s.id,
                                "actions": [s.action_name],
                                "delay_s": s.delay_s,
                                "duration_override_s": s.duration_s,
                                "overrides": {s.action_name: s.overrides} if s.overrides else {},
                                "trigger": "after_end",
                            }
                            for s in w.steps
                        ],
                        "created_at": w.created_at,
                        "updated_at": w.updated_at,
                    }
                    for w in wfs
                ],
            },
        }

    # ---- Workflows ----
    def list_workflows(self, robot_id: str | None = None) -> list[WorkflowDefinition]:
        items = list(self.workflows.values())
        if robot_id:
            items = [w for w in items if w.robot_id == robot_id]
        return sorted(items, key=lambda w: w.name)

    def upsert_workflow(self, data: dict[str, Any]) -> WorkflowDefinition:
        now = utc_iso()
        wf_id = data.get("id") or f"wf-{uuid.uuid4().hex[:8]}"
        existing = self.workflows.get(wf_id)
        raw_steps = []
        for idx, s in enumerate(data.get("steps", [])):
            raw_steps.append({**s, "id": s.get("id") or f"st-{idx + 1}-{uuid.uuid4().hex[:4]}"})
        payload = {
            **data,
            "id": wf_id,
            "steps": raw_steps,
            "created_at": existing.created_at if existing else now,
            "updated_at": now,
        }
        wf = WorkflowDefinition.model_validate(payload)
        self.workflows[wf.id] = wf
        self._save_workflows()
        return wf

    def delete_workflow(self, workflow_id: str) -> bool:
        if workflow_id in self.workflows:
            del self.workflows[workflow_id]
            self._save_workflows()
            return True
        return False

    # ---- Jobs ----
    def list_jobs(self, robot_id: str | None = None, state: str | None = None, kind: str | None = None) -> list[ExecutionJob]:
        items = list(self.jobs.values())
        if robot_id:
            items = [j for j in items if j.robot_id == robot_id]
        if state and state != "ALL":
            items = [j for j in items if j.state == state]
        if kind and kind != "ALL":
            items = [j for j in items if j.kind == kind]
        return sorted(items, key=lambda j: j.created_at, reverse=True)

    def save_job(self, job: ExecutionJob) -> ExecutionJob:
        self.jobs[job.job_id] = job
        if len(self.jobs) > 200:
            running = {jid: j for jid, j in self.jobs.items() if j.state == "Running"}
            finished = sorted(
                (j for j in self.jobs.values() if j.state != "Running"),
                key=lambda j: j.created_at,
                reverse=True,
            )[: max(0, 200 - len(running))]
            self.jobs = {**running, **{j.job_id: j for j in finished}}
        self._save_jobs()
        return job

    def get_job(self, job_id: str) -> ExecutionJob | None:
        return self.jobs.get(job_id)

    # ---- Loop Records ----
    def list_loop_records(self, robot_id: str | None = None) -> list[LoopRecord]:
        items = list(self.loop_records.values())
        if robot_id:
            items = [r for r in items if r.robot_id == robot_id]
        return sorted(items, key=lambda r: r.updated_at, reverse=True)

    def upsert_loop_record(self, data: dict[str, Any]) -> LoopRecord:
        now = utc_iso()
        rec_id = data.get("record_id") or f"loop-rec-{uuid.uuid4().hex[:6]}"
        existing = self.loop_records.get(rec_id)
        payload = {
            **data,
            "record_id": rec_id,
            "created_at": existing.created_at if existing else now,
            "updated_at": now,
        }
        rec = LoopRecord.model_validate(payload)
        self.loop_records[rec.record_id] = rec
        self._save_loop()
        return rec

    def delete_loop_record(self, record_id: str) -> bool:
        if record_id in self.loop_records:
            del self.loop_records[record_id]
            self._save_loop()
            return True
        return False

    # ---- Navigation Waypoints ----
    def list_waypoints(self, robot_id: str | None = None, map_id: str | None = None) -> list[NavWaypoint]:
        items = list(self.waypoints.values())
        if robot_id:
            items = [w for w in items if w.robot_id == robot_id]
        if map_id:
            items = [w for w in items if w.map_id == map_id]
        return sorted(items, key=lambda w: w.name)

    def upsert_waypoint(self, data: dict[str, Any]) -> NavWaypoint:
        now = utc_iso()
        wp_id = data.get("id") or f"wp-{uuid.uuid4().hex[:8]}"
        existing = self.waypoints.get(wp_id)
        payload = {
            **data,
            "id": wp_id,
            "created_at": existing.created_at if existing else now,
        }
        wp = NavWaypoint.model_validate(payload)
        self.waypoints[wp.id] = wp
        self._save_waypoints()
        return wp

    def delete_waypoint(self, waypoint_id: str) -> bool:
        if waypoint_id in self.waypoints:
            del self.waypoints[waypoint_id]
            self._save_waypoints()
            return True
        return False

    # ---- 2D Occupancy Map Non-Destructive Edits (Virtual Walls + Noise Eraser + Denoise Level) ----
    def get_map_edits(self, map_id: str) -> MapEditOverlay:
        mid = str(map_id or "fast_slam_20260914").strip()
        existing = self.map_edits.get(mid)
        if existing is not None:
            return existing
        return MapEditOverlay(map_id=mid, denoise_level=1, virtual_walls=[], erase_strokes=[], updated_at="")

    def save_map_edits(self, map_id: str, data: dict[str, Any]) -> MapEditOverlay:
        mid = str(map_id or "").strip()
        if not mid:
            raise ValueError("map_id is required")
        walls = list(data.get("virtual_walls") or [])[:200]
        strokes = list(data.get("erase_strokes") or [])[:400]
        for st in strokes:
            if isinstance(st, dict) and isinstance(st.get("points"), list):
                st["points"] = [
                    [_safe_finite(p[0]), _safe_finite(p[1])]
                    for p in st["points"][:300]
                    if isinstance(p, (list, tuple)) and len(p) >= 2
                ]
        payload = {
            "map_id": mid,
            "denoise_level": int(data.get("denoise_level", 1)),
            "virtual_walls": walls,
            "erase_strokes": strokes,
            "updated_at": utc_iso(),
        }
        ov = MapEditOverlay.model_validate(payload)
        self.map_edits[mid] = ov
        self._save_map_edits()
        return ov

    def reset_map_edits(self, map_id: str) -> MapEditOverlay:
        mid = str(map_id or "").strip()
        if mid in self.map_edits:
            del self.map_edits[mid]
            self._save_map_edits()
        return MapEditOverlay(map_id=mid, denoise_level=1, virtual_walls=[], erase_strokes=[], updated_at=utc_iso())
