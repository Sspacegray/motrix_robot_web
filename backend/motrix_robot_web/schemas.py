from __future__ import annotations

from typing import Any, Literal
from pydantic import BaseModel, Field

RobotMode = Literal["mock", "sim", "real"]
ConnectorType = Literal["mock", "edge", "r1pro", "piper_lelab", "dex_w1"]
TaskPermission = Literal["viewer", "operator", "admin"]
NavMode = Literal["MANUAL", "SWITCHING", "NAVIGATION", "FAULT"]
JobState = Literal["Pending", "Running", "Succeeded", "Failed", "Canceled"]
JobKind = Literal["Manual", "Workflow", "Eval", "Inference", "Calibration"]
EvalStatus = Literal["passed", "failed", "zero_success", "no_result", "unknown"]


class JointSpec(BaseModel):
    name: str
    label: str
    group: Literal["left_arm", "right_arm", "torso", "head", "gripper", "leg"]
    unit: Literal["rad", "norm"] = "rad"
    lower: float
    upper: float
    default: float = 0.0
    is_gripper: bool = False
    urdf_joint: str | None = None
    max_step_rad: float = 0.15


class CameraSpec(BaseModel):
    key: str
    label: str
    resolution: str = "640x480"
    has_depth: bool = False
    fps_target: float = 15.0


class RobotCapabilities(BaseModel):
    joints: bool = True
    urdf_3d: bool = True
    motrix_mirror: bool = True
    ee_6d: bool = True
    atomic_actions: bool = True
    workflows: bool = True
    navigation: bool = False
    chassis_teleop: bool = False
    cameras: bool = True
    head_depth: bool = False
    yolo_foundation_pose: bool = False
    aruco: bool = False
    policy_inference: bool = False
    dynamic_checkpoints: bool = False
    offline_replay: bool = False
    piper_safety_wizard: bool = False
    teach_arms_only: bool = True
    loop_registry: bool = True


class RobotDescriptor(BaseModel):
    id: str
    name: str
    robot_type: Literal["r1pro", "piper_dual", "dexforce_w1"]
    mode: RobotMode = "mock"
    connector_type: ConnectorType = "mock"
    readonly_only: bool = False
    arms_scope: list[str] = Field(default_factory=lambda: ["left", "right"])
    urdf_url: str | None = None
    map_image_url: str | None = None
    map_yaml: dict[str, Any] | None = None
    network_note: str = ""
    safety_note: str = ""
    joints: list[JointSpec]
    cameras: list[CameraSpec]
    capabilities: RobotCapabilities


class MotorDiag(BaseModel):
    joint: str
    enabled: bool = True
    temperature_c: float = 34.0
    effort_nm: float = 0.0
    error_code: int = 0
    statusword: str = "0x0637"
    severity: Literal["ok", "warn", "block"] = "ok"


class InterlockItem(BaseModel):
    code: str
    label: str
    severity: Literal["ok", "warn", "block"] = "ok"
    detail: str = ""


class BatteryState(BaseModel):
    percentage: float = 88.0
    voltage_v: float = 48.2
    current_a: float = -2.4
    source: str = "edge_obs"


class EEPose(BaseModel):
    arm: str
    frame: str = "base_link"
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0


class NavigationState(BaseModel):
    supported: bool = False
    mode: NavMode = "MANUAL"
    navigation_ready: bool = True
    localization_spin_active: bool = False
    pose_map: dict[str, float] = Field(default_factory=lambda: {"x": 0.0, "y": 0.0, "yaw": 0.0})
    goal_map: dict[str, float] | None = None
    chassis_vel: dict[str, float] = Field(default_factory=lambda: {"vx": 0.0, "vy": 0.0, "omega": 0.0})
    path_points: list[list[float]] = Field(default_factory=list)


class PerceptionState(BaseModel):
    head_depth_enabled: bool = False
    head_depth_ready: bool = False
    head_depth_fps: float = 0.0
    yolo_running: bool = False
    yolo_camera: str = "head_left_rgb"
    yolo_prompt: str = "bottle"
    yolo_conf: float = 0.4
    yolo_generation: int = 0
    yolo_detections: list[dict[str, Any]] = Field(default_factory=list)
    latched_mask: dict[str, Any] | None = None
    foundation_pose_running: bool = False
    foundation_pose_phase: Literal["idle", "waiting", "registering", "pose_ready", "tracking"] = "idle"
    foundation_pose_camera: str = "head_left_rgb"
    foundation_pose_mesh: str = "drink_Wahaha/assets/V_wahaha_mi.obj"
    foundation_pose_result: dict[str, Any] | None = None
    aruco_pose_running: bool = False
    aruco_fine_enabled: bool = False
    aruco_markers: list[dict[str, Any]] = Field(default_factory=list)
    last_3d_pick: dict[str, Any] | None = None
    logs: list[str] = Field(default_factory=list)


class CheckpointInfo(BaseModel):
    id: str
    name: str
    family: str = "pi05"
    action_dim: int = 14
    rtc_supported: bool = True
    path: str
    compatible: bool = True
    note: str = ""


class PolicyState(BaseModel):
    connected: bool = True
    endpoint: str = "edge://policy/rtc"
    active_checkpoint: str = ""
    available_checkpoints: list[CheckpointInfo] = Field(default_factory=list)
    execution_schema: str = "bimanual"
    rtc_enabled: bool = True
    infer_running: bool = False
    infer_hz: float = 0.0
    task_intent: str = "set_the_plates"
    speed_scale: float = 0.35
    wizard_steps: dict[str, bool] = Field(
        default_factory=lambda: {
            "self_check": True,
            "homing": False,
            "manual_slow_test": False,
            "offline_replay": False,
            "live_armed": False,
        }
    )
    last_offline_eval: dict[str, Any] | None = None
    recorded_episodes: list[dict[str, Any]] = Field(default_factory=list)


class ObservationSnapshot(BaseModel):
    robot_id: str
    timestamp: float
    connected: bool = True
    mode: RobotMode = "mock"
    feedback_age_ms: float = 18.0
    measured_hz: float = 30.0
    joint_positions: dict[str, float]
    joint_targets: dict[str, float]
    joint_velocities: dict[str, float]
    joint_efforts: dict[str, float]
    ee_poses: dict[str, EEPose] = Field(default_factory=dict)
    motors: list[MotorDiag] = Field(default_factory=list)
    interlocks: list[InterlockItem] = Field(default_factory=list)
    battery: BatteryState = Field(default_factory=BatteryState)
    navigation: NavigationState = Field(default_factory=NavigationState)
    perception: PerceptionState = Field(default_factory=PerceptionState)
    policy: PolicyState = Field(default_factory=PolicyState)
    motrix_mirror_clients: int = 0
    topics: list[dict[str, Any]] = Field(default_factory=list)


class AuthorityState(BaseModel):
    robot_id: str
    readonly_only: bool = False
    # Layer 1: High-level task permission (never substitutes for Edge Lease)
    task_permission: TaskPermission = "operator"
    operator_name: str = "operator@motphys"
    # Layer 2: Underlying Edge Lease / Control Lock
    lease_active: bool = False
    lease_id: str | None = None
    lease_source: str = "none"
    lease_ttl_s: float = 120.0
    lease_remaining_s: float = 0.0
    heartbeat_age_s: float = 0.0
    hb_timeout_s: float = 3.0
    idle_age_s: float = 0.0
    idle_timeout_s: float = 300.0
    # Safety latch & semantics
    estop_latched: bool = False
    estop_reason: str = ""
    estop_by: str = ""
    estop_at: str = ""
    teach_arms_only: bool = True
    software_stop_disclaimer: str = "软件 E-Stop = safe_stop（保持位姿/停下发），不等同于现场硬件断电急停"


class AtomicAction(BaseModel):
    id: str
    name: str
    robot_id: str
    kind: Literal[
        "joint_targets",
        "ee_pose",
        "ee_relative",
        "ee_axial",
        "chassis_velocity",
        "navigate_to_pose",
        "perception_step",
        "policy_intent",
    ]
    description: str = ""
    params: dict[str, Any] = Field(default_factory=dict)
    tags: list[str] = Field(default_factory=list)
    created_at: str = ""
    updated_at: str = ""


class WorkflowStep(BaseModel):
    id: str
    action_name: str
    delay_s: float = 0.0
    duration_s: float = 1.5
    overrides: dict[str, Any] = Field(default_factory=dict)


class WorkflowDefinition(BaseModel):
    id: str
    name: str
    robot_id: str
    description: str = ""
    kind: JobKind = "Workflow"
    steps: list[WorkflowStep] = Field(default_factory=list)
    created_at: str = ""
    updated_at: str = ""


class JobStep(BaseModel):
    step: str
    phase: Literal["pending", "running", "succeeded", "failed", "canceled"] = "pending"
    detail: str = ""
    started_at: str | None = None
    finished_at: str | None = None


class ExecutionJob(BaseModel):
    job_id: str
    run_id: str
    flow_name: str
    kind: JobKind = "Workflow"
    state: JobState = "Running"
    robot_id: str
    workflow_id: str | None = None
    created_at: str
    started_at: str
    finished_at: str | None = None
    exit_code: int | None = None
    pipeline_sha: str = "motrix-hl-v1-a4f92c1"
    operator: str = "operator@motphys"
    lease_id: str | None = None
    steps: list[JobStep] = Field(default_factory=list)
    logs: list[str] = Field(default_factory=list)


class LoopRecord(BaseModel):
    record_id: str
    robot_id: str
    task_name: str
    owner: str = "wubinxin"
    source_type: Literal["training_run", "existing_model"] = "existing_model"
    source_ref: str = ""
    model_artifact: str = ""
    eval_plan: str = ""
    eval_status: EvalStatus = "unknown"
    success_rate: float | None = None
    episodes_count: int = 0
    evidence_run_id: str = ""
    notes: str = ""
    next_action: str = ""
    created_at: str = ""
    updated_at: str = ""


class LeaseRequest(BaseModel):
    operator_name: str = "operator@motphys"
    task_permission: TaskPermission = "operator"
    token: str = ""
    confirm_safety: bool = False


class JointTargetsRequest(BaseModel):
    targets: dict[str, float]
    speed_scale: float = 0.4
    duration_s: float = 1.2


class EEStepRequest(BaseModel):
    arm: Literal["left", "right"] = "left"
    dx: float = 0.0
    dy: float = 0.0
    dz: float = 0.0
    droll_deg: float = 0.0
    dpitch_deg: float = 0.0
    dyaw_deg: float = 0.0


class ChassisVelRequest(BaseModel):
    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0
    duration_s: float = 0.4


class NavGoalRequest(BaseModel):
    x: float
    y: float
    yaw: float = 0.0


class EstopRequest(BaseModel):
    source: str = "web_ui"
    reason: str = "Operator triggered global E-Stop"
