from __future__ import annotations

import html
import math
import time
import uuid
from pathlib import Path
from typing import Any

from ..safety import SafetyError, utc_iso
from ..schemas import (
    BatteryState,
    CameraSpec,
    CheckpointInfo,
    EEPose,
    InterlockItem,
    JointSpec,
    MotorDiag,
    NavigationState,
    ObservationSnapshot,
    PerceptionState,
    PolicyState,
    RobotCapabilities,
    RobotDescriptor,
)
from .base import RobotConnector


def _r1pro_descriptor() -> RobotDescriptor:
    joints: list[JointSpec] = [
        JointSpec(name="torso_1", label="Torso 1 (Lower Pitch)", group="torso", lower=-1.13, upper=1.83, default=0.0, urdf_joint="torso_joint1"),
        JointSpec(name="torso_2", label="Torso 2 (Mid Pitch)", group="torso", lower=-2.79, upper=2.53, default=0.0, urdf_joint="torso_joint2"),
        JointSpec(name="torso_3", label="Torso 3 (Upper Pitch)", group="torso", lower=-1.83, upper=1.57, default=0.0, urdf_joint="torso_joint3"),
        JointSpec(name="torso_4", label="Torso 4 (Yaw)", group="torso", lower=-3.05, upper=3.05, default=0.0, urdf_joint="torso_joint4"),
    ]
    arm_defaults_l = [0.22, 0.35, 0.0, -0.85, 0.0, 0.30, 0.0]
    arm_defaults_r = [-0.22, -0.35, 0.0, -0.85, 0.0, 0.30, 0.0]
    for i in range(1, 8):
        joints.append(
            JointSpec(
                name=f"left_arm_{i}",
                label=f"Left Arm J{i}",
                group="left_arm",
                lower=-2.95,
                upper=2.95,
                default=arm_defaults_l[i - 1],
                urdf_joint=f"left_arm_joint{i}",
            )
        )
    joints.append(
        JointSpec(
            name="left_gripper",
            label="Left Gripper",
            group="gripper",
            unit="norm",
            lower=0.0,
            upper=1.0,
            default=0.75,
            is_gripper=True,
            urdf_joint="left_gripper_finger_joint1",
            urdf_scale=0.05,
        )
    )
    for i in range(1, 8):
        joints.append(
            JointSpec(
                name=f"right_arm_{i}",
                label=f"Right Arm J{i}",
                group="right_arm",
                lower=-2.95,
                upper=2.95,
                default=arm_defaults_r[i - 1],
                urdf_joint=f"right_arm_joint{i}",
            )
        )
    joints.append(
        JointSpec(
            name="right_gripper",
            label="Right Gripper",
            group="gripper",
            unit="norm",
            lower=0.0,
            upper=1.0,
            default=0.75,
            is_gripper=True,
            urdf_joint="right_gripper_finger_joint1",
            urdf_scale=0.05,
        )
    )

    return RobotDescriptor(
        id="r1pro-mock",
        name="Galaxea R1 Pro",
        robot_type="r1pro",
        mode="mock",
        connector_type="mock",
        arms_scope=["left", "right"],
        urdf_url="/assets/urdf/galaxea_r1pro/robot.urdf",
        map_image_url="/assets/maps/fast_slam_20260914/bag_20260914_map.png",
        map_yaml={"resolution": 0.05, "origin": [-14.8, -29.1, 0.0], "width": 709, "height": 755},
        network_note="真机位于 pc10 (192.168.3.x 独立 NAT / ROS_DOMAIN_ID=102)，过渡期保留 webv2，禁止双入口并发写。",
        safety_note="示教仅允许释放双臂；躯干/底盘/头部绝不释放。软件 E-Stop 不等同于硬急停。",
        joints=joints,
        cameras=[
            CameraSpec(key="head_left_rgb", label="Head Left RGB (Rectified)", resolution="1920x1080", has_depth=True, fps_target=15.0),
            CameraSpec(key="head_right_rgb", label="Head Right RGB", resolution="1920x1080", has_depth=False, fps_target=15.0),
            CameraSpec(key="wrist_left_rgb", label="Wrist Left RGB-D", resolution="640x480", has_depth=True, fps_target=30.0),
            CameraSpec(key="wrist_right_rgb", label="Wrist Right RGB-D", resolution="640x480", has_depth=True, fps_target=30.0),
        ],
        capabilities=RobotCapabilities(
            joints=True,
            urdf_3d=True,
            motrix_mirror=True,
            ee_6d=True,
            atomic_actions=True,
            workflows=True,
            navigation=True,
            chassis_teleop=True,
            cameras=True,
            head_depth=True,
            yolo_foundation_pose=True,
            aruco=True,
            policy_inference=True,
            dynamic_checkpoints=True,
            offline_replay=True,
            piper_safety_wizard=False,
            teach_arms_only=True,
            loop_registry=True,
        ),
    )


def _piper_descriptor() -> RobotDescriptor:
    limits = [
        (-2.61, 2.61),
        (0.0, 3.14),
        (-2.967, 0.0),
        (-1.74, 1.74),
        (-1.22, 1.22),
        (-2.09, 2.09),
    ]
    defaults = [0.0, 0.65, -0.95, 0.0, 0.45, 0.0]
    joints: list[JointSpec] = []
    for side in ("left", "right"):
        grp = "left_arm" if side == "left" else "right_arm"
        prefix = "Left" if side == "left" else "Right"
        for i in range(1, 7):
            lo, hi = limits[i - 1]
            joints.append(
                JointSpec(
                    name=f"{side}_j{i}",
                    label=f"{prefix} Piper J{i}",
                    group=grp,
                    lower=lo,
                    upper=hi,
                    default=defaults[i - 1],
                    urdf_joint=f"{side}_joint{i}",
                )
            )
        joints.append(
            JointSpec(
                name=f"{side}_gripper",
                label=f"{prefix} AGX Gripper",
                group="gripper",
                unit="norm",
                lower=0.0,
                upper=1.0,
                default=0.85,
                is_gripper=True,
                urdf_joint=f"{side}_gripper_joint1",
                urdf_scale=0.05,
            )
        )
    return RobotDescriptor(
        id="piper-dual-mock",
        name="AgileX Piper Dual-Arm",
        robot_type="piper_dual",
        mode="mock",
        connector_type="mock",
        arms_scope=["left", "right"],
        urdf_url="/assets/urdf/piper_dual_d405/robot.urdf",
        network_note="主路径走 motrix-edge (DualPiperAdapter 14-DOF) + Robot Pipeline；过渡期可互斥桥接 motphys:8095 + omah:18100。",
        safety_note="Piper 唯一 CAN 写者归 Robot Pipeline，严禁出现第二 CAN 写者；不碰 motphys 真控制端与 oldpiper。",
        joints=joints,
        cameras=[
            CameraSpec(key="cam_head", label="RealSense Head", resolution="640x480", has_depth=True, fps_target=30.0),
            CameraSpec(key="cam_left_wrist", label="RealSense Left Wrist", resolution="640x480", has_depth=True, fps_target=30.0),
            CameraSpec(key="cam_right_wrist", label="RealSense Right Wrist", resolution="640x480", has_depth=True, fps_target=30.0),
        ],
        capabilities=RobotCapabilities(
            joints=True,
            urdf_3d=True,
            motrix_mirror=True,
            ee_6d=True,
            atomic_actions=True,
            workflows=True,
            navigation=False,
            chassis_teleop=False,
            cameras=True,
            head_depth=False,
            yolo_foundation_pose=False,
            aruco=False,
            policy_inference=True,
            dynamic_checkpoints=True,
            offline_replay=True,
            piper_safety_wizard=True,
            teach_arms_only=True,
            loop_registry=True,
        ),
    )


def _w1_descriptor() -> RobotDescriptor:
    raw = [
        ("ANKLE_PITCH", "Ankle Pitch", "leg", -1.57, 1.57, 0.0),
        ("KNEE_PITCH", "Knee Pitch", "leg", -2.70, 2.70, 0.0),
        ("BUTTOCK_PITCH", "Buttock Pitch", "torso", -1.57, 1.91, 0.0),
        ("WAIST_YAW", "Waist Yaw", "torso", -3.75, 2.18, 0.0),
        ("HEAD_YAW_J1", "Head Yaw J1", "head", -1.57, 1.57, 0.0),
        ("HEAD_PITCH_J2", "Head Pitch J2", "head", -0.69, 0.78, 0.0),
        ("SHOULDER_PITCH_L_J1", "L Shoulder Pitch J1", "left_arm", -2.96, 2.96, 0.18),
        ("SHOULDER_ROLL_L_J2", "L Shoulder Roll J2", "left_arm", -2.09, 1.57, 0.12),
        ("ELBOW_PITCH_L_J3", "L Elbow Pitch J3", "left_arm", -2.96, 2.96, -0.62),
        ("ELBOW_YAW_L_J4", "L Elbow Yaw J4", "left_arm", -2.58, 1.57, 0.0),
        ("WRIST_PITCH_L_J5", "L Wrist Pitch J5", "left_arm", -2.96, 2.96, 0.0),
        ("WRIST_YAW_L_J6", "L Wrist Yaw J6", "left_arm", -1.65, 1.65, 0.0),
        ("WRIST_ROLL_L_J7", "L Wrist Roll J7", "left_arm", -1.74, 1.74, 0.0),
        ("SHOULDER_PITCH_R_J1", "R Shoulder Pitch J1", "right_arm", -2.96, 2.96, 0.18),
        ("SHOULDER_ROLL_R_J2", "R Shoulder Roll J2", "right_arm", -1.57, 2.09, -0.12),
        ("ELBOW_PITCH_R_J3", "R Elbow Pitch J3", "right_arm", -2.96, 2.96, -0.62),
        ("ELBOW_YAW_R_J4", "R Elbow Yaw J4", "right_arm", -1.57, 2.58, 0.0),
        ("WRIST_PITCH_R_J5", "R Wrist Pitch J5", "right_arm", -2.96, 2.96, 0.0),
        ("WRIST_YAW_R_J6", "R Wrist Yaw J6", "right_arm", -1.65, 1.65, 0.0),
        ("WRIST_ROLL_R_J7", "R Wrist Roll J7", "right_arm", -1.74, 1.74, 0.0),
    ]
    joints = [
        JointSpec(name=n, label=lbl, group=grp, lower=lo, upper=hi, default=d, urdf_joint=n)  # type: ignore[arg-type]
        for n, lbl, grp, lo, hi, d in raw
    ]
    return RobotDescriptor(
        id="dex-w1-mock",
        name="Dexforce W1 Humanoid",
        robot_type="dexforce_w1",
        mode="mock",
        connector_type="mock",
        arms_scope=["left", "right"],
        urdf_url="/assets/urdf/dexforce_w1_v030/robot.urdf",
        network_note="真机位于 PC2 (10.3.130.154:8090) + L40 Motrix 查看器；按路线图以只读镜像和 UI 复用为主，不重建 W1 ROS 底座。",
        safety_note="温度 ≥75°C / 电机故障 / 反馈超时 >1s 自动阻断；Motrix 查看器与 3D 视图严格只读。",
        joints=joints,
        cameras=[
            CameraSpec(key="head_cam", label="W1 Head Camera", resolution="1280x720", has_depth=True, fps_target=15.0),
            CameraSpec(key="wrist_cam", label="W1 Wrist Camera", resolution="640x480", has_depth=False, fps_target=15.0),
        ],
        capabilities=RobotCapabilities(
            joints=True,
            urdf_3d=True,
            motrix_mirror=True,
            ee_6d=True,
            atomic_actions=True,
            workflows=True,
            navigation=False,
            chassis_teleop=False,
            cameras=True,
            head_depth=False,
            yolo_foundation_pose=False,
            aruco=False,
            policy_inference=False,
            dynamic_checkpoints=False,
            offline_replay=False,
            piper_safety_wizard=False,
            teach_arms_only=True,
            loop_registry=True,
        ),
    )


class MockRobotConnector(RobotConnector):
    """Full-fidelity deterministic mock simulator for built-in and dynamically imported robots."""

    def __init__(self, descriptor: RobotDescriptor, custom_robots_dir: Path | None = None) -> None:
        self.descriptor = descriptor
        self._positions: dict[str, float] = {j.name: j.default for j in descriptor.joints}
        self._targets: dict[str, float] = dict(self._positions)
        self._velocities: dict[str, float] = {j.name: 0.0 for j in descriptor.joints}
        self._efforts: dict[str, float] = {j.name: 0.0 for j in descriptor.joints}
        self._temps: dict[str, float] = {j.name: 34.0 + (idx % 5) * 1.1 for idx, j in enumerate(descriptor.joints)}
        self._errors: dict[str, int] = {j.name: 0 for j in descriptor.joints}
        self._fault_stale: bool = False
        self._sim_time: float = 0.0
        self._last_obs_ts: float = time.time()

        # Real URDF FK/IK solver (lazy-safe offline pure-Python kinematics)
        self._kin = None
        try:
            from ..kinematics import RobotKinematics
            self._kin = RobotKinematics(descriptor, custom_robots_dir=custom_robots_dir)
        except Exception:
            self._kin = None

        # Fallback end-effector state when no URDF kinematics
        self._ee_offsets: dict[str, list[float]] = {
            "left": [0.42, 0.24, 0.96, 0.0, 12.0, 0.0],
            "right": [0.42, -0.24, 0.96, 0.0, 12.0, 0.0],
        }

        # Navigation state
        self._nav = NavigationState(
            supported=descriptor.capabilities.navigation,
            mode="MANUAL",
            navigation_ready=True,
            pose_map={"x": 0.0, "y": 0.0, "yaw": 0.0},
            goal_map=None,
            chassis_vel={"vx": 0.0, "vy": 0.0, "omega": 0.0},
            path_points=[],
        )
        self._chassis_cmd_until: float = 0.0
        self._spin_remaining_rad: float = 0.0

        # Perception state
        self._perc = PerceptionState(
            head_depth_enabled=False,
            head_depth_ready=False,
            head_depth_fps=0.0,
            yolo_camera=descriptor.cameras[0].key if descriptor.cameras else "head_left_rgb",
            foundation_pose_camera=descriptor.cameras[0].key if descriptor.cameras else "head_left_rgb",
            logs=["[perception] Perception pipeline ready (head_depth=OFF)"],
        )
        self._fp_register_elapsed: float = 0.0

        # Policy state
        if descriptor.robot_type == "piper_dual":
            ckpts = [
                CheckpointInfo(
                    id="pi05_real_all_mcap_20261119_base_30k_v1/030000",
                    name="pi0.5 Real All MCAP 30k (v1)",
                    family="pi05",
                    action_dim=14,
                    rtc_supported=True,
                    path="checkpoints/pi05_real_all_mcap_20261119_base_30k_v1/030000",
                    compatible=True,
                    note="omah:18100 当前在跑基线 (max_first_step=0.45)",
                ),
                CheckpointInfo(
                    id="pi05_real_noframe_136_30k/030000",
                    name="pi0.5 NoFrame 136 30k",
                    family="pi05",
                    action_dim=14,
                    rtc_supported=True,
                    path="checkpoints/pi05_real_noframe_136_30k/030000",
                    compatible=True,
                    note="无边框增广 136 条真机轨迹",
                ),
                CheckpointInfo(
                    id="act_real_shift0_241/060000",
                    name="ACT Real Shift0 241 60k",
                    family="act",
                    action_dim=14,
                    rtc_supported=False,
                    path="checkpoints/act_real_shift0_241/060000",
                    compatible=True,
                    note="轻量 ACT 基线模型（不支持 RTC）",
                ),
                CheckpointInfo(
                    id="r1pro_v2_upper_body_20dof/050000",
                    name="R1 Pro Upper Body 20-DOF (Incompatible)",
                    family="openpi_r1pro_v2",
                    action_dim=20,
                    rtc_supported=False,
                    path="checkpoints/r1pro_v2_upper_body_20dof/050000",
                    compatible=False,
                    note="维度 20 ≠ DualPiper 14，用于验证布局契约预检拦截",
                ),
            ]
            active_ckpt = ckpts[0].id
            endpoint = "http://10.30.20.8:18100 (OpenPI Piper v1)"
            schema = "dual_piper_14"
            intent = "set_the_plates"
        else:
            ckpts = [
                CheckpointInfo(
                    id="r1pro_v2_water_tap_lora/040000",
                    name="OpenPI r1pro.v2 Water Tap LoRA 40k",
                    family="openpi_r1pro_v2",
                    action_dim=20,
                    rtc_supported=False,
                    path="r1pro.v2/water_tap_40k",
                    compatible=True,
                    note="支持 execution_schema 按组裁剪 (left_arm_8 / bimanual_16 / upper_body_20)",
                ),
                CheckpointInfo(
                    id="r1pro_v2_basket_pick/030000",
                    name="OpenPI r1pro.v2 Basket Pick 30k",
                    family="openpi_r1pro_v2",
                    action_dim=20,
                    rtc_supported=False,
                    path="r1pro.v2/basket_pick_30k",
                    compatible=True,
                    note="双臂料盆搬运 VLA 检查点",
                ),
                CheckpointInfo(
                    id="piper_14dof_plates/030000",
                    name="Piper 14-DOF Plates (Incompatible)",
                    family="pi05",
                    action_dim=14,
                    rtc_supported=True,
                    path="checkpoints/pi05_real_all_mcap_20261119_base_30k_v1/030000",
                    compatible=False,
                    note="维度 14 ≠ R1 Pro 20，用于验证布局契约预检拦截",
                ),
            ]
            active_ckpt = ckpts[0].id
            endpoint = "ws://10.30.20.1:18081 (OpenPI r1pro.v2)"
            schema = "upper_body_20"
            intent = "grasp_and_place_basket"

        self._policy = PolicyState(
            connected=descriptor.capabilities.policy_inference,
            endpoint=endpoint,
            active_checkpoint=active_ckpt,
            available_checkpoints=ckpts,
            execution_schema=schema,
            rtc_enabled=(descriptor.robot_type == "piper_dual"),
            infer_running=False,
            infer_hz=0.0,
            task_intent=intent,
            speed_scale=0.35,
            recorded_episodes=[
                {
                    "id": "ep-001",
                    "timestamp": "2026-11-23T14:28:11Z",
                    "checkpoint": active_ckpt,
                    "frames": 412,
                    "outcome": "passed",
                    "notes": "动作平滑，无跳变或跟踪保持",
                },
                {
                    "id": "ep-002",
                    "timestamp": "2026-11-23T14:31:45Z",
                    "checkpoint": active_ckpt,
                    "frames": 289,
                    "outcome": "zero_success",
                    "notes": "夹爪提前闭合，任务未完成（零成功，与无结果区分）",
                },
            ],
        )

    def step(self, dt: float) -> None:
        self._sim_time += dt
        if not self._fault_stale:
            self._last_obs_ts = time.time()

        # Policy inference smooth mock excitation
        if self._policy.infer_running:
            amp = 0.22 * self._policy.speed_scale
            for idx, j in enumerate(self.descriptor.joints):
                if j.group in ("left_arm", "right_arm") and not j.is_gripper:
                    osc = j.default + amp * math.sin(self._sim_time * 1.4 + idx * 0.7)
                    self._targets[j.name] = max(j.lower, min(j.upper, osc))
            self._policy.infer_hz = 10.0

        # Smooth joint tracking (mock simulation servo speed)
        max_rate = 2.8  # rad/s
        for j in self.descriptor.joints:
            cur = self._positions[j.name]
            tgt = self._targets[j.name]
            err = tgt - cur
            step_lim = max_rate * dt
            delta = max(-step_lim, min(step_lim, err))
            if abs(err) < 1e-4:
                delta = err
            nxt = max(j.lower, min(j.upper, cur + delta))
            self._velocities[j.name] = round((nxt - cur) / max(dt, 1e-4), 4)
            self._positions[j.name] = round(nxt, 5)
            # Light gravity/motion effort estimation
            self._efforts[j.name] = round(1.8 * math.sin(nxt) + 0.6 * self._velocities[j.name], 3)
            # Gentle thermal drift if not faulted
            if self._temps[j.name] < 72.0:
                self._temps[j.name] = round(34.5 + abs(self._efforts[j.name]) * 2.1 + 0.4 * math.sin(self._sim_time * 0.3), 1)

        # Navigation motion update (holonomic omnidirectional translation + yaw alignment)
        if self._nav.supported:
            if self._nav.localization_spin_active:
                step_yaw = 0.7 * dt
                self._nav.pose_map["yaw"] = round(self._nav.pose_map["yaw"] + step_yaw, 4)
                self._spin_remaining_rad -= step_yaw
                if self._spin_remaining_rad <= 0.0:
                    self._nav.localization_spin_active = False
            elif self._nav.mode == "NAVIGATION" and self._nav.goal_map:
                px, py, pyaw = self._nav.pose_map["x"], self._nav.pose_map["y"], self._nav.pose_map["yaw"]
                gx, gy, gyaw = self._nav.goal_map["x"], self._nav.goal_map["y"], self._nav.goal_map["yaw"]
                dx_g, dy_g = gx - px, gy - py
                dist_g = math.hypot(dx_g, dy_g)
                dyaw_g = math.atan2(math.sin(gyaw - pyaw), math.cos(gyaw - pyaw))
                pos_done = dist_g <= 0.04 and len(self._nav.path_points) <= 2
                yaw_done = abs(dyaw_g) <= 0.05
                if pos_done and yaw_done:
                    self._nav.pose_map = {"x": round(gx, 4), "y": round(gy, 4), "yaw": round(gyaw, 4)}
                    self._nav.goal_map = None
                    self._nav.path_points = []
                    self._nav.path_source = ""
                    self._nav.mode = "MANUAL"
                else:
                    if not pos_done:
                        # Follow multi-waypoint A* corridor polyline when provided (never cuts through walls!)
                        remaining_step = max(1.6, min(3.6, max(dist_g, 0.8) * 3.8)) * dt
                        cx, cy = px, py
                        move_dx, move_dy = 0.0, 0.0
                        if self._nav.path_points and len(self._nav.path_points) >= 2:
                            pts = self._nav.path_points
                            while remaining_step > 1e-4 and len(pts) >= 2:
                                tx, ty = float(pts[1][0]), float(pts[1][1])
                                seg_dx, seg_dy = tx - cx, ty - cy
                                seg_d = math.hypot(seg_dx, seg_dy)
                                if seg_d <= 1e-4:
                                    pts.pop(0)
                                    continue
                                if seg_d <= remaining_step:
                                    cx, cy = tx, ty
                                    move_dx, move_dy = seg_dx, seg_dy
                                    remaining_step -= seg_d
                                    pts.pop(0)
                                else:
                                    ratio = remaining_step / seg_d
                                    cx += seg_dx * ratio
                                    cy += seg_dy * ratio
                                    move_dx, move_dy = seg_dx, seg_dy
                                    pts[0] = [round(cx, 3), round(cy, 3)]
                                    remaining_step = 0.0
                            self._nav.pose_map["x"] = round(cx, 4)
                            self._nav.pose_map["y"] = round(cy, 4)
                            if math.hypot(move_dx, move_dy) > 1e-3 and len(pts) > 2:
                                desired_yaw = math.atan2(move_dy, move_dx)
                                d_head = math.atan2(math.sin(desired_yaw - pyaw), math.cos(desired_yaw - pyaw))
                                self._nav.pose_map["yaw"] = round(pyaw + max(-5.0 * dt, min(5.0 * dt, d_head)), 4)
                            else:
                                step_y = max(-4.8 * dt, min(4.8 * dt, dyaw_g))
                                self._nav.pose_map["yaw"] = round(pyaw + step_y, 4)
                        else:
                            step_d = min(dist_g, remaining_step)
                            self._nav.pose_map["x"] = round(px + (dx_g / max(dist_g, 1e-6)) * step_d, 4)
                            self._nav.pose_map["y"] = round(py + (dy_g / max(dist_g, 1e-6)) * step_d, 4)
                            step_y = max(-4.8 * dt, min(4.8 * dt, dyaw_g))
                            self._nav.pose_map["yaw"] = round(pyaw + step_y, 4)
                    else:
                        self._nav.pose_map["x"] = round(gx, 4)
                        self._nav.pose_map["y"] = round(gy, 4)
                        if not yaw_done:
                            step_y = max(-4.8 * dt, min(4.8 * dt, dyaw_g))
                            self._nav.pose_map["yaw"] = round(pyaw + step_y, 4)
                        else:
                            self._nav.pose_map["yaw"] = round(gyaw, 4)
            elif self._nav.mode == "MANUAL" and self._sim_time < self._chassis_cmd_until:
                vx = self._nav.chassis_vel["vx"]
                vy = self._nav.chassis_vel["vy"]
                om = self._nav.chassis_vel["omega"]
                yaw = self._nav.pose_map["yaw"]
                self._nav.pose_map["x"] = round(self._nav.pose_map["x"] + (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt, 4)
                self._nav.pose_map["y"] = round(self._nav.pose_map["y"] + (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt, 4)
                self._nav.pose_map["yaw"] = round(yaw + om * dt, 4)
            elif self._sim_time >= self._chassis_cmd_until:
                self._nav.chassis_vel = {"vx": 0.0, "vy": 0.0, "omega": 0.0}

        # FoundationPose registration phase progression
        if self._perc.foundation_pose_running and self._perc.foundation_pose_phase in ("waiting", "registering"):
            self._fp_register_elapsed += dt
            if self._fp_register_elapsed >= 0.8:
                self._perc.foundation_pose_phase = "tracking"
                self._perc.foundation_pose_result = {
                    "frame": "base_link",
                    "object": self._perc.yolo_prompt,
                    "mesh": self._perc.foundation_pose_mesh,
                    "xyz_m": [0.684, -0.092, 0.812],
                    "rpy_deg": [1.4, -2.1, -88.5],
                    "confidence": 0.94,
                    "updated_at": utc_iso(),
                }
                self._log_perc("FoundationPose 首次注册完成，已进入 tracking 状态（可安全停止 YOLO）。")

        # Update tracking pose gently
        if self._perc.foundation_pose_running and self._perc.foundation_pose_phase == "tracking" and self._perc.foundation_pose_result:
            self._perc.foundation_pose_result["xyz_m"] = [
                round(0.684 + 0.003 * math.sin(self._sim_time * 1.2), 4),
                round(-0.092 + 0.002 * math.cos(self._sim_time * 1.1), 4),
                0.812,
            ]
            self._perc.foundation_pose_result["updated_at"] = utc_iso()

    def _log_perc(self, msg: str) -> None:
        self._perc.logs.append(f"[{utc_iso()[11:19]}] {msg}")
        if len(self._perc.logs) > 40:
            self._perc.logs = self._perc.logs[-40:]

    def _compute_ee_poses(self) -> dict[str, EEPose]:
        poses: dict[str, EEPose] = {}
        if not self.descriptor.capabilities.ee_6d:
            return poses
        if self._kin is not None:
            for side in self._kin.arm_cfg.keys():
                fk = self._kin.fk(side, self._positions)
                poses[side] = EEPose(
                    arm=side,
                    frame=fk["base_frame"],
                    x=round(fk["position"][0], 4),
                    y=round(fk["position"][1], 4),
                    z=round(fk["position"][2], 4),
                    roll_deg=round(fk["rpy_deg"][0], 2),
                    pitch_deg=round(fk["rpy_deg"][1], 2),
                    yaw_deg=round(fk["rpy_deg"][2], 2),
                )
            return poses
        for side in (self.descriptor.arms_scope or ["left", "right"]):
            base = self._ee_offsets.get(side, [0.42, 0.0, 0.90, 0.0, 0.0, 0.0])
            poses[side] = EEPose(
                arm=side,
                frame="base_link",
                x=round(base[0], 4),
                y=round(base[1], 4),
                z=round(base[2], 4),
                roll_deg=round(base[3], 2),
                pitch_deg=round(base[4], 2),
                yaw_deg=round(base[5], 2),
            )
        return poses

    def get_observation(self) -> ObservationSnapshot:
        age_ms = 2450.0 if self._fault_stale else round(max(6.0, (time.time() - self._last_obs_ts) * 1000.0 + 12.0), 1)
        motors: list[MotorDiag] = []
        interlocks: list[InterlockItem] = []

        max_temp = 0.0
        any_err = False
        for j in self.descriptor.joints:
            temp = self._temps[j.name]
            err = self._errors[j.name]
            max_temp = max(max_temp, temp)
            sev = "ok"
            if err != 0 or temp >= 75.0:
                sev = "block"
                any_err = any_err or (err != 0)
            elif temp >= 60.0:
                sev = "warn"
            motors.append(
                MotorDiag(
                    joint=j.name,
                    enabled=(err == 0),
                    temperature_c=temp,
                    effort_nm=self._efforts[j.name],
                    error_code=err,
                    statusword="0x0608" if err != 0 else "0x0637",
                    severity=sev,  # type: ignore[arg-type]
                )
            )

        interlocks.append(
            InterlockItem(
                code="feedback_freshness",
                label="观测反馈新鲜度",
                severity="block" if age_ms > 1000.0 else "ok",
                detail=f"{age_ms:.0f} ms (阈值 1000 ms)",
            )
        )
        interlocks.append(
            InterlockItem(
                code="thermal_guard",
                label="电机温度联锁",
                severity="block" if max_temp >= 75.0 else ("warn" if max_temp >= 60.0 else "ok"),
                detail=f"最高 {max_temp:.1f} °C (Warn 60°C / Block 75°C)",
            )
        )
        interlocks.append(
            InterlockItem(
                code="motor_fault",
                label="电机错误码",
                severity="block" if any_err else "ok",
                detail="检测到故障码 0x2310，已触发联锁阻断" if any_err else "全部电机 0x0000 正常",
            )
        )
        interlocks.append(
            InterlockItem(
                code="single_writer",
                label="底层唯一写者校验",
                severity="ok",
                detail="单写者通道已锁定（Robot Pipeline 独占底层写权）",
            )
        )

        topics = [
            {"topic": f"/edge/{self.descriptor.id}/observation", "hz": 0.0 if self._fault_stale else 30.0, "status": "stale" if self._fault_stale else "ok"},
            {"topic": f"/edge/{self.descriptor.id}/joint_states", "hz": 0.0 if self._fault_stale else 30.0, "status": "stale" if self._fault_stale else "ok"},
        ]
        for cam in self.descriptor.cameras:
            topics.append({"topic": f"/camera/{cam.key}/compressed", "hz": cam.fps_target, "status": "ok"})
        if self.descriptor.capabilities.navigation:
            topics.append({"topic": "/navigation/status", "hz": 5.0, "status": "ok"})

        return ObservationSnapshot(
            robot_id=self.descriptor.id,
            timestamp=time.time(),
            connected=not self._fault_stale,
            mode=self.descriptor.mode,
            feedback_age_ms=age_ms,
            measured_hz=0.0 if self._fault_stale else 30.0,
            joint_positions=dict(self._positions),
            joint_targets=dict(self._targets),
            joint_velocities=dict(self._velocities),
            joint_efforts=dict(self._efforts),
            ee_poses=self._compute_ee_poses(),
            motors=motors,
            interlocks=interlocks,
            battery=BatteryState(percentage=round(89.0 - (self._sim_time % 120) * 0.02, 1), voltage_v=48.4, current_a=-2.6, source="mock_bms"),
            navigation=self._nav.model_copy(deep=True),
            perception=self._perc.model_copy(deep=True),
            policy=self._policy.model_copy(deep=True),
            topics=topics,
        )

    def get_camera_svg(self, stream_key: str) -> str:
        t = self._sim_time
        ts = utc_iso()[11:19]
        cx = int(220 + 18 * math.sin(t * 0.9))
        cy = int(125 + 10 * math.cos(t * 0.7))
        overlays: list[str] = []

        # Depth heatmap tint when head_depth enabled
        if self._perc.head_depth_enabled and "head" in stream_key:
            overlays.append(
                '<rect x="0" y="0" width="480" height="270" fill="url(#depthGrad)" opacity="0.28"/>'
                '<text x="14" y="252" fill="#2dd4bf" font-size="11" font-family="monospace">STEREO DEPTH READY · 32FC1 · 14.8 FPS</text>'
            )

        # YOLO bbox + mask overlay
        if self._perc.yolo_running and self._perc.yolo_camera == stream_key:
            overlays.append(
                f'<rect x="{cx - 42}" y="{cy - 54}" width="84" height="108" rx="4" fill="rgba(13,148,136,0.18)" stroke="#14b8a6" stroke-width="2"/>'
                f'<rect x="{cx - 42}" y="{cy - 72}" width="118" height="18" rx="3" fill="#0d9488"/>'
                f'<text x="{cx - 36}" y="{cy - 59}" fill="#ffffff" font-size="11" font-family="monospace">{html.escape(str(self._perc.yolo_prompt), quote=True)} 0.91 [g{self._perc.yolo_generation}]</text>'
            )

        # Latched mask indicator
        if self._perc.latched_mask and self._perc.latched_mask.get("camera") == stream_key:
            overlays.append(
                f'<ellipse cx="{cx}" cy="{cy}" rx="28" ry="44" fill="rgba(56,189,248,0.28)" stroke="#38bdf8" stroke-width="1.5" stroke-dasharray="4 3"/>'
                '<text x="302" y="24" fill="#38bdf8" font-size="11" font-family="monospace">MASK LATCHED</text>'
            )

        # FoundationPose 6D axes overlay
        if self._perc.foundation_pose_running and self._perc.foundation_pose_camera == stream_key:
            overlays.append(
                f'<line x1="{cx}" y1="{cy}" x2="{cx + 52}" y2="{cy - 8}" stroke="#ef4444" stroke-width="2.5"/>'
                f'<line x1="{cx}" y1="{cy}" x2="{cx - 12}" y2="{cy - 48}" stroke="#22c55e" stroke-width="2.5"/>'
                f'<line x1="{cx}" y1="{cy}" x2="{cx - 32}" y2="{cy + 28}" stroke="#3b82f6" stroke-width="2.5"/>'
                f'<text x="14" y="42" fill="#facc15" font-size="11" font-family="monospace">FoundationPose: {html.escape(str(self._perc.foundation_pose_phase).upper(), quote=True)}</text>'
            )

        # ArUco marker overlay
        if self._perc.aruco_pose_running:
            overlays.append(
                '<rect x="332" y="138" width="54" height="54" fill="#ffffff" stroke="#f59e0b" stroke-width="2"/>'
                '<rect x="342" y="148" width="14" height="14" fill="#0f172a"/>'
                '<rect x="362" y="148" width="14" height="14" fill="#0f172a"/>'
                '<rect x="342" y="168" width="34" height="14" fill="#0f172a"/>'
                '<text x="330" y="132" fill="#f59e0b" font-size="11" font-family="monospace">ArUco #1 · 0.62m</text>'
            )

        # Picked 3D crosshair
        if self._perc.last_3d_pick and self._perc.last_3d_pick.get("stream_key") == stream_key:
            pu = int(self._perc.last_3d_pick.get("u", 240))
            pv = int(self._perc.last_3d_pick.get("v", 135))
            overlays.append(
                f'<circle cx="{pu}" cy="{pv}" r="7" fill="none" stroke="#f43f5e" stroke-width="2"/>'
                f'<line x1="{pu - 12}" y1="{pv}" x2="{pu + 12}" y2="{pv}" stroke="#f43f5e" stroke-width="1.5"/>'
                f'<line x1="{pu}" y1="{pv - 12}" x2="{pu}" y2="{pv + 12}" stroke="#f43f5e" stroke-width="1.5"/>'
            )

        return (
            '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 480 270" width="480" height="270">'
            "<defs>"
            '<linearGradient id="bg" x1="0" y1="0" x2="0" y2="1">'
            '<stop offset="0%" stop-color="#0f172a"/><stop offset="100%" stop-color="#1e293b"/>'
            "</linearGradient>"
            '<linearGradient id="depthGrad" x1="0" y1="0" x2="1" y2="1">'
            '<stop offset="0%" stop-color="#0ea5e9"/><stop offset="50%" stop-color="#14b8a6"/><stop offset="100%" stop-color="#f59e0b"/>'
            "</linearGradient>"
            "</defs>"
            '<rect width="480" height="270" fill="url(#bg)"/>'
            '<path d="M0 190 L480 190 M0 230 L480 230 M120 190 L40 270 M240 190 L240 270 M360 190 L440 270" stroke="#334155" stroke-width="1"/>'
            f'<rect x="{cx - 24}" y="{cy - 38}" width="48" height="76" rx="10" fill="#475569" stroke="#94a3b8" stroke-width="1.5"/>'
            f'<rect x="{cx - 10}" y="{cy - 48}" width="20" height="12" rx="3" fill="#cbd5e1"/>'
            '<rect x="52" y="150" width="110" height="58" rx="6" fill="#1d4ed8" fill-opacity="0.35" stroke="#60a5fa" stroke-width="1.5"/>'
            '<text x="64" y="183" fill="#93c5fd" font-size="11" font-family="monospace">WORK TRAY</text>'
            + "".join(overlays)
            + f'<text x="14" y="22" fill="#e2e8f0" font-size="12" font-family="monospace">{html.escape(str(self.descriptor.id).replace("-mock", ""), quote=True)} · {html.escape(str(stream_key), quote=True)}</text>'
            + f'<text x="394" y="254" fill="#94a3b8" font-size="11" font-family="monospace">{ts} LIVE</text>'
            + "</svg>"
        )

    # ---- Commands ----
    def send_joint_targets(self, targets: dict[str, float], duration_s: float = 1.2) -> dict[str, Any]:
        for name, val in targets.items():
            self._targets[name] = val
        return {"accepted": True, "targets_count": len(targets), "duration_s": duration_s}

    def send_ee_step(
        self, arm: str, dx: float, dy: float, dz: float, droll_deg: float, dpitch_deg: float, dyaw_deg: float
    ) -> dict[str, Any]:
        if not self.descriptor.capabilities.ee_6d:
            raise SafetyError(400, "ee_unsupported", f"{self.descriptor.name} 未声明末端 6D 控制能力。")
        if self._kin is not None:
            cur_fk = self._kin.fk(arm, self._targets)
            tgt_xyz = [
                cur_fk["position"][0] + dx,
                cur_fk["position"][1] + dy,
                cur_fk["position"][2] + dz,
            ]
            tgt_rpy = [
                cur_fk["rpy_deg"][0] + droll_deg,
                cur_fk["rpy_deg"][1] + dpitch_deg,
                cur_fk["rpy_deg"][2] + dyaw_deg,
            ]
            ik = self._kin.solve_ik(arm, tgt_xyz, tgt_rpy, self._targets)
            for jn, val in ik["solved_arm_joints"].items():
                self._targets[jn] = val
            return {"accepted": True, "arm": arm, "ik": ik, "ee_pose": self._compute_ee_poses().get(arm)}
        base = self._ee_offsets[arm]
        base[0] = round(max(0.15, min(0.85, base[0] + dx)), 4)
        base[1] = round(max(-0.65, min(0.65, base[1] + dy)), 4)
        base[2] = round(max(0.45, min(1.45, base[2] + dz)), 4)
        base[3] = round(base[3] + droll_deg, 2)
        base[4] = round(base[4] + dpitch_deg, 2)
        base[5] = round(base[5] + dyaw_deg, 2)
        return {"accepted": True, "arm": arm, "ee_pose": self._compute_ee_poses().get(arm)}

    def send_chassis_velocity(self, vx: float, vy: float, omega: float, duration_s: float = 0.4) -> dict[str, Any]:
        if not self.descriptor.capabilities.chassis_teleop:
            raise SafetyError(400, "chassis_unsupported", f"{self.descriptor.name} 不支持底盘速度控制。")
        if self._nav.mode != "MANUAL":
            raise SafetyError(409, "nav_mode_conflict", f"当前底盘模式为 {self._nav.mode}，仅 MANUAL 模式下允许手动底盘速度控制。")
        self._nav.chassis_vel = {
            "vx": max(-0.6, min(0.6, vx)),
            "vy": max(-0.4, min(0.4, vy)),
            "omega": max(-0.8, min(0.8, omega)),
        }
        self._chassis_cmd_until = self._sim_time + max(0.1, min(3.0, duration_s))
        return {"accepted": True, "chassis_vel": self._nav.chassis_vel}

    def set_navigation_mode(self, mode: str) -> dict[str, Any]:
        if not self.descriptor.capabilities.navigation:
            raise SafetyError(400, "navigation_unsupported", f"{self.descriptor.name} 无移动底盘导航能力。")
        m = mode.upper()
        if m not in ("MANUAL", "NAVIGATION"):
            raise SafetyError(400, "invalid_nav_mode", f"不支持的目标导航模式：{mode}")
        self._nav.chassis_vel = {"vx": 0.0, "vy": 0.0, "omega": 0.0}
        if m == "MANUAL":
            self._nav.goal_map = None
            self._nav.path_points = []
            self._nav.path_source = ""
        self._nav.mode = m  # type: ignore[assignment]
        return {"accepted": True, "mode": self._nav.mode}

    def send_navigation_goal(
        self, x: float, y: float, yaw: float, path_points: list[list[float]] | None = None
    ) -> dict[str, Any]:
        if not self.descriptor.capabilities.navigation:
            raise SafetyError(400, "navigation_unsupported", f"{self.descriptor.name} 无移动底盘导航能力。")
        if self._nav.mode != "NAVIGATION":
            raise SafetyError(409, "nav_mode_required", "必须先将底盘控制权切换至 NAVIGATION 模式后才能发送导航目标。")
        if not self._nav.navigation_ready:
            raise SafetyError(409, "nav_not_ready", "定位尚未就绪，拒绝下发导航目标。")
        self._nav.goal_map = {"x": round(x, 3), "y": round(y, 3), "yaw": round(yaw, 4)}
        px, py = self._nav.pose_map["x"], self._nav.pose_map["y"]

        # Accept validated collision-free A* polyline when provided by planner
        sanitized: list[list[float]] = []
        if isinstance(path_points, list) and len(path_points) >= 2:
            for item in path_points[:256]:
                if isinstance(item, (list, tuple)) and len(item) >= 2:
                    fx, fy = float(item[0]), float(item[1])
                    if math.isfinite(fx) and math.isfinite(fy):
                        sanitized.append([round(fx, 3), round(fy, 3)])
        if len(sanitized) >= 2:
            sanitized[0] = [round(px, 3), round(py, 3)]
            sanitized[-1] = [round(x, 3), round(y, 3)]
            self._nav.path_points = sanitized
            self._nav.path_source = "local_astar"
        else:
            pts: list[list[float]] = []
            for i in range(9):
                t = i / 8.0
                mx = px + (x - px) * t
                my = py + (y - py) * t + 0.18 * math.sin(math.pi * t)
                pts.append([round(mx, 3), round(my, 3)])
            self._nav.path_points = pts
            self._nav.path_source = "mock_direct"
        return {
            "accepted": True,
            "goal": self._nav.goal_map,
            "path_points": self._nav.path_points,
            "path_source": self._nav.path_source,
        }

    def cancel_navigation(self) -> dict[str, Any]:
        self._nav.goal_map = None
        self._nav.path_points = []
        self._nav.path_source = ""
        return {"accepted": True, "mode": self._nav.mode}

    def start_localization_spin(self, active: bool) -> dict[str, Any]:
        if not self.descriptor.capabilities.navigation:
            raise SafetyError(400, "navigation_unsupported", f"{self.descriptor.name} 无移动底盘导航能力。")
        self._nav.chassis_vel = {"vx": 0.0, "vy": 0.0, "omega": 0.0}
        self._nav.localization_spin_active = active
        self._spin_remaining_rad = 2 * math.pi if active else 0.0
        return {"accepted": True, "localization_spin_active": active}

    def control_perception(self, action: str, payload: dict[str, Any]) -> dict[str, Any]:
        if action == "set_head_depth":
            if not self.descriptor.capabilities.head_depth:
                raise SafetyError(400, "head_depth_unsupported", "当前机器人无头部在线双目深度处理能力。")
            enabled = bool(payload.get("enabled"))
            self._perc.head_depth_enabled = enabled
            self._perc.head_depth_ready = enabled
            self._perc.head_depth_fps = 14.8 if enabled else 0.0
            if not enabled:
                # Turning off head depth invalidates head_left detections/latch
                if self._perc.yolo_camera == "head_left_rgb":
                    self._perc.yolo_running = False
                    self._perc.latched_mask = None
                    self._perc.foundation_pose_running = False
                    self._perc.foundation_pose_phase = "idle"
            self._log_perc(f"头部在线深度处理已{'开启 (READY 14.8 FPS)' if enabled else '关闭'}。")
            return {"ok": True, "perception": self._perc.model_dump()}

        if action == "pick_3d":
            stream = str(payload.get("stream_key") or self.descriptor.cameras[0].key)
            u = float(payload.get("u", 240.0))
            v = float(payload.get("v", 135.0))
            if stream == "head_left_rgb" and self.descriptor.capabilities.head_depth and not self._perc.head_depth_ready:
                raise SafetyError(409, "head_depth_not_ready", "头部左相机取 3D 点必须先开启头部在线深度处理并等待 READY。")
            depth = round(0.72 + (u - 240.0) * 0.0005, 3)
            cam_x = round((u - 240.0) * 0.0014 * depth, 4)
            cam_y = round((v - 135.0) * 0.0014 * depth, 4)
            base_xyz = [round(depth, 4), round(-cam_x, 4), round(0.95 - cam_y, 4)]
            self._perc.last_3d_pick = {
                "stream_key": stream,
                "u": round(u, 1),
                "v": round(v, 1),
                "depth_m": depth,
                "camera_xyz_m": [cam_x, cam_y, depth],
                "base_link_xyz_m": base_xyz,
                "timestamp": utc_iso(),
            }
            self._log_perc(f"3D 取点 ({stream}) u={u:.0f},v={v:.0f} → base_link={base_xyz}")
            return {"ok": True, "pick": self._perc.last_3d_pick}

        if action == "start_yolo":
            if not self.descriptor.capabilities.yolo_foundation_pose:
                raise SafetyError(400, "perception_unsupported", "当前机器人未配置 YOLO/FoundationPose 识别管线。")
            cam = str(payload.get("camera") or "head_left_rgb")
            prompt = str(payload.get("prompt") or "bottle").strip() or "bottle"
            conf = float(payload.get("conf", 0.4))
            if cam == "head_left_rgb" and self.descriptor.capabilities.head_depth and not self._perc.head_depth_ready:
                raise SafetyError(409, "head_depth_not_ready", "门禁拦截：使用 head_left 启动 YOLO 前，必须先开启「头部在线深度处理」并进入 READY。")
            self._perc.yolo_generation += 1
            self._perc.yolo_running = True
            self._perc.yolo_camera = cam
            self._perc.yolo_prompt = prompt
            self._perc.yolo_conf = conf
            # Config/generation change invalidates old latch!
            self._perc.latched_mask = None
            self._perc.yolo_detections = [
                {
                    "class_name": prompt,
                    "confidence": 0.91,
                    "bbox_xyxy": [178, 71, 262, 179],
                    "area_px": 4120,
                    "generation": self._perc.yolo_generation,
                }
            ]
            self._log_perc(f"步骤① YOLO 已启动 (cam={cam}, prompt={prompt}, gen={self._perc.yolo_generation})；旧锁存已清空。")
            return {"ok": True, "perception": self._perc.model_dump()}

        if action == "stop_yolo":
            if self._perc.foundation_pose_running and self._perc.foundation_pose_phase in ("waiting", "registering"):
                raise SafetyError(
                    409,
                    "yolo_locked_during_registration",
                    "门禁拦截：FoundationPose 正在 waiting/registering 注册阶段，出现首个 object_pose 前禁止停止 YOLO。",
                )
            self._perc.yolo_running = False
            self._log_perc("YOLO 已停止。")
            return {"ok": True, "perception": self._perc.model_dump()}

        if action == "latch_yolo":
            if not self._perc.yolo_running or not self._perc.yolo_detections:
                raise SafetyError(409, "yolo_not_running", "门禁拦截：必须先启动 YOLO 并获得有效检测结果后，才能锁存最佳分割。")
            det = self._perc.yolo_detections[0]
            self._perc.latched_mask = {
                "generation": self._perc.yolo_generation,
                "camera": self._perc.yolo_camera,
                "class_name": det["class_name"],
                "area_px": det["area_px"],
                "mask_file": f"/tmp/r1pro_web/latched_mask_gen{self._perc.yolo_generation}_{det['class_name']}.png",
                "verified": True,
                "timestamp": utc_iso(),
            }
            self._log_perc(f"步骤② 已锁存最佳分割：{self._perc.latched_mask['mask_file']} (gen={self._perc.yolo_generation})")
            return {"ok": True, "perception": self._perc.model_dump()}

        if action == "start_foundation_pose":
            if not self.descriptor.capabilities.yolo_foundation_pose:
                raise SafetyError(400, "perception_unsupported", "当前机器人未配置 FoundationPose。")
            cam = str(payload.get("camera") or self._perc.yolo_camera)
            mesh = str(payload.get("mesh") or self._perc.foundation_pose_mesh)
            classes = [c.strip() for c in str(payload.get("classes") or self._perc.yolo_prompt).split(",") if c.strip()]
            # Strict 3-step gate check!
            if not self._perc.yolo_running:
                raise SafetyError(409, "gate_yolo_required", "门禁拦截（步骤①未满足）：启动 FoundationPose 时 YOLO 必须处于运行状态。")
            if not self._perc.latched_mask:
                raise SafetyError(409, "gate_latch_required", "门禁拦截（步骤②未满足）：请先执行「锁存最佳分割」，禁止跳过锁存直接启动 FoundationPose。")
            if self._perc.latched_mask.get("generation") != self._perc.yolo_generation:
                raise SafetyError(409, "gate_stale_latch", "门禁拦截：YOLO 代次已变化，旧锁存已失效，请重新锁存。")
            if self._perc.latched_mask.get("camera") != cam:
                raise SafetyError(409, "gate_camera_mismatch", f"门禁拦截：FoundationPose 相机 ({cam}) 必须与锁存目标相机 ({self._perc.latched_mask.get('camera')}) 一致。")
            if self._perc.latched_mask.get("class_name") not in classes:
                raise SafetyError(409, "gate_class_mismatch", f"门禁拦截：锁存类别 ({self._perc.latched_mask.get('class_name')}) 不在 FoundationPose Classes 中。")

            self._perc.foundation_pose_running = True
            self._perc.foundation_pose_phase = "registering"
            self._perc.foundation_pose_camera = cam
            self._perc.foundation_pose_mesh = mesh
            self._fp_register_elapsed = 0.0
            self._log_perc(f"步骤③ FoundationPose 启动中 (registering, mesh={mesh})，注册完成前锁定停止 YOLO。")
            return {"ok": True, "perception": self._perc.model_dump()}

        if action == "stop_foundation_pose":
            self._perc.foundation_pose_running = False
            self._perc.foundation_pose_phase = "idle"
            self._perc.foundation_pose_result = None
            self._log_perc("FoundationPose 已停止。")
            return {"ok": True, "perception": self._perc.model_dump()}

        if action == "aruco_start_pose":
            if not self.descriptor.capabilities.aruco:
                raise SafetyError(400, "aruco_unsupported", "当前机器人未配置 ArUco 检测。")
            self._perc.aruco_pose_running = True
            self._perc.aruco_markers = [
                {"id": 1, "frame": "base_link", "x_m": 0.624, "y_m": 0.014, "z_m": 0.742, "yaw_deg": -89.2}
            ]
            self._log_perc("ArUco 位姿检测已启动，检测到 Marker #1 (x=0.624m, yaw=-89.2°)。")
            return {"ok": True, "perception": self._perc.model_dump()}

        if action == "aruco_stop_pose":
            self._perc.aruco_pose_running = False
            self._perc.aruco_fine_enabled = False
            self._perc.aruco_markers = []
            self._log_perc("ArUco 位姿检测及微调已停止。")
            return {"ok": True, "perception": self._perc.model_dump()}

        if action == "aruco_set_fine":
            if not self._perc.aruco_pose_running:
                raise SafetyError(409, "aruco_pose_required", "请先启动 ArUco 位姿识别后再开启底盘离散微调。")
            en = bool(payload.get("enabled"))
            self._perc.aruco_fine_enabled = en
            if en and self._perc.aruco_markers:
                self._perc.aruco_markers[0]["x_m"] = 0.601
                self._perc.aruco_markers[0]["y_m"] = 0.002
                self._perc.aruco_markers[0]["yaw_deg"] = -90.0
            self._log_perc(f"ArUco 底盘离散微调已{'开启并完成对准' if en else '关闭'}。")
            return {"ok": True, "perception": self._perc.model_dump()}

        raise SafetyError(400, "unknown_perception_action", f"未知感知动作：{action}")

    def control_policy(self, action: str, payload: dict[str, Any]) -> dict[str, Any]:
        if not self.descriptor.capabilities.policy_inference:
            raise SafetyError(400, "policy_unsupported", f"{self.descriptor.name} 未声明策略推理能力。")

        if action == "switch_checkpoint":
            ckpt_id = str(payload.get("checkpoint_id") or "")
            found = next((c for c in self._policy.available_checkpoints if c.id == ckpt_id), None)
            if not found:
                raise SafetyError(404, "checkpoint_not_found", f"未找到检查点：{ckpt_id}")
            if not found.compatible or found.action_dim != len(self.descriptor.joints):
                raise SafetyError(
                    400,
                    "action_layout_mismatch",
                    f"Action Layout 契约拦截：检查点维度 {found.action_dim} 与当前机器人 {self.descriptor.name} ({len(self.descriptor.joints)}-DOF) 不匹配。",
                )
            if self._policy.infer_running:
                raise SafetyError(409, "infer_running", "请先停止当前推理会话再动态切换检查点。")
            for key in self._policy.wizard_steps:
                self._policy.wizard_steps[key] = key == "self_check"
            self._policy.last_offline_eval = None
            self._policy.active_checkpoint = found.id
            self._policy.rtc_enabled = found.rtc_supported
            return {"ok": True, "active_checkpoint": found.id, "rtc_enabled": found.rtc_supported}

        if action == "set_config":
            if self._policy.infer_running:
                raise SafetyError(409, "infer_running", "先停止推理再更改配置。")
            self._policy.wizard_steps["live_armed"] = False
            self._policy.wizard_steps["offline_replay"] = False
            self._policy.last_offline_eval = None
            if "task_intent" in payload:
                self._policy.task_intent = str(payload["task_intent"])
            if "execution_schema" in payload:
                self._policy.execution_schema = str(payload["execution_schema"])
            if "speed_scale" in payload:
                value = float(payload["speed_scale"])
                if not math.isfinite(value):
                    raise SafetyError(400, "invalid_number", "速度比例必须为有限数值。")
                self._policy.speed_scale = max(0.1, min(1.0, value))
            return {"ok": True, "policy": self._policy.model_dump()}

        if action == "wizard_step":
            step_key = str(payload.get("step") or "")
            if step_key not in self._policy.wizard_steps:
                raise SafetyError(400, "invalid_wizard_step", f"未知向导步骤：{step_key}")
            val = bool(payload.get("done", True))
            order = list(self._policy.wizard_steps)
            index = order.index(step_key)
            if val and not all(self._policy.wizard_steps[k] for k in order[:index]):
                raise SafetyError(409, "wizard_prerequisite", "必须按顺序完成前置安全放行步骤，禁止跳步。")
            if val and step_key == "offline_replay":
                raise SafetyError(409, "evaluation_required", "离线空跑步骤必须由离线对比评测自动完成，不接受手动勾选。")
            self._policy.wizard_steps[step_key] = val
            for key in order[index + 1:]:
                self._policy.wizard_steps[key] = False
            self._policy.infer_running = False
            self._policy.infer_hz = 0.0
            return {"ok": True, "wizard_steps": dict(self._policy.wizard_steps)}

        if action == "offline_eval":
            if self.descriptor.capabilities.piper_safety_wizard and not all(
                self._policy.wizard_steps[k] for k in ("self_check", "homing", "manual_slow_test")
            ):
                raise SafetyError(409, "wizard_prerequisite", "请先依次完成自检、回零校准和低速行程确认步骤。")
            dataset = str(payload.get("dataset") or ("set_the_plates_real_all_mcap_20261119_v1" if self.descriptor.robot_type == "piper_dual" else "r1pro_basket_mcap_sample"))
            steps = list(range(16))
            gt_j1 = [round(0.2 + 0.35 * math.sin(i * 0.35), 4) for i in steps]
            pred_j1 = [round(v + 0.022 * math.cos(i * 0.6), 4) for i, v in enumerate(gt_j1)]
            gt_j2 = [round(0.55 - 0.28 * math.cos(i * 0.3), 4) for i in steps]
            pred_j2 = [round(v - 0.018 * math.sin(i * 0.5), 4) for i, v in enumerate(gt_j2)]
            rmse = round(math.sqrt(sum((a - b) ** 2 for a, b in zip(gt_j1, pred_j1)) / len(steps)), 4)
            res = {
                "data_source": "synthetic_mock",
                "dataset_parsed": False,
                "qualified_for_hardware": False,
                "disclaimer": "生成的 Mock 演示曲线；未读取 MCAP/LeRobot，不是真实模型评测或硬件放行证据。",
                "eval_id": f"eval-{uuid.uuid4().hex[:8]}",
                "timestamp": utc_iso(),
                "dataset": dataset,
                "checkpoint": self._policy.active_checkpoint,
                "horizon_steps": len(steps),
                "rmse_rad": rmse,
                "max_jump_rad": 0.041,
                "track_error_hold_trips": 0,
                "safety_contract": "jump_ref=target · track_error_on=command · rtc=overlap(0.7)",
                "provenance": f"model={self._policy.active_checkpoint} -> postprocessor=QUANTILES -> action_dim={len(self.descriptor.joints)}",
                "series": {
                    "steps": steps,
                    "gt_primary": gt_j1,
                    "pred_primary": pred_j1,
                    "gt_secondary": gt_j2,
                    "pred_secondary": pred_j2,
                },
            }
            self._policy.last_offline_eval = res
            self._policy.wizard_steps["offline_replay"] = True
            self._policy.wizard_steps["live_armed"] = False
            return {"ok": True, "eval": res}

        if action == "start_infer":
            if self.descriptor.capabilities.piper_safety_wizard and not all(self._policy.wizard_steps.values()):
                raise SafetyError(409, "live_arm_required", "Piper 安全门禁：必须完成 6 步武装清单并勾选「⑤ 武装 Live（推理）」后才能启动实时推理。")
            self._policy.infer_running = True
            self._policy.infer_hz = 10.0
            return {"ok": True, "infer_running": True}

        if action == "stop_infer":
            self._policy.infer_running = False
            self._policy.infer_hz = 0.0
            return {"ok": True, "infer_running": False}

        if action == "tag_episode":
            outcome = str(payload.get("outcome") or "unknown")
            if outcome not in ("passed", "failed", "zero_success", "no_result", "unknown"):
                raise SafetyError(400, "invalid_outcome", f"无效打标状态：{outcome}")
            ep = {
                "id": f"ep-{len(self._policy.recorded_episodes) + 1:03d}",
                "timestamp": utc_iso(),
                "checkpoint": self._policy.active_checkpoint,
                "frames": int(payload.get("frames") or 360),
                "outcome": outcome,
                "notes": str(payload.get("notes") or ""),
            }
            self._policy.recorded_episodes.insert(0, ep)
            return {"ok": True, "episode": ep}

        raise SafetyError(400, "unknown_policy_action", f"未知策略动作：{action}")

    def safe_stop(self, reason: str) -> None:
        # Hold current pose, zero chassis vel, halt inference/navigation/depth
        for name, pos in self._positions.items():
            self._targets[name] = pos
            self._velocities[name] = 0.0
        self._nav.chassis_vel = {"vx": 0.0, "vy": 0.0, "omega": 0.0}
        self._nav.goal_map = None
        self._nav.path_points = []
        self._nav.localization_spin_active = False
        if self._nav.supported:
            self._nav.mode = "FAULT"
        self._policy.infer_running = False
        self._policy.infer_hz = 0.0
        for key in self._policy.wizard_steps:
            self._policy.wizard_steps[key] = key == "self_check"
        self._policy.last_offline_eval = None
        self._perc.aruco_fine_enabled = False
        self._log_perc(f"SAFE STOP triggered: {reason}")

    def inject_fault(self, fault_type: str) -> dict[str, Any]:
        first_joint = self.descriptor.joints[0].name
        if fault_type == "overheat":
            self._temps[first_joint] = 79.5
            self.safe_stop("thermal_block_injected")
            return {"ok": True, "injected": "overheat", "joint": first_joint, "temp_c": 79.5}
        if fault_type == "motor_error":
            self._errors[first_joint] = 0x2310
            self.safe_stop("motor_error_injected")
            return {"ok": True, "injected": "motor_error", "joint": first_joint, "error_code": "0x2310"}
        if fault_type == "stale_feedback":
            self._fault_stale = True
            return {"ok": True, "injected": "stale_feedback"}
        if fault_type == "clear":
            self._fault_stale = False
            for j in self.descriptor.joints:
                self._temps[j.name] = 35.0
                self._errors[j.name] = 0
            if self._nav.supported and self._nav.mode == "FAULT":
                self._nav.mode = "MANUAL"
            return {"ok": True, "cleared": True}
        raise SafetyError(400, "unknown_fault", f"不支持的故障注入类型：{fault_type}")


def build_default_mock_connectors() -> dict[str, MockRobotConnector]:
    conns = [
        MockRobotConnector(_r1pro_descriptor()),
        MockRobotConnector(_piper_descriptor()),
        MockRobotConnector(_w1_descriptor()),
    ]
    return {c.descriptor.id: c for c in conns}
