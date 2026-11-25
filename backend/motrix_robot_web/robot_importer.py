from __future__ import annotations

import io
import json
import math
import re
import shutil
import struct
import time
import uuid
import xml.etree.ElementTree as ET
import zipfile
from pathlib import Path
from typing import Any

import yaml

from .connectors.piper_lelab import PiperLeLabConnector
from .kinematics import RobotKinematics, inspect_urdf_file
from .schemas import (
    CameraSpec,
    JointSpec,
    KinematicsArmSpec,
    KinematicsProfile,
    ModelProfile,
    ModelTrailLinkSpec,
    RobotCapabilities,
    RobotDescriptor,
    Ros2BridgeConfig,
)

ALLOWED_EXTENSIONS = frozenset({
    ".urdf", ".xml", ".glb", ".gltf", ".bin", ".obj", ".mtl", ".stl", ".dae",
    ".png", ".jpg", ".jpeg", ".webp", ".yaml", ".yml", ".json", ".pcd", ".txt", ".md",
})
MAX_ZIP_FILES = 400
MAX_ZIP_BYTES = 80 * 1024 * 1024  # 80 MB extracted cap
BUILTIN_ROBOT_IDS = frozenset({"r1pro-mock", "piper-dual-mock", "dex-w1-mock"})
ROBOT_ID_RE = re.compile(r"^[a-z0-9][a-z0-9_-]{2,47}$")


# ---- Minimal Binary STL Generator (Pure Python, Zero External Dependencies) ----

def _pack_triangles_stl(triangles: list[tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]]) -> bytes:
    header = b"Motphys Procedural URDF Visual Mesh".ljust(80, b"\x00")[:80]
    buf = bytearray(header)
    buf.extend(struct.pack("<I", len(triangles)))
    for v0, v1, v2 in triangles:
        ux, uy, uz = v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]
        vx, vy, vz = v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]
        nx = uy * vz - uz * vy
        ny = uz * vx - ux * vz
        nz = ux * vy - uy * vx
        mag = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
        buf.extend(
            struct.pack(
                "<12fH",
                nx / mag, ny / mag, nz / mag,
                v0[0], v0[1], v0[2],
                v1[0], v1[1], v1[2],
                v2[0], v2[1], v2[2],
                0,
            )
        )
    return bytes(buf)


def _cylinder_tris(radius: float, height: float, z0: float = 0.0, axis: str = "z", segs: int = 20) -> list[Any]:
    tris = []
    z1 = z0 + height

    def _pt(x: float, y: float, z: float) -> tuple[float, float, float]:
        if axis == "y":
            return (x, z, -y)
        if axis == "x":
            return (z, x, y)
        return (x, y, z)

    c0 = _pt(0.0, 0.0, z0)
    c1 = _pt(0.0, 0.0, z1)
    for i in range(segs):
        a0 = (2.0 * math.pi * i) / segs
        a1 = (2.0 * math.pi * (i + 1)) / segs
        p00 = _pt(radius * math.cos(a0), radius * math.sin(a0), z0)
        p01 = _pt(radius * math.cos(a1), radius * math.sin(a1), z0)
        p10 = _pt(radius * math.cos(a0), radius * math.sin(a0), z1)
        p11 = _pt(radius * math.cos(a1), radius * math.sin(a1), z1)
        tris.append((p00, p01, p11))
        tris.append((p00, p11, p10))
        tris.append((c0, p01, p00))
        tris.append((c1, p10, p11))
    return tris


def _box_tris(sx: float, sy: float, sz: float, cx: float = 0.0, cy: float = 0.0, cz: float = 0.0) -> list[Any]:
    hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
    v = [
        (cx - hx, cy - hy, cz - hz),
        (cx + hx, cy - hy, cz - hz),
        (cx + hx, cy + hy, cz - hz),
        (cx - hx, cy + hy, cz - hz),
        (cx - hx, cy - hy, cz + hz),
        (cx + hx, cy - hy, cz + hz),
        (cx + hx, cy + hy, cz + hz),
        (cx - hx, cy + hy, cz + hz),
    ]
    faces = [
        (0, 2, 1), (0, 3, 2),  # -Z
        (4, 5, 6), (4, 6, 7),  # +Z
        (0, 1, 5), (0, 5, 4),  # -Y
        (2, 3, 7), (2, 7, 6),  # +Y
        (0, 4, 7), (0, 7, 3),  # -X
        (1, 2, 6), (1, 6, 5),  # +X
    ]
    return [(v[a], v[b], v[c]) for a, b, c in faces]


def build_sample_robot_package_zip(robot_id: str = "motphys-arm6-demo") -> bytes:
    """Build a complete, authentic 6-DOF + gripper URDF + binary STL + manifest .zip template."""
    urdf_xml = """<?xml version="1.0"?>
<robot name="motphys_arm6_demo">
  <material name="slate_dark"><color rgba="0.16 0.21 0.27 1.0"/></material>
  <material name="titanium_silver"><color rgba="0.82 0.86 0.90 1.0"/></material>
  <material name="motphys_teal"><color rgba="0.09 0.50 0.52 1.0"/></material>
  <material name="gripper_accent"><color rgba="0.93 0.36 0.24 1.0"/></material>

  <link name="base_link">
    <visual><geometry><mesh filename="meshes/base.stl"/></geometry><material name="slate_dark"/></visual>
  </link>
  <link name="link_1">
    <visual><geometry><mesh filename="meshes/link1.stl"/></geometry><material name="motphys_teal"/></visual>
  </link>
  <link name="link_2">
    <visual><geometry><mesh filename="meshes/link2.stl"/></geometry><material name="titanium_silver"/></visual>
  </link>
  <link name="link_3">
    <visual><geometry><mesh filename="meshes/link3.stl"/></geometry><material name="titanium_silver"/></visual>
  </link>
  <link name="link_4">
    <visual><geometry><mesh filename="meshes/link4.stl"/></geometry><material name="motphys_teal"/></visual>
  </link>
  <link name="link_5">
    <visual><geometry><mesh filename="meshes/link5.stl"/></geometry><material name="titanium_silver"/></visual>
  </link>
  <link name="ee_link">
    <visual><geometry><mesh filename="meshes/ee_base.stl"/></geometry><material name="slate_dark"/></visual>
  </link>
  <link name="finger_left">
    <visual><geometry><mesh filename="meshes/finger.stl"/></geometry><material name="gripper_accent"/></visual>
  </link>
  <link name="finger_right">
    <visual><geometry><mesh filename="meshes/finger.stl"/></geometry><material name="gripper_accent"/></visual>
  </link>

  <joint name="arm_j1" type="revolute">
    <parent link="base_link"/><child link="link_1"/>
    <origin xyz="0 0 0.14" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.05" upper="3.05" effort="80" velocity="2.2"/>
  </joint>
  <joint name="arm_j2" type="revolute">
    <parent link="link_1"/><child link="link_2"/>
    <origin xyz="0 0 0.12" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-2.10" upper="2.10" effort="80" velocity="2.2"/>
  </joint>
  <joint name="arm_j3" type="revolute">
    <parent link="link_2"/><child link="link_3"/>
    <origin xyz="0 0 0.36" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-2.60" upper="2.60" effort="60" velocity="2.5"/>
  </joint>
  <joint name="arm_j4" type="revolute">
    <parent link="link_3"/><child link="link_4"/>
    <origin xyz="0 0 0.32" rpy="0 0 0"/><axis xyz="1 0 0"/>
    <limit lower="-3.05" upper="3.05" effort="30" velocity="3.0"/>
  </joint>
  <joint name="arm_j5" type="revolute">
    <parent link="link_4"/><child link="link_5"/>
    <origin xyz="0 0 0.09" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-2.20" upper="2.20" effort="30" velocity="3.0"/>
  </joint>
  <joint name="arm_j6" type="revolute">
    <parent link="link_5"/><child link="ee_link"/>
    <origin xyz="0 0 0.09" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="20" velocity="3.5"/>
  </joint>
  <joint name="gripper_finger_left" type="prismatic">
    <parent link="ee_link"/><child link="finger_left"/>
    <origin xyz="0 0.008 0.055" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="0.0" upper="0.04" effort="20" velocity="0.5"/>
  </joint>
  <joint name="gripper_finger_right" type="prismatic">
    <parent link="ee_link"/><child link="finger_right"/>
    <origin xyz="0 -0.008 0.055" rpy="0 0 0"/><axis xyz="0 -1 0"/>
    <limit lower="0.0" upper="0.04" effort="20" velocity="0.5"/>
    <mimic joint="gripper_finger_left" multiplier="1.0" offset="0.0"/>
  </joint>
</robot>
"""

    meshes: dict[str, bytes] = {
        "urdf/meshes/base.stl": _pack_triangles_stl(
            _cylinder_tris(0.115, 0.035, 0.0) + _cylinder_tris(0.072, 0.105, 0.035)
        ),
        "urdf/meshes/link1.stl": _pack_triangles_stl(
            _cylinder_tris(0.064, 0.12, 0.0) + _cylinder_tris(0.056, 0.125, -0.0625, axis="y")
        ),
        "urdf/meshes/link2.stl": _pack_triangles_stl(
            _cylinder_tris(0.052, 0.115, -0.0575, axis="y")
            + _box_tris(0.072, 0.066, 0.34, 0.0, 0.0, 0.18)
            + _cylinder_tris(0.048, 0.105, -0.0525, axis="y")
        ),
        "urdf/meshes/link3.stl": _pack_triangles_stl(
            _cylinder_tris(0.048, 0.105, -0.0525, axis="y")
            + _box_tris(0.058, 0.054, 0.30, 0.0, 0.0, 0.16)
        ),
        "urdf/meshes/link4.stl": _pack_triangles_stl(
            _cylinder_tris(0.042, 0.09, 0.0)
        ),
        "urdf/meshes/link5.stl": _pack_triangles_stl(
            _cylinder_tris(0.038, 0.085, -0.0425, axis="y")
            + _cylinder_tris(0.034, 0.09, 0.0)
        ),
        "urdf/meshes/ee_base.stl": _pack_triangles_stl(
            _cylinder_tris(0.036, 0.025, 0.0)
            + _box_tris(0.044, 0.108, 0.036, 0.0, 0.0, 0.040)
        ),
        "urdf/meshes/finger.stl": _pack_triangles_stl(
            _box_tris(0.020, 0.012, 0.075, 0.0, 0.0, 0.0375)
        ),
    }

    manifest = {
        "schema_version": "1.0",
        "id": robot_id,
        "name": "Motphys Arm6 Cobot (Sample)",
        "robot_type": "motphys_arm6",
        "mode": "mock",
        "connector_type": "mock",
        "readonly_only": False,
        "endpoint": "",
        "arms_scope": ["left"],
        "network_note": "标准机器人包示例：可直接用于 3D + 6D IK + 轨迹预演，切 connector_type=edge 可对接 motrix-edge /v1。",
        "safety_note": "默认安全初始位避开奇异点；非 Mock 连接强制默认只读。",
        "model": {
            "urdf_path": "urdf/robot.urdf",
            "label": "Motphys Arm6 标准 URDF (STL)",
            "format_label": "STL",
            "default_view_dir": [1.35, 0.75, 1.35],
            "trail_links": [
                {"name": "ee_link", "color": "#00f0ff", "light_color": "#0284c7", "label": "末端"},
            ],
        },
        "kinematics": {
            "base_frame": "base_link",
            "arms": {
                "left": {"tip": "ee_link", "group": "left_arm"},
            },
            "trail_links": ["ee_link"],
        },
        "capabilities": {
            "joints": True,
            "urdf_3d": True,
            "motrix_mirror": True,
            "ee_6d": True,
            "atomic_actions": True,
            "workflows": True,
            "navigation": False,
            "chassis_teleop": False,
            "cameras": True,
            "head_depth": False,
            "yolo_foundation_pose": False,
            "aruco": False,
            "policy_inference": True,
            "dynamic_checkpoints": True,
            "offline_replay": True,
            "piper_safety_wizard": False,
            "teach_arms_only": True,
            "loop_registry": True,
        },
        "cameras": [
            {"key": "wrist_rgb", "label": "Arm6 Wrist RGB-D", "resolution": "640x480", "has_depth": True, "fps_target": 30.0},
            {"key": "overhead_rgb", "label": "Workspace Overhead RGB", "resolution": "1280x720", "has_depth": False, "fps_target": 15.0},
        ],
        "joints": [
            {"name": "j1_pan", "label": "J1 Shoulder Pan", "group": "left_arm", "urdf_joint": "arm_j1", "lower": -3.05, "upper": 3.05, "default": 0.0, "unit": "rad", "sign": 1.0, "zero_offset": 0.0, "urdf_scale": 1.0, "max_step_rad": 0.15},
            {"name": "j2_lift", "label": "J2 Shoulder Lift", "group": "left_arm", "urdf_joint": "arm_j2", "lower": -2.10, "upper": 2.10, "default": 0.45, "unit": "rad", "sign": 1.0, "zero_offset": 0.0, "urdf_scale": 1.0, "max_step_rad": 0.15},
            {"name": "j3_elbow", "label": "J3 Elbow Pitch", "group": "left_arm", "urdf_joint": "arm_j3", "lower": -2.60, "upper": 2.60, "default": 0.85, "unit": "rad", "sign": 1.0, "zero_offset": 0.0, "urdf_scale": 1.0, "max_step_rad": 0.15},
            {"name": "j4_roll", "label": "J4 Wrist Roll", "group": "left_arm", "urdf_joint": "arm_j4", "lower": -3.05, "upper": 3.05, "default": 0.0, "unit": "rad", "sign": 1.0, "zero_offset": 0.0, "urdf_scale": 1.0, "max_step_rad": 0.15},
            {"name": "j5_pitch", "label": "J5 Wrist Pitch", "group": "left_arm", "urdf_joint": "arm_j5", "lower": -2.20, "upper": 2.20, "default": 0.40, "unit": "rad", "sign": 1.0, "zero_offset": 0.0, "urdf_scale": 1.0, "max_step_rad": 0.15},
            {"name": "j6_flange", "label": "J6 Wrist Flange", "group": "left_arm", "urdf_joint": "arm_j6", "lower": -3.14, "upper": 3.14, "default": 0.0, "unit": "rad", "sign": 1.0, "zero_offset": 0.0, "urdf_scale": 1.0, "max_step_rad": 0.15},
            {"name": "gripper", "label": "Parallel Gripper", "group": "gripper", "urdf_joint": "gripper_finger_left", "lower": 0.0, "upper": 1.0, "default": 0.65, "unit": "norm", "is_gripper": True, "sign": 1.0, "zero_offset": 0.0, "urdf_scale": 0.04, "max_step_rad": 0.25},
        ],
    }

    readme = (
        "# Motphys Robot Package Template\n\n"
        "替换 urdf/robot.urdf 与 urdf/meshes/*，并按需修改 robot_manifest.json 即可直接上传导入。\n"
        "也支持仅打包 urdf/robot.urdf + urdf/meshes/*（不含 robot_manifest.json），平台将自动解析关节、限位与运动学链。\n"
    )

    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        zf.writestr("robot_manifest.json", json.dumps(manifest, ensure_ascii=False, indent=2))
        zf.writestr("urdf/robot.urdf", urdf_xml)
        zf.writestr("README.md", readme)
        for rel, data in meshes.items():
            zf.writestr(rel, data)
    return buf.getvalue()


# ---- Helpers ----

def _slugify(raw: str, fallback: str = "custom-robot") -> str:
    s = re.sub(r"[^a-z0-9_-]+", "-", str(raw).strip().lower()).strip("-_")
    if len(s) < 3:
        s = fallback
    return s[:48]


def _guess_group(joint_name: str, jtype: str) -> tuple[str, bool]:
    low = joint_name.lower()
    if any(k in low for k in ("grip", "finger", "claw", "jaw")):
        return "gripper", True
    if any(k in low for k in ("head", "neck", "zed", "cam")):
        return "head", False
    if any(k in low for k in ("torso", "waist", "spine", "chest", "buttock")):
        return "torso", False
    if any(k in low for k in ("leg", "knee", "ankle", "hip", "foot", "wheel")):
        return "leg", False
    if any(k in low for k in ("right", "_r_", "r_arm", "r_shoulder", "r_elbow", "r_wrist")) or low.startswith("r_"):
        return "right_arm", False
    if any(k in low for k in ("left", "_l_", "l_arm", "l_shoulder", "l_elbow", "l_wrist")) or low.startswith("l_"):
        return "left_arm", False
    return ("gripper", True) if jtype == "prismatic" else ("left_arm", False)


def _format_from_meshes(mesh_refs: list[str]) -> str:
    exts = {Path(m).suffix.lower().lstrip(".") for m in mesh_refs if m}
    if "glb" in exts or "gltf" in exts:
        return "GLB"
    if "obj" in exts and "stl" in exts:
        return "STL/OBJ"
    if "obj" in exts:
        return "OBJ"
    if "stl" in exts:
        return "STL"
    return "URDF"


def _strip_common_root(extract_dir: Path) -> None:
    """If zip wrapped everything in a single folder (e.g. my_robot/...), hoist it cleanly."""
    entries = [p for p in extract_dir.iterdir() if p.name != "__MACOSX"]
    if len(entries) == 1 and entries[0].is_dir():
        inner = entries[0]
        for child in list(inner.iterdir()):
            shutil.move(str(child), str(extract_dir / child.name))
        shutil.rmtree(inner, ignore_errors=True)
    macosx = extract_dir / "__MACOSX"
    if macosx.exists():
        shutil.rmtree(macosx, ignore_errors=True)


def safe_extract_zip(zip_bytes: bytes, dest_dir: Path) -> list[str]:
    """Safely extract zip with Zip-Slip, symlink, extension whitelist, and size guards."""
    if not zip_bytes:
        raise ValueError("上传的压缩包为空")
    try:
        zf = zipfile.ZipFile(io.BytesIO(zip_bytes))
    except zipfile.BadZipFile as exc:
        raise ValueError("无效的 .zip 文件格式") from exc

    extracted: list[str] = []
    total_bytes = 0
    dest_resolved = dest_dir.resolve()
    dest_dir.mkdir(parents=True, exist_ok=True)

    with zf:
        infos = zf.infolist()
        if len(infos) > MAX_ZIP_FILES:
            raise ValueError(f"压缩包文件过多（{len(infos)} > 上限 {MAX_ZIP_FILES}）")

        for info in infos:
            raw_name = info.filename.replace("\\", "/")
            if not raw_name or raw_name.startswith("__MACOSX/") or "/__MACOSX/" in raw_name:
                continue
            if raw_name.startswith("/") or raw_name.startswith("../") or "/../" in raw_name or raw_name == "..":
                raise ValueError(f"安全拦截：检测到非法路径穿越条目 '{info.filename}'")
            # Reject symlinks (UNIX mode 0120000)
            if ((info.external_attr >> 16) & 0o170000) == 0o120000:
                raise ValueError(f"安全拦截：不允许包含符号链接 '{info.filename}'")
            if info.is_dir():
                continue

            ext = Path(raw_name).suffix.lower()
            if ext not in ALLOWED_EXTENSIONS:
                raise ValueError(f"安全拦截：不允许的文件类型 '{raw_name}'（仅支持 URDF/网格/贴图/地图/JSON/YAML）")

            total_bytes += info.file_size
            if total_bytes > MAX_ZIP_BYTES:
                raise ValueError(f"压缩包解压总大小超过上限 ({MAX_ZIP_BYTES // (1024 * 1024)} MB)")

            target = (dest_dir / raw_name).resolve()
            if not str(target).startswith(str(dest_resolved) + "/") and target != dest_resolved:
                raise ValueError(f"安全拦截：路径越界 '{info.filename}'")

            target.parent.mkdir(parents=True, exist_ok=True)
            with zf.open(info, "r") as src, target.open("wb") as dst:
                chunk_total = 0
                while True:
                    chunk = src.read(65536)
                    if not chunk:
                        break
                    chunk_total += len(chunk)
                    if chunk_total > MAX_ZIP_BYTES:
                        raise ValueError("压缩包存在解压膨胀风险，已拒绝")
                    dst.write(chunk)
            extracted.append(raw_name)

    _strip_common_root(dest_dir)
    return extracted


class RobotPackageManager:
    """Manages staging, 6-gate preflight validation, and persistence for dynamically imported robots."""

    def __init__(self, root_dir: Path) -> None:
        self.root_dir = root_dir.resolve()
        self.packages_dir = self.root_dir / "packages"
        self.staging_dir = self.root_dir / "_staging"
        self.registry_file = self.root_dir / "registry.json"
        self.root_dir.mkdir(parents=True, exist_ok=True)
        self.packages_dir.mkdir(parents=True, exist_ok=True)
        self.staging_dir.mkdir(parents=True, exist_ok=True)
        self.prune_stale_staging()

    def prune_stale_staging(self, max_age_s: float = 3600.0) -> int:
        """Remove abandoned staging directories older than max_age_s."""
        removed = 0
        now = time.time()
        if not self.staging_dir.exists():
            return 0
        for child in list(self.staging_dir.iterdir()):
            if not child.is_dir():
                continue
            try:
                age = now - child.stat().st_mtime
                if age >= max_age_s:
                    shutil.rmtree(child, ignore_errors=True)
                    removed += 1
            except OSError:
                continue
        return removed

    # ---- Registry Persistence ----

    def load_persisted_descriptors(self) -> list[RobotDescriptor]:
        if not self.registry_file.exists():
            return []
        try:
            raw = json.loads(self.registry_file.read_text(encoding="utf-8"))
        except Exception:
            return []
        out: list[RobotDescriptor] = []
        for item in raw.get("robots", []):
            try:
                desc = RobotDescriptor.model_validate(item)
                out.append(desc)
            except Exception:
                continue
        return out

    def _save_registry(self, descriptors: list[RobotDescriptor]) -> None:
        payload = {
            "schema_version": "1.0",
            "updated_at": time.time(),
            "robots": [d.model_dump() for d in descriptors],
        }
        tmp = self.registry_file.with_suffix(".tmp")
        tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        tmp.replace(self.registry_file)

    def delete_custom_robot(self, robot_id: str) -> bool:
        if robot_id in BUILTIN_ROBOT_IDS:
            raise ValueError(f"内置机型 '{robot_id}' 受保护，不可删除")
        existing = self.load_persisted_descriptors()
        kept = [d for d in existing if d.id != robot_id]
        if len(kept) == len(existing):
            return False
        self._save_registry(kept)
        pkg_dir = (self.packages_dir / robot_id).resolve()
        if pkg_dir.exists() and str(pkg_dir).startswith(str(self.packages_dir) + "/"):
            shutil.rmtree(pkg_dir, ignore_errors=True)
        legacy_dir = (self.root_dir / robot_id).resolve()
        if legacy_dir.exists() and legacy_dir != self.packages_dir and legacy_dir != self.staging_dir and str(legacy_dir).startswith(str(self.root_dir) + "/"):
            shutil.rmtree(legacy_dir, ignore_errors=True)
        return True

    # ---- URDF Discovery, Package URI Normalization & Auto-Draft ----

    def _find_urdf(self, pkg_dir: Path, manifest_hint: str | None = None) -> Path | None:
        if manifest_hint:
            cand = (pkg_dir / manifest_hint.lstrip("/")).resolve()
            if cand.exists() and cand.is_file() and str(cand).startswith(str(pkg_dir.resolve())):
                return cand
        for preferred in ("urdf/robot.urdf", "robot.urdf"):
            cand = pkg_dir / preferred
            if cand.exists():
                return cand
        all_urdfs = sorted(pkg_dir.rglob("*.urdf"))
        return all_urdfs[0] if all_urdfs else None

    def _normalize_urdf_assets(self, pkg_dir: Path, urdf_path: Path) -> dict[str, Any]:
        """Resolve <mesh> and <texture> refs inside pkg_dir; auto-rewrite package:// if local file exists."""
        pkg_resolved = pkg_dir.resolve()
        urdf_dir = urdf_path.parent.resolve()
        tree = ET.parse(urdf_path)
        root = tree.getroot()

        missing: list[str] = []
        remote: list[str] = []
        escaped: list[str] = []
        normalized: list[str] = []
        visual_mesh_paths: list[Path] = []
        texture_files: set[Path] = set()
        modified = False

        def _resolve_ref(raw: str) -> tuple[str | None, Path | None]:
            ref = raw.strip()
            if re.match(r"^(https?:|file:|//)", ref, re.IGNORECASE):
                remote.append(ref)
                return None, None
            if ref.startswith("package://"):
                rest = ref[len("package://"):]
                # Try stripping package prefix first, then full rest
                parts = rest.split("/", 1)
                candidates = []
                if len(parts) == 2:
                    candidates.append(pkg_dir / parts[1])
                    candidates.append(urdf_dir / parts[1])
                candidates.append(pkg_dir / rest)
                for cand in candidates:
                    resolved = cand.resolve()
                    if resolved.exists() and str(resolved).startswith(str(pkg_resolved) + "/"):
                        rel = Path(resolved).relative_to(urdf_dir) if str(resolved).startswith(str(urdf_dir) + "/") else Path(
                            *([ ".."] * len(urdf_dir.relative_to(pkg_resolved).parts)),
                            resolved.relative_to(pkg_resolved),
                        )
                        rel_str = rel.as_posix()
                        normalized.append(f"{ref} -> {rel_str}")
                        return rel_str, resolved
                missing.append(ref)
                return None, None

            cand = (urdf_dir / ref).resolve()
            if not str(cand).startswith(str(pkg_resolved) + "/") and cand != pkg_resolved:
                escaped.append(ref)
                return None, None
            if not cand.exists():
                # Fallback: try relative to pkg_dir root and rewrite
                alt = (pkg_dir / ref).resolve()
                if alt.exists() and str(alt).startswith(str(pkg_resolved) + "/"):
                    rel = Path(
                        *([".."] * len(urdf_dir.relative_to(pkg_resolved).parts)),
                        alt.relative_to(pkg_resolved),
                    )
                    normalized.append(f"{ref} -> {rel.as_posix()}")
                    return rel.as_posix(), alt
                missing.append(ref)
                return None, None
            return ref, cand

        for link_el in root.findall("link"):
            for vis in link_el.findall("visual"):
                mesh_el = vis.find("geometry/mesh")
                if mesh_el is not None and mesh_el.get("filename"):
                    orig = mesh_el.get("filename", "")
                    rewritten, resolved = _resolve_ref(orig)
                    if rewritten and rewritten != orig:
                        mesh_el.set("filename", rewritten)
                        modified = True
                    if resolved:
                        visual_mesh_paths.append(resolved)
                tex_el = vis.find("material/texture")
                if tex_el is not None and tex_el.get("filename"):
                    orig = tex_el.get("filename", "")
                    rewritten, resolved = _resolve_ref(orig)
                    if rewritten and rewritten != orig:
                        tex_el.set("filename", rewritten)
                        modified = True
                    if resolved:
                        texture_files.add(resolved)

        for mat_el in root.findall("material"):
            tex_el = mat_el.find("texture")
            if tex_el is not None and tex_el.get("filename"):
                orig = tex_el.get("filename", "")
                rewritten, resolved = _resolve_ref(orig)
                if rewritten and rewritten != orig:
                    tex_el.set("filename", rewritten)
                    modified = True
                if resolved:
                    texture_files.add(resolved)

        if modified:
            tree.write(urdf_path, encoding="utf-8", xml_declaration=True)

        # Count GLB embedded textures if GLB meshes are present
        glb_tex_count = 0
        for p in set(visual_mesh_paths):
            if p.suffix.lower() == ".glb":
                try:
                    data = p.read_bytes()
                    if len(data) > 20 and data[:4] == b"glTF":
                        json_len = struct.unpack("<I", data[12:16])[0]
                        gltf_json = json.loads(data[20 : 20 + json_len].decode("utf-8", errors="ignore"))
                        glb_tex_count += len(gltf_json.get("images") or [])
                except Exception:
                    pass

        return {
            "visual_count": len(visual_mesh_paths),
            "format_label": _format_from_meshes([str(p) for p in visual_mesh_paths]),
            "embedded_images": len(texture_files) + glb_tex_count,
            "missing": missing,
            "remote": remote,
            "escaped": escaped,
            "normalized": normalized,
        }

    def _discover_maps(self, pkg_dir: Path) -> list[dict[str, Any]]:
        maps: list[dict[str, Any]] = []
        for yml in sorted(pkg_dir.rglob("*.yaml")) + sorted(pkg_dir.rglob("*.yml")):
            try:
                meta = yaml.safe_load(yml.read_text(encoding="utf-8")) or {}
            except Exception:
                continue
            if not isinstance(meta, dict) or "resolution" not in meta or "origin" not in meta:
                continue
            img_name = str(meta.get("image") or yml.with_suffix(".png").name)
            img_path = (yml.parent / img_name).resolve()
            if not img_path.exists():
                continue
            rel_img = img_path.relative_to(pkg_dir.resolve()).as_posix()
            rel_yml = yml.resolve().relative_to(pkg_dir.resolve()).as_posix()
            pcd_cands = sorted(yml.parent.glob("*.pcd"))
            rel_pcd = pcd_cands[0].resolve().relative_to(pkg_dir.resolve()).as_posix() if pcd_cands else None
            maps.append(
                {
                    "id": _slugify(yml.stem, "imported-map"),
                    "name": f"导入地图 ({yml.stem} · {meta['resolution']}m/pix)",
                    "kind": "recorded_slam_map" if rel_pcd else "occupancy_grid",
                    "rel_image": rel_img,
                    "rel_yaml": rel_yml,
                    "rel_pcd": rel_pcd,
                    "yaml": {
                        "resolution": float(meta.get("resolution", 0.05)),
                        "origin": [float(v) for v in (meta.get("origin") or [0.0, 0.0, 0.0])[:3]],
                    },
                }
            )
        return maps

    def _auto_draft_manifest(self, pkg_dir: Path, urdf_path: Path, urdf_info: dict[str, Any], asset_stats: dict[str, Any]) -> dict[str, Any]:
        rel_urdf = urdf_path.resolve().relative_to(pkg_dir.resolve()).as_posix()
        base_id = _slugify(f"{urdf_info['robot_name']}-custom", "custom-robot")
        robot_type = _slugify(urdf_info["robot_name"], "custom_robot").replace("-", "_")
        base_frame = urdf_info["root_links"][0] if urdf_info["root_links"] else "base_link"

        joints_draft: list[dict[str, Any]] = []
        groups_present: set[str] = set()
        for j in urdf_info["movable_joints"]:
            grp, is_grip = _guess_group(j["name"], j["type"])
            groups_present.add(grp)
            lo, hi = float(j["lower"]), float(j["upper"])
            if hi <= lo:
                lo, hi = -3.14, 3.14
            if is_grip and j["type"] == "prismatic" and hi > 0:
                default_val = 0.5
                scale = round(hi, 4)
                lo_out, hi_out, unit = 0.0, 1.0, "norm"
            else:
                default_val = round(max(lo, min(hi, 0.0)), 3)
                scale = 1.0
                lo_out, hi_out, unit = round(lo, 4), round(hi, 4), "rad"
            joints_draft.append(
                {
                    "name": j["name"],
                    "label": j["name"].replace("_", " ").title(),
                    "group": grp,
                    "urdf_joint": j["name"],
                    "lower": lo_out,
                    "upper": hi_out,
                    "default": default_val,
                    "unit": unit,
                    "is_gripper": is_grip,
                    "sign": 1.0,
                    "zero_offset": 0.0,
                    "urdf_scale": scale,
                    "max_step_rad": 0.15,
                }
            )

        # Auto-infer arm tips from deepest links driven by left_arm / right_arm joints
        child_by_joint = {j["name"]: j["child"] for j in urdf_info["joints"]}
        arms: dict[str, dict[str, str]] = {}
        trail_links: list[dict[str, Any]] = []
        for side, grp, col, lcol, lbl in (
            ("left", "left_arm", "#00f0ff", "#0284c7", "左末端"),
            ("right", "right_arm", "#ff2e93", "#db2777", "右末端"),
        ):
            grp_joints = [j for j in joints_draft if j["group"] == grp]
            if grp_joints:
                tip = child_by_joint.get(grp_joints[-1]["urdf_joint"])
                if tip:
                    arms[side] = {"tip": tip, "group": grp}
                    trail_links.append({"name": tip, "color": col, "light_color": lcol, "label": lbl})

        if not arms and urdf_info["leaf_links"] and joints_draft:
            first_grp = joints_draft[0]["group"]
            tip = urdf_info["leaf_links"][0]
            arms["left"] = {"tip": tip, "group": first_grp}
            trail_links.append({"name": tip, "color": "#00f0ff", "light_color": "#0284c7", "label": "末端"})

        discovered_maps = self._discover_maps(pkg_dir)
        has_nav = len(discovered_maps) > 0

        return {
            "schema_version": "1.0",
            "id": base_id,
            "name": urdf_info["robot_name"].replace("_", " ").title(),
            "robot_type": robot_type,
            "mode": "mock",
            "connector_type": "mock",
            "readonly_only": False,
            "endpoint": "",
            "arms_scope": list(arms.keys()) or ["left"],
            "network_note": "通过机器人包导入向导自动解析生成；连接真机时默认锁只读，需核验关节方向后再开放控制。",
            "safety_note": "软限位由导入包 URDF/Manifest 定义；软件急停不等同于硬件断电急停。",
            "model": {
                "urdf_path": rel_urdf,
                "label": f"{urdf_info['robot_name']} ({asset_stats['format_label']})",
                "format_label": asset_stats["format_label"],
                "preserve_gltf_pbr": asset_stats["format_label"] == "GLB",
                "default_view_dir": [1.35, 0.7, 1.35],
                "trail_links": trail_links,
            },
            "kinematics": {
                "base_frame": base_frame,
                "arms": arms,
                "trail_links": [t["name"] for t in trail_links],
            },
            "capabilities": {
                "joints": True,
                "urdf_3d": True,
                "motrix_mirror": True,
                "ee_6d": len(arms) > 0,
                "atomic_actions": True,
                "workflows": True,
                "navigation": has_nav,
                "chassis_teleop": has_nav,
                "cameras": True,
                "head_depth": False,
                "yolo_foundation_pose": False,
                "aruco": False,
                "policy_inference": False,
                "dynamic_checkpoints": False,
                "offline_replay": False,
                "piper_safety_wizard": False,
                "teach_arms_only": True,
                "loop_registry": True,
            },
            "cameras": [
                {"key": "main_rgb", "label": "Main Workspace RGB", "resolution": "1280x720", "has_depth": False, "fps_target": 15.0}
            ],
            "joints": joints_draft,
            "discovered_maps": discovered_maps,
        }

    # ---- Stage 1: Inspect Zip ----

    def inspect_zip(self, zip_bytes: bytes, existing_connectors: dict[str, Any]) -> dict[str, Any]:
        self.prune_stale_staging()
        stage_id = f"stg-{uuid.uuid4().hex[:10]}"
        stage_dir = self.staging_dir / stage_id
        try:
            extracted_files = safe_extract_zip(zip_bytes, stage_dir)
            manifest_file = stage_dir / "robot_manifest.json"
            raw_manifest: dict[str, Any] | None = None
            manifest_source = "auto_urdf"
            if manifest_file.exists():
                raw_manifest = json.loads(manifest_file.read_text(encoding="utf-8"))
                manifest_source = "manifest_json"

            hint = (raw_manifest or {}).get("model", {}).get("urdf_path") or (raw_manifest or {}).get("urdf_path")
            urdf_path = self._find_urdf(stage_dir, manifest_hint=hint)
            if urdf_path is None:
                raise ValueError("压缩包中未找到任何 .urdf 文件")

            asset_stats = self._normalize_urdf_assets(stage_dir, urdf_path)
            urdf_info = inspect_urdf_file(urdf_path)
            auto_draft = self._auto_draft_manifest(stage_dir, urdf_path, urdf_info, asset_stats)

            if raw_manifest is not None:
                merged = dict(auto_draft)
                for k, v in raw_manifest.items():
                    if k in ("model", "kinematics", "capabilities") and isinstance(v, dict):
                        merged[k] = {**auto_draft.get(k, {}), **v}
                    else:
                        merged[k] = v
                manifest = merged
            else:
                manifest = auto_draft

            # Ensure unique default ID if already taken
            if manifest.get("id") in existing_connectors:
                manifest["id"] = _slugify(f"{manifest['id']}-{uuid.uuid4().hex[:4]}")

            validation = self.validate_staged(stage_id, manifest, existing_connectors)
            return {
                "stage_id": stage_id,
                "manifest_source": manifest_source,
                "extracted_count": len(extracted_files),
                "urdf_summary": {
                    "robot_name": urdf_info["robot_name"],
                    "urdf_path": urdf_path.resolve().relative_to(stage_dir.resolve()).as_posix(),
                    "links": urdf_info["links"],
                    "movable_joints": urdf_info["movable_joints"],
                    "visual_count": asset_stats["visual_count"],
                    "format_label": asset_stats["format_label"],
                    "embedded_images": asset_stats["embedded_images"],
                    "normalized_uris": asset_stats["normalized"],
                },
                "manifest": manifest,
                **validation,
            }
        except Exception:
            shutil.rmtree(stage_dir, ignore_errors=True)
            raise

    # ---- Stage 2: 6-Gate Preflight Validation ----

    def _build_descriptor_from_manifest(
        self,
        manifest: dict[str, Any],
        url_prefix: str,
        asset_stats: dict[str, Any],
        pkg_dir: Path,
    ) -> RobotDescriptor:
        rid = str(manifest.get("id") or "").strip()
        name = str(manifest.get("name") or rid).strip()
        rtype = str(manifest.get("robot_type") or rid.replace("-", "_")).strip().lower()
        ctype = str(manifest.get("connector_type") or "mock").strip()
        mode = str(manifest.get("mode") or ("mock" if ctype == "mock" else "real")).strip()
        readonly = bool(manifest.get("readonly_only", ctype != "mock"))
        endpoint = str(manifest.get("endpoint") or "").strip()

        model_cfg = manifest.get("model") or {}
        rel_urdf = str(model_cfg.get("urdf_path") or "urdf/robot.urdf").lstrip("/")
        urdf_url = f"{url_prefix}/{rel_urdf}"

        trail_specs = []
        for t in (model_cfg.get("trail_links") or []):
            if isinstance(t, dict) and t.get("name"):
                trail_specs.append(
                    ModelTrailLinkSpec(
                        name=str(t["name"]),
                        color=str(t.get("color") or "#00f0ff"),
                        light_color=str(t.get("light_color") or "#0284c7"),
                        label=str(t.get("label") or t["name"]),
                    )
                )

        kin_raw = manifest.get("kinematics") or {}
        arms_dict: dict[str, KinematicsArmSpec] = {}
        for arm_name, acfg in (kin_raw.get("arms") or {}).items():
            if isinstance(acfg, dict) and acfg.get("tip") and acfg.get("group"):
                arms_dict[str(arm_name)] = KinematicsArmSpec(tip=str(acfg["tip"]), group=str(acfg["group"]))

        kin_profile = KinematicsProfile(
            base_frame=str(kin_raw.get("base_frame") or "base_link"),
            arms=arms_dict,
            trail_links=[t.name for t in trail_specs] or list(kin_raw.get("trail_links") or []),
        )

        model_profile = ModelProfile(
            label=str(model_cfg.get("label") or f"{name} URDF"),
            format_label=str(model_cfg.get("format_label") or asset_stats.get("format_label") or "URDF"),
            url=urdf_url,
            visual_count=int(asset_stats.get("visual_count", 0)),
            embedded_images=int(asset_stats.get("embedded_images", 0)),
            preserve_gltf_pbr=bool(model_cfg.get("preserve_gltf_pbr", asset_stats.get("format_label") == "GLB")),
            pbr_profile=model_cfg.get("pbr_profile"),
            default_view_dir=[float(v) for v in (model_cfg.get("default_view_dir") or [1.35, 0.7, 1.35])[:3]],
            trail_links=trail_specs,
            hands=list(model_cfg.get("hands") or []),
        )

        joints = [JointSpec.model_validate(j) for j in (manifest.get("joints") or [])]
        cameras = [CameraSpec.model_validate(c) for c in (manifest.get("cameras") or [])]
        caps = RobotCapabilities.model_validate(manifest.get("capabilities") or {})

        # Resolve optional discovered maps
        extra_maps: list[dict[str, Any]] = []
        for m in (manifest.get("discovered_maps") or self._discover_maps(pkg_dir)):
            if not isinstance(m, dict) or not m.get("rel_image"):
                continue
            rel_pcd = m.get("rel_pcd")
            extra_maps.append(
                {
                    "id": str(m.get("id") or "custom-map"),
                    "name": str(m.get("name") or "导入地图"),
                    "kind": str(m.get("kind") or "occupancy_grid"),
                    "image_url": f"{url_prefix}/{str(m['rel_image']).lstrip('/')}",
                    "yaml_url": f"{url_prefix}/{str(m.get('rel_yaml') or '').lstrip('/')}",
                    "yaml": m.get("yaml") or {"resolution": 0.05, "origin": [0.0, 0.0, 0.0]},
                    "pointcloud": {
                        "available": bool(rel_pcd),
                        "live": False,
                        "source": "recorded_lidar_pcd" if rel_pcd else "mock_from_occupancy",
                        "pcd_url": f"{url_prefix}/{str(rel_pcd).lstrip('/')}" if rel_pcd else None,
                        "frame": "map",
                        "label": "导入包静态 3D 点云（离线地图，非实时扫描）" if rel_pcd else "由导入 2D 栅格生成示例点云",
                    },
                }
            )

        map_img = extra_maps[0]["image_url"] if extra_maps else None
        map_yml = extra_maps[0]["yaml"] if extra_maps else None

        ros2_cfg = None
        if ctype == "ros2_bridge":
            raw_r2 = dict(manifest.get("ros2_config") or {})
            if endpoint and not raw_r2.get("ws_url"):
                raw_r2["ws_url"] = endpoint
            elif raw_r2.get("ws_url") and not endpoint:
                endpoint = str(raw_r2["ws_url"])
            ros2_cfg = Ros2BridgeConfig.model_validate(raw_r2)

        return RobotDescriptor(
            id=rid,
            name=name,
            robot_type=rtype,
            mode=mode,  # type: ignore[arg-type]
            integration_status="mock" if ctype == "mock" else ("live" if ctype == "ros2_bridge" else "unimplemented"),
            connector_type=ctype,  # type: ignore[arg-type]
            readonly_only=readonly,
            is_custom=True,
            endpoint=endpoint,
            ros2_config=ros2_cfg,
            arms_scope=list(manifest.get("arms_scope") or list(arms_dict.keys()) or ["left"]),
            urdf_url=urdf_url,
            map_image_url=map_img,
            map_yaml=map_yml,
            model_profile=model_profile,
            kinematics=kin_profile,
            extra_maps=extra_maps,
            network_note=str(manifest.get("network_note") or ""),
            safety_note=str(manifest.get("safety_note") or "软件 E-Stop 不等同于现场硬件断电急停。"),
            joints=joints,
            cameras=cameras,
            capabilities=caps,
        )

    def validate_staged(
        self,
        stage_id: str,
        manifest: dict[str, Any],
        existing_connectors: dict[str, Any],
    ) -> dict[str, Any]:
        stage_dir = (self.staging_dir / stage_id).resolve()
        if not stage_dir.exists() or not str(stage_dir).startswith(str(self.staging_dir.resolve()) + "/"):
            raise ValueError(f"暂存包 '{stage_id}' 不存在或已过期，请重新上传")

        checks: list[dict[str, Any]] = []
        hint = (manifest.get("model") or {}).get("urdf_path") or manifest.get("urdf_path")
        urdf_path = self._find_urdf(stage_dir, manifest_hint=hint)

        # Gate 1: Asset Integrity
        asset_stats: dict[str, Any] = {"visual_count": 0, "format_label": "URDF", "embedded_images": 0, "missing": [], "remote": [], "escaped": []}
        urdf_info: dict[str, Any] = {"links": [], "joints": [], "movable_joints": []}
        if urdf_path is None:
            checks.append({"id": "assets_integrity", "label": "1. URDF 与网格贴图完整性", "ok": False, "detail": f"未找到 URDF 文件 ({hint or 'robot.urdf'})"})
        else:
            try:
                asset_stats = self._normalize_urdf_assets(stage_dir, urdf_path)
                urdf_info = inspect_urdf_file(urdf_path)
                errs = []
                if asset_stats["remote"]:
                    errs.append(f"含禁止的外部远程 URL: {asset_stats['remote'][:2]}")
                if asset_stats["escaped"]:
                    errs.append(f"含越界路径: {asset_stats['escaped'][:2]}")
                if asset_stats["missing"]:
                    errs.append(f"缺失网格/贴图文件 ({len(asset_stats['missing'])}): {asset_stats['missing'][:3]}")
                if asset_stats["visual_count"] == 0:
                    errs.append("URDF 未包含任何 <visual><mesh> 几何体")
                checks.append({
                    "id": "assets_integrity",
                    "label": "1. URDF 与网格贴图完整性",
                    "ok": len(errs) == 0,
                    "detail": "; ".join(errs) if errs else f"{asset_stats['visual_count']} 个 {asset_stats['format_label']} 视觉网格完整校验通过（0 缺失 / 0 外链）",
                })
            except Exception as exc:
                checks.append({"id": "assets_integrity", "label": "1. URDF 与网格贴图完整性", "ok": False, "detail": f"URDF 解析失败: {exc}"})

        # Gate 2: Joint Mapping & Limits
        urdf_joint_names = {j["name"] for j in urdf_info.get("joints", [])}
        raw_joints = manifest.get("joints") or []
        joint_errs: list[str] = []
        seen_names: set[str] = set()
        if not raw_joints:
            joint_errs.append("关节表为空，至少需要 1 个关节")
        for j in raw_joints:
            jn = str(j.get("name") or "").strip()
            uj = str(j.get("urdf_joint") or "").strip()
            if not jn:
                joint_errs.append("存在空关节名")
                continue
            if jn in seen_names:
                joint_errs.append(f"重复关节名 '{jn}'")
            seen_names.add(jn)
            if uj and urdf_joint_names and uj not in urdf_joint_names:
                joint_errs.append(f"'{jn}' 映射的 urdf_joint '{uj}' 在 URDF 中不存在")
            try:
                lo, hi, df = float(j["lower"]), float(j["upper"]), float(j.get("default", 0.0))
                sign = float(j.get("sign", 1.0))
                scale = float(j.get("urdf_scale", 1.0))
                step = float(j.get("max_step_rad", 0.15))
                if not all(math.isfinite(x) for x in (lo, hi, df, sign, scale, step)):
                    joint_errs.append(f"'{jn}' 存在 NaN/Inf 参数")
                elif hi <= lo:
                    joint_errs.append(f"'{jn}' 限位非法 (lower={lo} >= upper={hi})")
                elif not (lo - 1e-6 <= df <= hi + 1e-6):
                    joint_errs.append(f"'{jn}' 默认位 {df} 超出软限位 [{lo}, {hi}]")
                elif sign == 0 or scale <= 0 or step <= 0:
                    joint_errs.append(f"'{jn}' sign/scale/max_step_rad 必须非零且为正")
            except Exception as exc:
                joint_errs.append(f"'{jn}' 参数格式错误: {exc}")
        checks.append({
            "id": "joints_mapping",
            "label": "2. 关节映射与软限位闭环",
            "ok": len(joint_errs) == 0,
            "detail": "; ".join(joint_errs[:3]) if joint_errs else f"{len(raw_joints)} 个关节全部命中 URDF 且软限位/方向系数合法",
        })

        # Build candidate descriptor (catch Pydantic schema errors)
        descriptor: RobotDescriptor | None = None
        schema_err = ""
        try:
            rid_preview = str(manifest.get("id") or "preview")
            descriptor = self._build_descriptor_from_manifest(
                manifest,
                url_prefix=f"/custom-robots/{rid_preview}",
                asset_stats=asset_stats,
                pkg_dir=stage_dir,
            )
        except Exception as exc:
            schema_err = str(exc)

        # Gate 3: Kinematics FK + IK Self-Check
        if descriptor is None or urdf_path is None or not checks[0]["ok"] or not checks[1]["ok"]:
            checks.append({
                "id": "kinematics_fk_ik",
                "label": "3. 运动学链 FK / 6D IK 连通性自检",
                "ok": False,
                "detail": schema_err or "前置 URDF 或关节检查未通过，跳过运动学自检",
            })
        elif not descriptor.capabilities.ee_6d or not (descriptor.kinematics and descriptor.kinematics.arms):
            checks.append({
                "id": "kinematics_fk_ik",
                "label": "3. 运动学链 FK / 6D IK 连通性自检",
                "ok": True,
                "detail": "未启用 6D IK 末端链路（仅使用关节空间控制与轨迹规划）",
            })
        else:
            try:
                kin = RobotKinematics(descriptor, urdf_path=urdf_path)
                defaults = {j.name: j.default for j in descriptor.joints}
                arm_reports = []
                kin_errs = []
                for arm_name, acfg in kin.arm_cfg.items():
                    fk0 = kin.fk(arm_name, defaults)
                    # Build a guaranteed-reachable interior target by perturbing active arm joints within soft limits
                    q_probe = dict(defaults)
                    active_count = 0
                    for j in descriptor.joints:
                        if j.group == acfg["group"] and not j.is_gripper:
                            span = j.upper - j.lower
                            step = min(0.12, span * 0.12)
                            cand = j.default + step if j.default + step <= j.upper else j.default - step
                            q_probe[j.name] = max(j.lower, min(j.upper, cand))
                            active_count += 1
                    fk_tgt = kin.fk(arm_name, q_probe)
                    ik_res = kin.solve_ik(
                        arm_name,
                        fk_tgt["position"],
                        fk_tgt["rpy_deg"],
                        defaults,
                        position_only=(active_count < 6),
                        max_iter=100,
                    )
                    err_mm = ik_res["pos_error_m"] * 1000.0
                    if active_count == 0:
                        kin_errs.append(f"{arm_name} 关节组 '{acfg['group']}' 未匹配到任何活动关节")
                    elif err_mm > 15.0:
                        kin_errs.append(f"{arm_name} IK 误差偏大 ({err_mm:.1f}mm)")
                    arm_reports.append(f"{arm_name}({acfg['tip']}) FK+IK 误差 {err_mm:.2f}mm (起 {fk0['position'][0]:.2f},{fk0['position'][2]:.2f})")
                checks.append({
                    "id": "kinematics_fk_ik",
                    "label": "3. 运动学链 FK / 6D IK 连通性自检",
                    "ok": len(kin_errs) == 0,
                    "detail": "; ".join(kin_errs) if kin_errs else " · ".join(arm_reports),
                })
            except Exception as exc:
                checks.append({
                    "id": "kinematics_fk_ik",
                    "label": "3. 运动学链 FK / 6D IK 连通性自检",
                    "ok": False,
                    "detail": f"运动学链构建失败: {exc}",
                })

        # Gate 4: Safety Defaults & Non-Mock Read-Only Guard
        safety_errs: list[str] = []
        ctype = str(manifest.get("connector_type") or "mock")
        readonly = bool(manifest.get("readonly_only", ctype != "mock"))
        endpoint = str(manifest.get("endpoint") or "").strip()
        if ctype != "mock" and not readonly:
            safety_errs.append("非 Mock 真机/仿真连接入库时必须强制开启只读保护 (readonly_only=true)")
        if ctype != "mock" and not endpoint:
            safety_errs.append(f"连接器类型 '{ctype}' 需要填写有效的边缘/桥接 Endpoint URL")
        if ctype == "ros2_bridge" and endpoint and not (endpoint.startswith("ws://") or endpoint.startswith("wss://")):
            safety_errs.append("ROS 2 Bridge 地址必须以 ws:// 或 wss:// 开头（例如 ws://192.168.3.10:9090）")
        if any(float(j.get("max_step_rad", 0.15)) > 0.6 for j in raw_joints if isinstance(j, dict)):
            safety_errs.append("单步最大位移 max_step_rad 不可超过 0.6 rad")
        checks.append({
            "id": "safety_defaults",
            "label": "4. 安全默认只读与步长门禁",
            "ok": len(safety_errs) == 0,
            "detail": "; ".join(safety_errs) if safety_errs else (
                "Mock 沙箱模式，默认无 Lease、3s 心跳与免租约急停已就绪"
                if ctype == "mock"
                else f"非 Mock ({ctype}) 已锁定只读镜像模式 (readonly_only=true)"
            ),
        })

        # Gate 5: Exclusivity & Anti-Dual-Writer Guard
        excl_errs: list[str] = []
        rid = str(manifest.get("id") or "").strip()
        if not ROBOT_ID_RE.match(rid):
            excl_errs.append(f"robot_id '{rid}' 须为 3-48 位小写字母/数字/连字符")
        if rid in BUILTIN_ROBOT_IDS:
            excl_errs.append(f"robot_id '{rid}' 与内置机型冲突")
        elif rid in existing_connectors:
            excl_errs.append(f"robot_id '{rid}' 已被已注册机型占用")
        if endpoint:
            for ec in existing_connectors.values():
                if getattr(ec.descriptor, "endpoint", "") == endpoint and not readonly and not ec.descriptor.readonly_only:
                    excl_errs.append(f"Endpoint '{endpoint}' 已被 '{ec.descriptor.id}' 占用写权（禁止双写）")
        if descriptor is not None:
            try:
                PiperLeLabConnector.assert_no_dual_can_writer(
                    [
                        (c.descriptor.robot_type, c.descriptor.connector_type, c.descriptor.readonly_only)
                        for c in existing_connectors.values()
                    ]
                    + [(descriptor.robot_type, descriptor.connector_type, descriptor.readonly_only)]
                )
            except Exception as exc:
                excl_errs.append(str(exc))
        checks.append({
            "id": "exclusivity_check",
            "label": "5. 唯一标识与防双写互斥校验",
            "ok": len(excl_errs) == 0,
            "detail": "; ".join(excl_errs) if excl_errs else f"ID '{rid}' 唯一，无双写/CAN 资源冲突",
        })

        # Gate 6: Runtime 3D & Schema Readiness
        ready_errs: list[str] = []
        if schema_err:
            ready_errs.append(f"Schema 校验失败: {schema_err}")
        if descriptor and descriptor.capabilities.urdf_3d and asset_stats.get("visual_count", 0) <= 0:
            ready_errs.append("开启 3D 模型视图时必须包含有效视觉网格")
        checks.append({
            "id": "runtime_preview_ready",
            "label": "6. 3D 视口渲染与运行态契约就绪",
            "ok": len(ready_errs) == 0,
            "detail": "; ".join(ready_errs) if ready_errs else (
                f"就绪：可立即热注册为 {ctype.upper()} 实例并加载 {asset_stats['visual_count']} 个 {asset_stats['format_label']} 网格"
            ),
        })

        ready_to_commit = all(c["ok"] for c in checks)
        return {
            "checks": checks,
            "ready_to_commit": ready_to_commit,
            "preview_descriptor": descriptor.model_dump() if descriptor else None,
        }

    # ---- Stage 3: Promote & Persist ----

    def commit_staged(
        self,
        stage_id: str,
        manifest: dict[str, Any],
        existing_connectors: dict[str, Any],
    ) -> RobotDescriptor:
        validation = self.validate_staged(stage_id, manifest, existing_connectors)
        if not validation["ready_to_commit"]:
            failed = [f"{c['label']}: {c['detail']}" for c in validation["checks"] if not c["ok"]]
            raise ValueError("入库预检未通过: " + " | ".join(failed))

        rid = str(manifest["id"]).strip()
        stage_dir = (self.staging_dir / stage_id).resolve()
        final_dir = (self.packages_dir / rid).resolve()
        if not str(final_dir).startswith(str(self.packages_dir) + "/"):
            raise ValueError("非法目标目录")
        if final_dir.exists():
            shutil.rmtree(final_dir)
        shutil.move(str(stage_dir), str(final_dir))

        hint = (manifest.get("model") or {}).get("urdf_path") or manifest.get("urdf_path")
        urdf_path = self._find_urdf(final_dir, manifest_hint=hint)
        assert urdf_path is not None
        asset_stats = self._normalize_urdf_assets(final_dir, urdf_path)
        descriptor = self._build_descriptor_from_manifest(
            manifest,
            url_prefix=f"/custom-robots/{rid}",
            asset_stats=asset_stats,
            pkg_dir=final_dir,
        )

        # Save finalized manifest alongside package for reproducibility
        (final_dir / "robot_manifest.json").write_text(
            json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8"
        )

        persisted = [d for d in self.load_persisted_descriptors() if d.id != rid]
        persisted.append(descriptor)
        self._save_registry(persisted)
        return descriptor
