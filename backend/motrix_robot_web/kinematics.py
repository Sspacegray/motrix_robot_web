from __future__ import annotations

import math
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

from .schemas import RobotDescriptor

PROJECT_ROOT = Path(__file__).resolve().parents[2]
FRONTEND_DIR = PROJECT_ROOT / "frontend"

# Exact base_frame -> arm tip_link mapping per authentic URDF
ROBOT_KINEMATICS_CONFIG: dict[str, dict[str, Any]] = {
    "r1pro": {
        "base_frame": "base_link",
        "arms": {
            "left": {"tip": "left_gripper_link", "group": "left_arm"},
            "right": {"tip": "right_gripper_link", "group": "right_arm"},
        },
        "trail_links": ["left_gripper_link", "right_gripper_link", "zed_link"],
    },
    "piper_dual": {
        "base_frame": "dual_bench_base",
        "arms": {
            "left": {"tip": "left_gripper_base", "group": "left_arm"},
            "right": {"tip": "right_gripper_base", "group": "right_arm"},
        },
        "trail_links": ["left_gripper_base", "right_gripper_base"],
    },
    "dexforce_w1": {
        "base_frame": "base_link",
        "arms": {
            "left": {"tip": "left_ee", "group": "left_arm"},
            "right": {"tip": "right_ee", "group": "right_arm"},
        },
        "trail_links": ["left_ee", "right_ee", "head_pitch_j2_link", "ankle_pitch_link"],
    },
}


# ---- Pure-Python 3D Rigid Math (Zero External Dependencies) ----

def _mat4_identity() -> list[list[float]]:
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _mat4_mul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    out = [[0.0] * 4 for _ in range(4)]
    for i in range(4):
        ai = a[i]
        oi = out[i]
        for k in range(4):
            aik = ai[k]
            if aik == 0.0:
                continue
            bk = b[k]
            oi[0] += aik * bk[0]
            oi[1] += aik * bk[1]
            oi[2] += aik * bk[2]
            oi[3] += aik * bk[3]
    return out


def _rpy_to_rot3(r: float, p: float, y: float) -> list[list[float]]:
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _rot3_to_rpy(R: list[list[float]]) -> tuple[float, float, float]:
    sp = max(-1.0, min(1.0, -R[2][0]))
    pitch = math.asin(sp)
    if abs(math.cos(pitch)) > 1e-6:
        roll = math.atan2(R[2][1], R[2][2])
        yaw = math.atan2(R[1][0], R[0][0])
    else:
        roll = math.atan2(-R[1][2], R[1][1])
        yaw = 0.0
    return roll, pitch, yaw


def _rot3_to_quat_xyzw(R: list[list[float]]) -> list[float]:
    tr = R[0][0] + R[1][1] + R[2][2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2][1] - R[1][2]) / s
        qy = (R[0][2] - R[2][0]) / s
        qz = (R[1][0] - R[0][1]) / s
    elif R[0][0] > R[1][1] and R[0][0] > R[2][2]:
        s = math.sqrt(max(1e-12, 1.0 + R[0][0] - R[1][1] - R[2][2])) * 2.0
        qw = (R[2][1] - R[1][2]) / s
        qx = 0.25 * s
        qy = (R[0][1] + R[1][0]) / s
        qz = (R[0][2] + R[2][0]) / s
    elif R[1][1] > R[2][2]:
        s = math.sqrt(max(1e-12, 1.0 + R[1][1] - R[0][0] - R[2][2])) * 2.0
        qw = (R[0][2] - R[2][0]) / s
        qx = (R[0][1] + R[1][0]) / s
        qy = 0.25 * s
        qz = (R[1][2] + R[2][1]) / s
    else:
        s = math.sqrt(max(1e-12, 1.0 + R[2][2] - R[0][0] - R[1][1])) * 2.0
        qw = (R[1][0] - R[0][1]) / s
        qx = (R[0][2] + R[2][0]) / s
        qy = (R[1][2] + R[2][1]) / s
        qz = 0.25 * s
    return [qx, qy, qz, qw]


def _axis_angle_to_rot3(axis: list[float], theta: float) -> list[list[float]]:
    ax, ay, az = axis
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    if norm < 1e-12 or abs(theta) < 1e-12:
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    ax, ay, az = ax / norm, ay / norm, az / norm
    c = math.cos(theta)
    s = math.sin(theta)
    t = 1.0 - c
    return [
        [t * ax * ax + c, t * ax * ay - s * az, t * ax * az + s * ay],
        [t * ax * ay + s * az, t * ay * ay + c, t * ay * az - s * ax],
        [t * ax * az - s * ay, t * ay * az + s * ax, t * az * az + c],
    ]


def _origin_to_mat4(xyz: list[float], rpy: list[float]) -> list[list[float]]:
    R = _rpy_to_rot3(rpy[0], rpy[1], rpy[2])
    return [
        [R[0][0], R[0][1], R[0][2], xyz[0]],
        [R[1][0], R[1][1], R[1][2], xyz[1]],
        [R[2][0], R[2][1], R[2][2], xyz[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _rot3_mul(A: list[list[float]], B: list[list[float]]) -> list[list[float]]:
    return [
        [sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]


def _rot3_transpose(R: list[list[float]]) -> list[list[float]]:
    return [[R[j][i] for j in range(3)] for i in range(3)]


def _rot3_log(R: list[list[float]]) -> list[float]:
    """SO(3) logarithm: rotation matrix -> axis-angle 3-vector."""
    tr = R[0][0] + R[1][1] + R[2][2]
    cos_theta = max(-1.0, min(1.0, 0.5 * (tr - 1.0)))
    theta = math.acos(cos_theta)
    if theta < 1e-6:
        return [
            0.5 * (R[2][1] - R[1][2]),
            0.5 * (R[0][2] - R[2][0]),
            0.5 * (R[1][0] - R[0][1]),
        ]
    sin_theta = math.sin(theta)
    if abs(sin_theta) < 1e-5:
        # Near pi: extract dominant diagonal axis
        ax = math.sqrt(max(0.0, 0.5 * (R[0][0] + 1.0)))
        ay = math.sqrt(max(0.0, 0.5 * (R[1][1] + 1.0)))
        az = math.sqrt(max(0.0, 0.5 * (R[2][2] + 1.0)))
        if R[2][1] - R[1][2] < 0:
            ax = -ax
        if R[0][2] - R[2][0] < 0:
            ay = -ay
        if R[1][0] - R[0][1] < 0:
            az = -az
        n = math.sqrt(ax * ax + ay * ay + az * az) or 1.0
        return [theta * ax / n, theta * ay / n, theta * az / n]
    k = theta / (2.0 * sin_theta)
    return [
        k * (R[2][1] - R[1][2]),
        k * (R[0][2] - R[2][0]),
        k * (R[1][0] - R[0][1]),
    ]


def _rot3_exp(w: list[float]) -> list[list[float]]:
    theta = math.sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2])
    if theta < 1e-12:
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    return _axis_angle_to_rot3([w[0] / theta, w[1] / theta, w[2] / theta], theta)


def _cross(a: list[float], b: list[float]) -> list[float]:
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def _solve_linear(A: list[list[float]], b: list[float]) -> list[float]:
    """Small dense linear solver with partial pivoting (m <= 6)."""
    m = len(b)
    M = [row[:] + [b[i]] for i, row in enumerate(A)]
    for col in range(m):
        pivot = max(range(col, m), key=lambda r: abs(M[r][col]))
        if abs(M[pivot][col]) < 1e-12:
            continue
        if pivot != col:
            M[col], M[pivot] = M[pivot], M[col]
        piv = M[col][col]
        for r in range(col + 1, m):
            factor = M[r][col] / piv
            if factor == 0.0:
                continue
            for c in range(col, m + 1):
                M[r][c] -= factor * M[col][c]
    x = [0.0] * m
    for i in range(m - 1, -1, -1):
        denom = M[i][i]
        if abs(denom) < 1e-12:
            x[i] = 0.0
            continue
        s = M[i][m] - sum(M[i][j] * x[j] for j in range(i + 1, m))
        x[i] = s / denom
    return x


def _quintic(u: float) -> float:
    x = max(0.0, min(1.0, u))
    return 10.0 * (x**3) - 15.0 * (x**4) + 6.0 * (x**5)


# ---- URDF Inspection & Kinematic Chain Solver ----

def inspect_urdf_file(urdf_path: Path) -> dict[str, Any]:
    """Parse any URDF XML and return links, movable joints, mesh/texture refs, and root/leaf topology."""
    root = ET.parse(urdf_path).getroot()
    robot_name = (root.get("name") or urdf_path.stem).strip()
    links: list[str] = []
    visual_meshes: list[str] = []
    textures: list[str] = []

    for link_el in root.findall("link"):
        lname = (link_el.get("name") or "").strip()
        if lname:
            links.append(lname)
        for vis in link_el.findall("visual"):
            mesh_el = vis.find("geometry/mesh")
            if mesh_el is not None and mesh_el.get("filename"):
                visual_meshes.append(mesh_el.get("filename", "").strip())
            tex_el = vis.find("material/texture")
            if tex_el is not None and tex_el.get("filename"):
                textures.append(tex_el.get("filename", "").strip())

    for mat_el in root.findall("material"):
        tex_el = mat_el.find("texture")
        if tex_el is not None and tex_el.get("filename"):
            textures.append(tex_el.get("filename", "").strip())

    joints: list[dict[str, Any]] = []
    children_set: set[str] = set()
    parents_set: set[str] = set()
    for j in root.findall("joint"):
        jname = (j.get("name") or "").strip()
        jtype = (j.get("type") or "fixed").strip()
        parent_el = j.find("parent")
        child_el = j.find("child")
        parent = (parent_el.get("link") or "").strip() if parent_el is not None else ""
        child = (child_el.get("link") or "").strip() if child_el is not None else ""
        if parent:
            parents_set.add(parent)
        if child:
            children_set.add(child)
        lim_el = j.find("limit")
        lower = float(lim_el.get("lower", -3.14)) if lim_el is not None and lim_el.get("lower") is not None else -3.14
        upper = float(lim_el.get("upper", 3.14)) if lim_el is not None and lim_el.get("upper") is not None else 3.14
        mimic_el = j.find("mimic")
        mimic_joint = mimic_el.get("joint") if mimic_el is not None else None
        joints.append(
            {
                "name": jname,
                "type": jtype,
                "parent": parent,
                "child": child,
                "lower": round(lower, 4),
                "upper": round(upper, 4),
                "mimic": mimic_joint,
            }
        )

    root_links = [l for l in links if l not in children_set] or (links[:1] if links else [])
    leaf_links = [l for l in links if l not in parents_set and l in children_set] or (links[-1:] if links else [])
    movable_joints = [j for j in joints if j["type"] in ("revolute", "continuous", "prismatic") and not j["mimic"]]
    return {
        "robot_name": robot_name,
        "links": links,
        "joints": joints,
        "movable_joints": movable_joints,
        "visual_meshes": visual_meshes,
        "textures": textures,
        "root_links": root_links,
        "leaf_links": leaf_links,
    }


def resolve_urdf_path(urdf_url: str, custom_robots_dir: Path | None = None) -> Path:
    clean = urdf_url.lstrip("/")
    if clean.startswith("custom-robots/") and custom_robots_dir is not None:
        rel = clean[len("custom-robots/"):]
        return (custom_robots_dir / rel).resolve()
    return (FRONTEND_DIR / clean).resolve()


class RobotKinematics:
    """Real URDF-backed FK, 6D DLS IK, and Quintic trajectory planner."""

    def __init__(
        self,
        descriptor: RobotDescriptor,
        urdf_path: Path | None = None,
        custom_robots_dir: Path | None = None,
    ) -> None:
        self.descriptor = descriptor
        if descriptor.kinematics and descriptor.kinematics.arms:
            self.base_frame = descriptor.kinematics.base_frame
            self.arm_cfg = {
                arm: {"tip": spec.tip, "group": spec.group}
                for arm, spec in descriptor.kinematics.arms.items()
            }
            self.trail_links = list(descriptor.kinematics.trail_links)
        else:
            cfg = ROBOT_KINEMATICS_CONFIG.get(descriptor.robot_type)
            if not cfg:
                raise ValueError(f"Unsupported robot_type for kinematics: {descriptor.robot_type}")
            self.base_frame = cfg["base_frame"]
            self.arm_cfg = cfg["arms"]
            self.trail_links = list(cfg["trail_links"])

        if urdf_path is None:
            if not descriptor.urdf_url:
                raise ValueError(f"Robot {descriptor.id} has no urdf_url")
            urdf_path = resolve_urdf_path(descriptor.urdf_url, custom_robots_dir=custom_robots_dir)
        self.urdf_path = urdf_path

        # Map URDF joint name -> descriptor joint spec
        self.urdf_to_desc = {j.urdf_joint: j for j in descriptor.joints if j.urdf_joint}
        self.desc_by_name = {j.name: j for j in descriptor.joints}

        self._child_to_joint: dict[str, dict[str, Any]] = {}
        self._parse_urdf(urdf_path)
        self._chains: dict[str, list[dict[str, Any]]] = {}
        for arm, acfg in self.arm_cfg.items():
            self._chains[arm] = self._build_chain(self.base_frame, acfg["tip"])

    def _parse_urdf(self, path: Path) -> None:
        root = ET.parse(path).getroot()
        for j in root.findall("joint"):
            parent_el = j.find("parent")
            child_el = j.find("child")
            if parent_el is None or child_el is None:
                continue
            xyz = [0.0, 0.0, 0.0]
            rpy = [0.0, 0.0, 0.0]
            ori = j.find("origin")
            if ori is not None:
                if ori.get("xyz"):
                    xyz = [float(v) for v in ori.get("xyz", "").split()]
                if ori.get("rpy"):
                    rpy = [float(v) for v in ori.get("rpy", "").split()]
            axis = [0.0, 0.0, 1.0]
            ax = j.find("axis")
            if ax is not None and ax.get("xyz"):
                axis = [float(v) for v in ax.get("xyz", "").split()]
            lim_el = j.find("limit")
            lower, upper = -math.pi, math.pi
            if lim_el is not None:
                if lim_el.get("lower") is not None:
                    lower = float(lim_el.get("lower", -math.pi))
                if lim_el.get("upper") is not None:
                    upper = float(lim_el.get("upper", math.pi))
            info = {
                "name": j.get("name") or "",
                "type": j.get("type") or "fixed",
                "parent": parent_el.get("link") or "",
                "child": child_el.get("link") or "",
                "xyz": xyz,
                "rpy": rpy,
                "origin_T": _origin_to_mat4(xyz, rpy),
                "axis": axis,
                "lower": lower,
                "upper": upper,
            }
            self._child_to_joint[info["child"]] = info

    def _build_chain(self, base_link: str, tip_link: str) -> list[dict[str, Any]]:
        seq: list[dict[str, Any]] = []
        link = tip_link
        seen: set[str] = set()
        while link != base_link:
            if link in seen:
                raise RuntimeError(f"Cycle in URDF kinematic chain at {link}")
            seen.add(link)
            joint = self._child_to_joint.get(link)
            if joint is None:
                raise RuntimeError(f"Link '{tip_link}' not connected to base '{base_link}'")
            seq.append(joint)
            link = joint["parent"]
        seq.reverse()
        return seq

    def _eval_chain(
        self, arm: str, q_desc: dict[str, float], active_group: str | None = None
    ) -> tuple[list[list[float]], list[dict[str, Any]]]:
        """Evaluate FK along chain, returning (T_tip, active_joint_frames)."""
        T = _mat4_identity()
        active_frames: list[dict[str, Any]] = []
        for seg in self._chains[arm]:
            T = _mat4_mul(T, seg["origin_T"])
            jtype = seg["type"]
            if jtype in ("revolute", "continuous", "prismatic"):
                spec = self.urdf_to_desc.get(seg["name"])
                q_val = 0.0
                eff_scale = 1.0
                if spec is not None:
                    sign = float(getattr(spec, "sign", 1.0) or 1.0)
                    offset = float(getattr(spec, "zero_offset", 0.0) or 0.0)
                    scale = float(spec.urdf_scale)
                    eff_scale = sign * scale
                    q_val = (float(q_desc.get(spec.name, spec.default)) * sign + offset) * scale
                # World-frame joint origin and axis before applying joint rotation
                if active_group is not None and spec is not None and spec.group == active_group and jtype in ("revolute", "continuous"):
                    ax = seg["axis"]
                    wx = (T[0][0] * ax[0] + T[0][1] * ax[1] + T[0][2] * ax[2]) * eff_scale
                    wy = (T[1][0] * ax[0] + T[1][1] * ax[1] + T[1][2] * ax[2]) * eff_scale
                    wz = (T[2][0] * ax[0] + T[2][1] * ax[1] + T[2][2] * ax[2]) * eff_scale
                    active_frames.append(
                        {
                            "desc_name": spec.name,
                            "urdf_name": seg["name"],
                            "lower": spec.lower,
                            "upper": spec.upper,
                            "origin": [T[0][3], T[1][3], T[2][3]],
                            "axis_world": [wx, wy, wz],
                        }
                    )
                if jtype in ("revolute", "continuous"):
                    Rj = _axis_angle_to_rot3(seg["axis"], q_val)
                    Tj = [
                        [Rj[0][0], Rj[0][1], Rj[0][2], 0.0],
                        [Rj[1][0], Rj[1][1], Rj[1][2], 0.0],
                        [Rj[2][0], Rj[2][1], Rj[2][2], 0.0],
                        [0.0, 0.0, 0.0, 1.0],
                    ]
                    T = _mat4_mul(T, Tj)
                elif jtype == "prismatic":
                    ax = seg["axis"]
                    Tj = [
                        [1.0, 0.0, 0.0, ax[0] * q_val],
                        [0.0, 1.0, 0.0, ax[1] * q_val],
                        [0.0, 0.0, 1.0, ax[2] * q_val],
                        [0.0, 0.0, 0.0, 1.0],
                    ]
                    T = _mat4_mul(T, Tj)
        return T, active_frames

    def fk(self, arm: str, joint_positions: dict[str, float]) -> dict[str, Any]:
        if arm not in self._chains:
            raise ValueError(f"Unknown arm '{arm}' for robot {self.descriptor.id}")
        T, _ = self._eval_chain(arm, joint_positions)
        R = [[T[i][j] for j in range(3)] for i in range(3)]
        roll, pitch, yaw = _rot3_to_rpy(R)
        return {
            "arm": arm,
            "base_frame": self.base_frame,
            "tip_link": self.arm_cfg[arm]["tip"],
            "position": [round(T[0][3], 5), round(T[1][3], 5), round(T[2][3], 5)],
            "rpy_rad": [round(roll, 5), round(pitch, 5), round(yaw, 5)],
            "rpy_deg": [round(math.degrees(roll), 2), round(math.degrees(pitch), 2), round(math.degrees(yaw), 2)],
            "orientation_xyzw": [round(v, 5) for v in _rot3_to_quat_xyzw(R)],
        }

    def fk_all_arms(self, joint_positions: dict[str, float]) -> dict[str, dict[str, Any]]:
        return {arm: self.fk(arm, joint_positions) for arm in self._chains}

    def solve_ik(
        self,
        arm: str,
        target_xyz: list[float],
        target_rpy_deg: list[float] | None,
        current_joints: dict[str, float],
        *,
        position_only: bool = False,
        max_iter: int = 120,
        pos_tol_m: float = 0.005,
        rot_tol_deg: float = 2.0,
    ) -> dict[str, Any]:
        if arm not in self._chains:
            raise ValueError(f"Unknown arm '{arm}' for robot {self.descriptor.id}")
        active_group = self.arm_cfg[arm]["group"]
        base_q = {j.name: float(current_joints.get(j.name, j.default)) for j in self.descriptor.joints}

        if target_rpy_deg is None:
            cur_fk = self.fk(arm, base_q)
            target_rpy_deg = cur_fk["rpy_deg"]
        R_tgt = _rpy_to_rot3(
            math.radians(float(target_rpy_deg[0])),
            math.radians(float(target_rpy_deg[1])),
            math.radians(float(target_rpy_deg[2])),
        )
        p_tgt = [float(target_xyz[0]), float(target_xyz[1]), float(target_xyz[2])]

        # Multi-seed list: current pose first (minimum motion!), then neutral default, then mid-workspace seed
        seeds: list[dict[str, float]] = [dict(base_q)]
        default_seed = dict(base_q)
        mid_seed = dict(base_q)
        for j in self.descriptor.joints:
            if j.group == active_group:
                default_seed[j.name] = j.default
                mid_seed[j.name] = 0.5 * (j.lower + j.upper)
        seeds.extend([default_seed, mid_seed])

        best: dict[str, Any] | None = None
        for seed_idx, seed in enumerate(seeds):
            q = dict(seed)
            iters_used = 0
            pos_err = 999.0
            rot_err_deg = 999.0

            for it in range(max_iter):
                iters_used = it + 1
                T, frames = self._eval_chain(arm, q, active_group=active_group)
                n_dof = len(frames)
                if n_dof == 0:
                    break
                p_cur = [T[0][3], T[1][3], T[2][3]]
                R_cur = [[T[i][j] for j in range(3)] for i in range(3)]

                ep = [p_tgt[0] - p_cur[0], p_tgt[1] - p_cur[1], p_tgt[2] - p_cur[2]]
                pos_err = math.sqrt(ep[0] ** 2 + ep[1] ** 2 + ep[2] ** 2)

                R_err = _rot3_mul(R_tgt, _rot3_transpose(R_cur))
                ew = _rot3_log(R_err)
                rot_err_rad = math.sqrt(ew[0] ** 2 + ew[1] ** 2 + ew[2] ** 2)
                rot_err_deg = math.degrees(rot_err_rad)

                if pos_err <= pos_tol_m and (position_only or rot_err_deg <= rot_tol_deg):
                    break

                # Clamp error step for numerical stability
                if pos_err > 0.12:
                    s = 0.12 / pos_err
                    ep = [ep[0] * s, ep[1] * s, ep[2] * s]
                if rot_err_rad > 0.35:
                    s = 0.35 / rot_err_rad
                    ew = [ew[0] * s, ew[1] * s, ew[2] * s]

                # Build analytic geometric Jacobian J (m x n_dof)
                m_dim = 3 if position_only else 6
                J = [[0.0] * n_dof for _ in range(m_dim)]
                for k, fr in enumerate(frames):
                    r_vec = [p_cur[0] - fr["origin"][0], p_cur[1] - fr["origin"][1], p_cur[2] - fr["origin"][2]]
                    jv = _cross(fr["axis_world"], r_vec)
                    J[0][k], J[1][k], J[2][k] = jv[0], jv[1], jv[2]
                    if not position_only:
                        J[3][k], J[4][k], J[5][k] = fr["axis_world"][0], fr["axis_world"][1], fr["axis_world"][2]

                # Weight orientation error gently so position converges smoothly even near singularities
                rot_w = 0.65
                e_vec = ep if position_only else [ep[0], ep[1], ep[2], ew[0] * rot_w, ew[1] * rot_w, ew[2] * rot_w]
                if not position_only:
                    for row in range(3, 6):
                        for k in range(n_dof):
                            J[row][k] *= rot_w

                # Damped Least Squares: (J J^T + lam^2 I) y = e
                lam = 0.04 if pos_err > 0.03 else 0.015
                JJt = [[0.0] * m_dim for _ in range(m_dim)]
                for r in range(m_dim):
                    for c in range(m_dim):
                        JJt[r][c] = sum(J[r][k] * J[c][k] for k in range(n_dof))
                    JJt[r][r] += lam * lam

                y = _solve_linear(JJt, e_vec)
                dq = [sum(J[r][k] * y[r] for r in range(m_dim)) for k in range(n_dof)]

                # Add gentle null-space posture regularization toward current seed on redundant 7-DOF arms
                max_dq = max((abs(v) for v in dq), default=0.0)
                scale = min(1.0, 0.25 / max_dq) if max_dq > 0.25 else 1.0
                for k, fr in enumerate(frames):
                    name = fr["desc_name"]
                    val = q[name] + dq[k] * scale
                    q[name] = max(fr["lower"], min(fr["upper"], val))

            # Final error check
            T_fin, frames_fin = self._eval_chain(arm, q, active_group=active_group)
            p_fin = [T_fin[0][3], T_fin[1][3], T_fin[2][3]]
            R_fin = [[T_fin[i][j] for j in range(3)] for i in range(3)]
            pos_err = math.sqrt(sum((p_tgt[i] - p_fin[i]) ** 2 for i in range(3)))
            ew_fin = _rot3_log(_rot3_mul(R_tgt, _rot3_transpose(R_fin)))
            rot_err_deg = math.degrees(math.sqrt(sum(v * v for v in ew_fin)))
            converged = pos_err <= max(pos_tol_m * 2.5, 0.015) and (position_only or rot_err_deg <= max(rot_tol_deg * 3.0, 8.0))

            # Compute score (lower is better; favor current seed when converged)
            score = pos_err * 100.0 + (0.0 if position_only else rot_err_deg * 0.15) + seed_idx * 0.05
            cand = {
                "ok": converged,
                "arm": arm,
                "base_frame": self.base_frame,
                "tip_link": self.arm_cfg[arm]["tip"],
                "target_xyz": [round(v, 5) for v in p_tgt],
                "target_rpy_deg": [round(float(v), 2) for v in target_rpy_deg],
                "solved_xyz": [round(v, 5) for v in p_fin],
                "solved_rpy_deg": [round(math.degrees(v), 2) for v in _rot3_to_rpy(R_fin)],
                "pos_error_m": round(pos_err, 5),
                "rot_error_deg": round(rot_err_deg, 2),
                "iterations": iters_used,
                "seed_index": seed_idx,
                "position_only": position_only,
                "solved_arm_joints": {fr["desc_name"]: round(q[fr["desc_name"]], 5) for fr in frames_fin},
                "solved_joints": {k: round(v, 5) for k, v in q.items()},
                "_score": score,
            }
            if best is None or cand["_score"] < best["_score"]:
                best = cand
            if converged:
                break

        assert best is not None
        best.pop("_score", None)
        return best

    def build_joint_plan(
        self,
        start_joints: dict[str, float],
        goal_joints: dict[str, float],
        *,
        max_vel: float = 1.2,
        duration_s: float | None = None,
        kind: str = "joint_quintic",
        ik_meta: dict[str, Any] | None = None,
        sample_count: int = 25,
    ) -> dict[str, Any]:
        """Dexforce W1 compatible quintic trajectory preview plan."""
        start: dict[str, float] = {}
        goal: dict[str, float] = {}
        clamped: dict[str, dict[str, float]] = {}
        moving: list[str] = []
        max_delta = 0.0

        for j in self.descriptor.joints:
            s_val = float(start_joints.get(j.name, j.default))
            raw_g = float(goal_joints.get(j.name, s_val))
            c_val = max(j.lower, min(j.upper, raw_g))
            if abs(c_val - raw_g) > 1e-6:
                clamped[j.name] = {"requested": round(raw_g, 4), "clamped": round(c_val, 4)}
            start[j.name] = round(s_val, 5)
            goal[j.name] = round(c_val, 5)
            delta = abs(c_val - s_val)
            if delta > 0.002:
                moving.append(j.name)
                if delta > max_delta:
                    max_delta = delta

        safe_max_vel = max(0.1, min(3.5, float(max_vel)))
        min_dur = max(0.5, (1.875 * max_delta) / safe_max_vel) if max_delta > 1e-4 else 0.6
        dur = max(0.4, float(duration_s)) if duration_s is not None else min_dur
        dur = round(dur, 3)
        peak_vel = round((1.875 * max_delta) / max(0.1, dur), 3)

        n = max(8, int(sample_count))
        samples: list[dict[str, Any]] = []
        for i in range(n):
            u = i / (n - 1)
            s = _quintic(u)
            q_sample = {
                name: round(start[name] + (goal[name] - start[name]) * s, 5)
                for name in start
            }
            samples.append({"t": round(dur * u, 4), "q": q_sample})

        return {
            "plan_id": f"plan-{uuid.uuid4().hex[:10]}",
            "robot_id": self.descriptor.id,
            "kind": kind,
            "duration": dur,
            "start": start,
            "goal": goal,
            "moving": moving,
            "clamped": clamped,
            "max_delta": round(max_delta, 4),
            "max_peak_vel": peak_vel,
            "max_vel": safe_max_vel,
            "trail_links": list(self.trail_links),
            "samples": samples,
            "ik": ik_meta,
        }

    def plan_cartesian_ik(
        self,
        arm: str,
        start_joints: dict[str, float],
        target_xyz: list[float],
        target_rpy_deg: list[float] | None,
        *,
        position_only: bool = False,
        waypoints: int = 12,
        duration_s: float = 1.6,
        max_vel: float = 1.2,
    ) -> dict[str, Any]:
        """R1 Pro style Cartesian IK waypoint interpolation + W1 style scrubbable plan."""
        start_fk = self.fk(arm, start_joints)
        p0 = start_fk["position"]
        R0 = _rpy_to_rot3(*start_fk["rpy_rad"])
        p1 = [float(target_xyz[0]), float(target_xyz[1]), float(target_xyz[2])]
        if target_rpy_deg is None:
            target_rpy_deg = start_fk["rpy_deg"]
        R1 = _rpy_to_rot3(
            math.radians(float(target_rpy_deg[0])),
            math.radians(float(target_rpy_deg[1])),
            math.radians(float(target_rpy_deg[2])),
        )
        w_rel = _rot3_log(_rot3_mul(_rot3_transpose(R0), R1))

        n = max(4, min(32, int(waypoints)))
        cur_q = {j.name: float(start_joints.get(j.name, j.default)) for j in self.descriptor.joints}
        samples: list[dict[str, Any]] = []
        last_ik: dict[str, Any] | None = None
        dur = max(0.5, float(duration_s))

        for i in range(n):
            u = i / (n - 1)
            s = _quintic(u)
            if i == 0:
                samples.append({"t": 0.0, "q": {k: round(v, 5) for k, v in cur_q.items()}})
                continue
            pi = [p0[d] + (p1[d] - p0[d]) * s for d in range(3)]
            Ri = _rot3_mul(R0, _rot3_exp([w_rel[0] * s, w_rel[1] * s, w_rel[2] * s]))
            rpy_i = [math.degrees(v) for v in _rot3_to_rpy(Ri)]
            ik_res = self.solve_ik(arm, pi, rpy_i, cur_q, position_only=position_only, max_iter=80)
            cur_q = dict(ik_res["solved_joints"])
            last_ik = ik_res
            samples.append({"t": round(dur * u, 4), "q": dict(cur_q)})

        assert last_ik is not None
        plan = self.build_joint_plan(
            start_joints,
            cur_q,
            max_vel=max_vel,
            duration_s=dur,
            kind="ik_cartesian",
            ik_meta=last_ik,
            sample_count=len(samples),
        )
        plan["samples"] = samples
        return {"ik": last_ik, "plan": plan}
