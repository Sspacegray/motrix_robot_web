from __future__ import annotations

import io
import zipfile
from pathlib import Path

from fastapi.testclient import TestClient

from motrix_robot_web.app import create_app
from motrix_robot_web.robot_importer import build_sample_robot_package_zip


def _make_raw_urdf_only_zip() -> bytes:
    """Take the sample package and strip robot_manifest.json to test zero-manifest auto-discovery."""
    src = build_sample_robot_package_zip("raw-arm-demo")
    out = io.BytesIO()
    with zipfile.ZipFile(io.BytesIO(src), "r") as zin, zipfile.ZipFile(out, "w") as zout:
        for info in zin.infolist():
            if info.filename == "robot_manifest.json":
                continue
            zout.writestr(info, zin.read(info.filename))
    return out.getvalue()


def _make_zip_slip_archive() -> bytes:
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as zf:
        zf.writestr("../escape.txt", "pwned")
    return buf.getvalue()


def _make_missing_mesh_zip() -> bytes:
    buf = io.BytesIO()
    urdf = """<?xml version="1.0"?>
<robot name="broken_bot">
  <link name="base_link">
    <visual><geometry><mesh filename="meshes/does_not_exist.stl"/></geometry></visual>
  </link>
  <link name="tip_link"/>
  <joint name="j1" type="revolute">
    <parent link="base_link"/><child link="tip_link"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-1.0" upper="1.0" effort="10" velocity="1.0"/>
  </joint>
</robot>
"""
    with zipfile.ZipFile(buf, "w") as zf:
        zf.writestr("urdf/robot.urdf", urdf)
    return buf.getvalue()


def test_template_download_and_sample_import_full_lifecycle(tmp_path: Path) -> None:
    app = create_app(data_dir=tmp_path)
    with TestClient(app) as client:
        # 1. Download template .zip
        tpl = client.get("/api/robots/import/template")
        assert tpl.status_code == 200
        assert tpl.headers["content-type"] == "application/zip"
        with zipfile.ZipFile(io.BytesIO(tpl.content), "r") as zf:
            names = set(zf.namelist())
            assert "robot_manifest.json" in names
            assert "urdf/robot.urdf" in names
            assert "urdf/meshes/base.stl" in names

        # 2. Inspect sample package (?sample=true) -> all 6 preflight gates PASS
        insp = client.post("/api/robots/import/inspect?sample=true")
        assert insp.status_code == 200, insp.text
        body = insp.json()
        assert body["ready_to_commit"] is True
        assert len(body["checks"]) == 6
        assert all(c["ok"] for c in body["checks"]), body["checks"]
        assert body["urdf_summary"]["visual_count"] == 9
        stage_id = body["stage_id"]
        manifest = body["manifest"]
        manifest["id"] = "cobot-arm6-test"

        # 3. Verify Gate 4 blocks non-mock with readonly_only=False
        bad_manifest = {**manifest, "connector_type": "edge", "endpoint": "http://10.0.0.5:8080/v1", "readonly_only": False}
        bad_val = client.post("/api/robots/import/validate", json={"stage_id": stage_id, "manifest": bad_manifest})
        assert bad_val.status_code == 200
        bad_checks = {c["id"]: c for c in bad_val.json()["checks"]}
        assert bad_checks["safety_defaults"]["ok"] is False
        assert bad_val.json()["ready_to_commit"] is False

        # 4. Flip a joint sign=-1.0 to verify signed SDK<->URDF FK/IK alignment passes
        signed_manifest = dict(manifest)
        signed_joints = [dict(j) for j in manifest["joints"]]
        signed_joints[0]["sign"] = -1.0
        signed_manifest["joints"] = signed_joints
        good_val = client.post("/api/robots/import/validate", json={"stage_id": stage_id, "manifest": signed_manifest})
        assert good_val.status_code == 200
        assert good_val.json()["ready_to_commit"] is True, good_val.json()["checks"]

        # 5. Commit -> hot-registers live robot with static URDF + STL + FK/IK + Quintic plan!
        commit = client.post("/api/robots/import/commit", json={"stage_id": stage_id, "manifest": signed_manifest})
        assert commit.status_code == 200, commit.text
        desc = commit.json()["robot"]
        rid = desc["id"]
        assert rid == "cobot-arm6-test"
        assert desc["is_custom"] is True
        assert desc["urdf_url"] == "/custom-robots/cobot-arm6-test/urdf/robot.urdf"

        # Robot appears in /api/robots and serves static URDF & STL with HTTP 200
        robots = client.get("/api/robots").json()["robots"]
        assert any(r["id"] == rid and r["is_custom"] for r in robots)
        assert client.get(desc["urdf_url"]).status_code == 200
        assert client.get("/custom-robots/cobot-arm6-test/urdf/meshes/base.stl").status_code == 200

        # Real URDF FK & 6D IK + trajectory plan work immediately on newly imported robot!
        fk_res = client.post(f"/api/robots/{rid}/kinematics/fk", json={"arm": "left"})
        assert fk_res.status_code == 200, fk_res.text
        p0 = fk_res.json()["fk"]["left"]["position"]
        rpy0 = fk_res.json()["fk"]["left"]["rpy_deg"]

        ik_res = client.post(
            f"/api/robots/{rid}/kinematics/ik",
            json={
                "arm": "left",
                "xyz_m": [p0[0] + 0.05, p0[1] + 0.04, p0[2] - 0.03],
                "rpy_deg": rpy0,
                "plan_trajectory": True,
                "duration_s": 1.0,
            },
        )
        assert ik_res.status_code == 200, ik_res.text
        assert ik_res.json()["ik"]["pos_error_m"] < 0.005
        assert len(ik_res.json()["plan"]["samples"]) >= 10

        # Built-in robot deletion must be rejected
        assert client.delete("/api/robots/r1pro-mock").status_code == 400

    # 6. Restart app with same data_dir -> persisted custom robot reloads automatically!
    app2 = create_app(data_dir=tmp_path)
    with TestClient(app2) as client2:
        robots2 = client2.get("/api/robots").json()["robots"]
        assert any(r["id"] == "cobot-arm6-test" and r["is_custom"] for r in robots2)
        assert client2.post("/api/robots/cobot-arm6-test/kinematics/fk", json={"arm": "left"}).status_code == 200

        # 7. Delete custom robot cleanly
        del_res = client2.delete("/api/robots/cobot-arm6-test")
        assert del_res.status_code == 200
        assert del_res.json()["deleted"] is True
        robots3 = client2.get("/api/robots").json()["robots"]
        assert not any(r["id"] == "cobot-arm6-test" for r in robots3)


def test_raw_urdf_only_zip_auto_discovers_joints_and_kinematics(tmp_path: Path) -> None:
    app = create_app(data_dir=tmp_path)
    with TestClient(app) as client:
        raw_zip = _make_raw_urdf_only_zip()
        res = client.post(
            "/api/robots/import/inspect",
            content=raw_zip,
            headers={"Content-Type": "application/zip"},
        )
        assert res.status_code == 200, res.text
        data = res.json()
        assert data["manifest_source"] == "auto_urdf"
        assert len(data["manifest"]["joints"]) == 7  # 6 revolute + 1 prismatic (mimic excluded)
        assert data["manifest"]["kinematics"]["base_frame"] == "base_link"
        assert data["ready_to_commit"] is True, data["checks"]


def test_import_security_and_preflight_rejections(tmp_path: Path) -> None:
    app = create_app(data_dir=tmp_path)
    with TestClient(app) as client:
        # 1. Zip-Slip attack blocked with 400
        slip = client.post("/api/robots/import/inspect", content=_make_zip_slip_archive())
        assert slip.status_code == 400
        assert "安全拦截" in slip.json()["detail"]

        # 2. Missing mesh blocked in Gate 1 (cannot commit)
        broken = client.post("/api/robots/import/inspect", content=_make_missing_mesh_zip())
        assert broken.status_code == 200
        bdata = broken.json()
        assert bdata["ready_to_commit"] is False
        checks = {c["id"]: c for c in bdata["checks"]}
        assert checks["assets_integrity"]["ok"] is False

        commit_fail = client.post(
            "/api/robots/import/commit",
            json={"stage_id": bdata["stage_id"], "manifest": bdata["manifest"]},
        )
        assert commit_fail.status_code == 400

        # 3. Static mount /custom-robots must NEVER expose internal registry.json or _staging/*
        assert client.get("/custom-robots/registry.json").status_code == 404
        assert client.get(f"/custom-robots/_staging/{bdata['stage_id']}/urdf/robot.urdf").status_code == 404

        # 4. Stale staging pruning cleans abandoned staging folders
        pruned = app.state.ctx.importer.prune_stale_staging(max_age_s=0.0)
        assert pruned >= 1
        assert not (app.state.ctx.importer.staging_dir / bdata["stage_id"]).exists()
