"""Automated tests for real URDF FK/IK, Quintic trajectory planning, and R1 Pro workflow parameter passing."""
import time
from fastapi.testclient import TestClient
from motrix_robot_web.app import create_app


def test_read_only_urdf_fk_ik_and_quintic_plan_across_all_three_robots(tmp_path):
    with TestClient(create_app(data_dir=tmp_path)) as client:
        cases = [
            ("r1pro-mock", "left", [0.06, 0.04, 0.05]),
            ("r1pro-mock", "right", [0.06, -0.04, 0.05]),
            ("piper-dual-mock", "left", [0.05, 0.03, 0.04]),
            ("piper-dual-mock", "right", [0.05, -0.03, 0.04]),
            ("dex-w1-mock", "left", [0.15, -0.15, -0.10]),
        ]
        for robot_id, arm, delta in cases:
            fk_resp = client.post(f"/api/robots/{robot_id}/kinematics/fk", json={"arm": arm})
            assert fk_resp.status_code == 200
            fk0 = fk_resp.json()["fk"][arm]
            target_xyz = [fk0["position"][i] + delta[i] for i in range(3)]

            # 6D Cartesian IK + Trajectory Plan requires ZERO lease and dispatches ZERO motion
            ik_resp = client.post(
                f"/api/robots/{robot_id}/kinematics/ik",
                json={
                    "arm": arm,
                    "xyz_m": target_xyz,
                    "rpy_deg": fk0["rpy_deg"],
                    "plan_trajectory": True,
                    "waypoints": 10,
                    "duration_s": 1.4,
                },
            )
            assert ik_resp.status_code == 200
            body = ik_resp.json()
            assert body["ik"]["ok"] is True
            assert body["ik"]["pos_error_m"] < 0.006
            assert body["plan"]["kind"] == "ik_cartesian"
            assert len(body["plan"]["samples"]) == 10
            assert len(body["plan"]["moving"]) >= 4

            # Verify no lease acquired & robot did not move
            state = client.get(f"/api/robots/{robot_id}/state").json()
            assert state["authority"]["lease_active"] is False

        # Joint-space Quintic trajectory plan with limit clamping
        plan_resp = client.post(
            "/api/robots/dex-w1-mock/kinematics/plan",
            json={"targets": {"SHOULDER_PITCH_L_J1": 0.45, "ELBOW_PITCH_L_J3": -99.0}, "max_vel": 1.0},
        )
        assert plan_resp.status_code == 200
        plan = plan_resp.json()["plan"]
        assert plan["kind"] == "joint_quintic"
        assert "ELBOW_PITCH_L_J3" in plan["clamped"]
        assert len(plan["samples"]) == 25
        assert plan["duration"] >= 0.4


def test_kinematics_rejects_nan_and_inf(tmp_path):
    with TestClient(create_app(data_dir=tmp_path)) as client:
        for path, payload in [
            ("/api/robots/r1pro-mock/kinematics/ik", '{"arm":"left","xyz_m":[0.3,NaN,0.9]}'),
            ("/api/robots/r1pro-mock/kinematics/plan", '{"targets":{"torso_1":Infinity}}'),
        ]:
            resp = client.post(path, content=payload, headers={"content-type": "application/json"})
            assert resp.status_code == 422


def test_workflow_parameter_passing_validation_preview_and_dry_run_context(tmp_path):
    with TestClient(create_app(data_dir=tmp_path)) as client:
        # 1. Seeded R1 Pro FoundationPose -> IK grasp flow passes static dependency check
        wfs = client.get("/api/workflows?robot_id=r1pro-mock").json()["workflows"]
        fp_wf = next((w for w in wfs if w["id"] == "wf-r1-fp-grasp-param-pass"), None)
        assert fp_wf is not None
        assert fp_wf["validation"]["ok"] is True
        contracts = fp_wf["validation"]["step_contracts"]
        assert "workflow_pose" in contracts[0]["produces"]
        assert "workflow_pose" in contracts[1]["requires"]
        assert contracts[1]["overrides"]["offset_xyz_m"] == [0.0, 0.0, 0.1]

        # 2. Missing upstream dependency is rejected by BOTH /validate AND /run (422)
        bad_draft = {
            "robot_id": "r1pro-mock",
            "name": "missing_producer_flow",
            "steps": [
                {"id": "s1", "action_name": "ee_pose_template", "delay_s": 0.0, "duration_s": 1.2, "overrides": {"xyz_source": "workflow_pose"}},
            ],
        }
        val_bad = client.post("/api/workflows/validate", json=bad_draft).json()
        assert val_bad["ok"] is False
        assert "workflow_pose" in val_bad["message"]

        saved_bad = client.post("/api/workflows", json=bad_draft).json()["workflow"]
        run_bad = client.post(f"/api/workflows/{saved_bad['id']}/run", json={"dry_run": True})
        assert run_bad.status_code == 422
        assert run_bad.json()["code"] == "workflow_dependency_error"

        # 3. Read-only 3D workflow trajectory preview produces full samples + context without lease
        prev = client.post("/api/workflows/wf-r1-fp-grasp-param-pass/preview")
        assert prev.status_code == 200
        pdata = prev.json()
        assert pdata["ok"] is True
        assert "workflow_pose" in pdata["workflow_context"]
        assert "workflow_xyz" in pdata["workflow_context"]
        assert pdata["plan"]["kind"] == "workflow_preview"
        assert len(pdata["plan"]["samples"]) >= 20
        assert all(st.get("ik_ok", True) for st in pdata["steps"])

        # 4. Dry-run execution records workflow_context and IK errors in job logs
        run_ok = client.post("/api/workflows/wf-r1-fp-grasp-param-pass/run", json={"dry_run": True})
        assert run_ok.status_code == 200
        jid = run_ok.json()["job"]["job_id"]
        job = None
        for _ in range(40):
            job = client.get(f"/api/jobs/{jid}").json()["job"]
            if job["state"] != "Running":
                break
            time.sleep(0.1)
        assert job["state"] == "Succeeded"
        assert "workflow_pose" in job["workflow_context"]
        log_blob = "\n".join(job["logs"])
        assert "Dry-run context -> produces workflow_xyz" in log_blob
        assert "Dry-run IK: src=workflow_pose" in log_blob


def test_multi_step_workflow_preview_segments_moving_union_and_ee_axial(tmp_path):
    with TestClient(create_app(data_dir=tmp_path)) as client:
        # 1. Verify 4-step FoundationPose -> IK grasp workflow returns contiguous step_segments
        prev4 = client.post("/api/workflows/wf-r1-fp-grasp-param-pass/preview")
        assert prev4.status_code == 200
        d4 = prev4.json()
        plan4 = d4["plan"]
        segs4 = plan4["step_segments"]
        assert len(segs4) == 4
        assert plan4["workflow_id"] == "wf-r1-fp-grasp-param-pass"
        assert plan4["duration"] == plan4["samples"][-1]["t"]
        assert segs4[0]["t_start"] == 0.0
        assert segs4[-1]["t_end"] == plan4["duration"]
        for i in range(1, len(segs4)):
            assert abs(segs4[i]["t_start"] - segs4[i - 1]["t_end"]) < 1e-3

        # 2. Verify 5-step dual-arm box transport workflow preserves full moving union across round-trip steps
        prev5 = client.post("/api/workflows/wf-r1-box-transport/preview")
        assert prev5.status_code == 200
        d5 = prev5.json()
        plan5 = d5["plan"]
        segs5 = plan5["step_segments"]
        assert len(segs5) == 5
        assert plan5["duration"] == plan5["samples"][-1]["t"]
        assert segs5[-1]["t_end"] == plan5["duration"]
        moving_set = set(plan5["moving"])
        assert any(j.startswith("torso_") for j in moving_set), f"expected torso joints in moving union: {moving_set}"
        assert any(j.startswith("left_arm_") for j in moving_set), f"expected left arm in moving union: {moving_set}"
        assert any(j.startswith("right_arm_") for j in moving_set), f"expected right arm in moving union: {moving_set}"

        # 3. Verify ee_axial atomic action inside a workflow produces real preview motion
        wf_axial = client.post(
            "/api/workflows",
            json={
                "robot_id": "r1pro-mock",
                "name": "axial_turn_preview_test",
                "steps": [
                    {"id": "a1", "action_name": "ee_axial_pour_template", "delay_s": 0.0, "duration_s": 1.0, "overrides": {"arm": "right", "delta_xyz_m": [0.05, 0.0, 0.04], "end_angle_deg": 25.0}},
                ],
            },
        )
        assert wf_axial.status_code == 200
        wf_id = wf_axial.json()["workflow"]["id"]
        prev_ax = client.post(f"/api/workflows/{wf_id}/preview")
        assert prev_ax.status_code == 200
        d_ax = prev_ax.json()
        assert len(d_ax["plan"]["step_segments"]) == 1
        assert d_ax["plan"]["step_segments"][0]["has_motion"] is True
        assert len(d_ax["plan"]["moving"]) >= 3

