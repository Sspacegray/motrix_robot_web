from unittest.mock import patch
from fastapi.testclient import TestClient
from motrix_robot_web.app import create_app


def test_r1_map_metadata_exposes_fast_slam_hd_mx_map_and_robust_slam_without_lease(tmp_path):
    with TestClient(create_app(data_dir=tmp_path)) as client:
        payload = client.get("/api/robots/r1pro-mock/navigation/map").json()
        assert payload["supported"] is True
        assert payload["default_map_id"] == "fast_slam_20260914"
        assert payload["kind"] == "recorded_slam_map"
        assert payload["image_url"] == "/assets/maps/fast_slam_20260914/bag_20260914_map.png"
        assert payload["yaml"]["resolution"] == 0.05
        assert payload["pointcloud"]["live"] is False
        assert payload["pointcloud"]["source"] == "recorded_lidar_pcd"
        assert payload["pointcloud"]["pcd_url"] == "/assets/maps/fast_slam_20260914/virtual_machine_map.pcd"
        assert payload["pointcloud"]["point_count"] == 981005
        assert "离线静态地图" in payload["pointcloud"]["label"]

        # Map catalog includes Fast-SLAM Plus HD map, pc10 mx_map, and Robust-SLAM comparison map
        map_ids = [m["id"] for m in payload["maps"]]
        assert map_ids == ["fast_slam_20260914", "mx_map", "r1pro_20260918"]
        mx = payload["maps"][1]
        assert mx["image_url"] == "/assets/mx_map.png"
        assert mx["pointcloud"]["source"] == "mock_from_occupancy"
        assert mx["pointcloud"]["live"] is False

        # All real assets serve HTTP 200
        pcd_resp = client.get("/assets/maps/fast_slam_20260914/virtual_machine_map.pcd")
        assert pcd_resp.status_code == 200
        assert b"POINTS 981005" in pcd_resp.content[:512]
        assert client.get("/assets/maps/fast_slam_20260914/bag_20260914_map.png").status_code == 200
        assert client.get("/assets/maps/fast_slam_20260914/bag_20260914_map.yaml").status_code == 200
        assert client.get("/assets/mx_map.png").status_code == 200
        assert client.get("/assets/mx_map.yaml").status_code == 200
        assert client.get("/assets/maps/r1pro_20260918/r1pro_map.pcd").status_code == 200

        # Reading maps never acquires lease
        state = client.get("/api/robots/r1pro-mock/state").json()
        assert state["authority"]["lease_active"] is False


def test_w1_and_piper_maps_are_disabled_without_substituting_r1_assets(tmp_path):
    with TestClient(create_app(data_dir=tmp_path)) as client:
        for robot in ("dex-w1-mock", "piper-dual-mock"):
            payload = client.get(f"/api/robots/{robot}/navigation/map").json()
            assert payload["supported"] is False
            assert payload["pointcloud"]["available"] is False
            assert payload["pointcloud"]["live"] is False
            assert payload["pointcloud"]["source"] is None
            assert payload["maps"] == []
            assert "不使用其他机型地图替代" in payload["reason"]


def test_map_metadata_does_not_dispatch_navigation_commands(tmp_path):
    app = create_app(data_dir=tmp_path)
    connector = app.state.ctx.connectors["r1pro-mock"]
    with TestClient(app) as client, patch.object(connector, "send_navigation_goal", side_effect=AssertionError("map read dispatched a goal")) as goal:
        assert client.get("/api/robots/r1pro-mock/navigation/map").status_code == 200
        assert goal.call_count == 0


def test_navigation_waypoints_crud(tmp_path):
    with TestClient(create_app(data_dir=tmp_path)) as client:
        seeded = client.get("/api/navigation/waypoints?robot_id=r1pro-mock&map_id=r1pro_20260918").json()["waypoints"]
        assert len(seeded) >= 3

        created = client.post(
            "/api/navigation/waypoints",
            json={"name": "测试充电点", "robot_id": "r1pro-mock", "map_id": "r1pro_20260918", "x": 2.5, "y": -1.2, "yaw": 0.5},
        )
        assert created.status_code == 200
        wp = created.json()["waypoint"]
        assert wp["name"] == "测试充电点"

        listed = client.get("/api/navigation/waypoints?robot_id=r1pro-mock&map_id=r1pro_20260918").json()["waypoints"]
        assert any(w["id"] == wp["id"] for w in listed)

        deleted = client.delete(f"/api/navigation/waypoints/{wp['id']}")
        assert deleted.status_code == 200 and deleted.json()["deleted"] is True
        assert client.delete(f"/api/navigation/waypoints/{wp['id']}").status_code == 404


def test_single_atomic_action_execute_enforces_lease_and_creates_job(tmp_path):
    with TestClient(create_app(data_dir=tmp_path)) as client:
        actions = client.get("/api/actions?robot_id=r1pro-mock").json()["actions"]
        assert len(actions) > 0
        act_id = actions[0]["id"]

        # Live execution without lease must fail 403
        no_lease = client.post(f"/api/actions/{act_id}/execute", json={"dry_run": False})
        assert no_lease.status_code == 403

        # Dry-run succeeds without lease and creates an auditable job
        dry = client.post(f"/api/actions/{act_id}/execute", json={"dry_run": True})
        assert dry.status_code == 200
        dry_job = dry.json()["job"]
        assert dry_job["job_id"] and dry_job["lease_id"] is None
        assert actions[0]["name"] in dry_job["flow_name"]

        # Acquire lease -> live single-action execution succeeds
        assert client.post("/api/robots/r1pro-mock/lease/acquire", json={"operator_name": "op1", "confirm_safety": True}).status_code == 200
        live = client.post(f"/api/actions/{act_id}/execute", json={"dry_run": False})
        assert live.status_code == 200
        live_job = live.json()["job"]
        assert live_job["job_id"] and live_job["lease_id"] is not None
