from unittest.mock import patch
from fastapi.testclient import TestClient
from motrix_robot_web.app import create_app


def test_r1_map_metadata_is_mock_occupancy_and_does_not_require_lease(tmp_path):
    with TestClient(create_app(data_dir=tmp_path)) as client:
        payload = client.get("/api/robots/r1pro-mock/navigation/map").json()
        assert payload["supported"] is True
        assert payload["kind"] == "occupancy_grid"
        assert payload["image_url"] == "/assets/mx_map.png"
        assert payload["pointcloud"]["live"] is False
        assert payload["pointcloud"]["source"] == "mock_from_occupancy"
        assert "不是真实激光雷达" in payload["pointcloud"]["label"]
        assert client.get("/assets/mx_map.png").status_code == 200
        assert client.get("/assets/mx_map.yaml").status_code == 200
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
            assert "不使用其他机型地图替代" in payload["reason"]


def test_map_metadata_does_not_dispatch_navigation_commands(tmp_path):
    app = create_app(data_dir=tmp_path)
    connector = app.state.ctx.connectors["r1pro-mock"]
    with TestClient(app) as client, patch.object(connector, "send_navigation_goal", side_effect=AssertionError("map read dispatched a goal")) as goal:
        assert client.get("/api/robots/r1pro-mock/navigation/map").status_code == 200
        assert goal.call_count == 0
