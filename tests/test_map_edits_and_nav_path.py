from __future__ import annotations

from pathlib import Path

from fastapi.testclient import TestClient

from motrix_robot_web.app import create_app
from motrix_robot_web.connectors.mock import _r1pro_descriptor
from motrix_robot_web.connectors.ros2_bridge import Ros2BridgeConnector
from motrix_robot_web.schemas import Ros2BridgeConfig


def test_map_edits_persistence_roundtrip(tmp_path: Path) -> None:
    app = create_app(data_dir=tmp_path)
    with TestClient(app) as client:
        # Default overlay
        r0 = client.get("/api/navigation/map-edits", params={"map_id": "fast_slam_20260914"})
        assert r0.status_code == 200
        assert r0.json()["edits"]["denoise_level"] == 1
        assert r0.json()["edits"]["virtual_walls"] == []

        # Save virtual wall + erase stroke + strong denoise
        payload = {
            "denoise_level": 2,
            "virtual_walls": [
                {"id": "vw-1", "x1": 1.0, "y1": -2.0, "x2": 3.5, "y2": -2.0, "thickness_m": 0.16, "label": "虚拟墙 #1"}
            ],
            "erase_strokes": [
                {"id": "er-1", "points": [[2.0, -4.0], [2.4, -4.2]], "radius_m": 0.35}
            ],
        }
        r1 = client.put("/api/navigation/map-edits/fast_slam_20260914", json=payload)
        assert r1.status_code == 200
        saved = r1.json()["edits"]
        assert saved["denoise_level"] == 2
        assert len(saved["virtual_walls"]) == 1
        assert len(saved["erase_strokes"]) == 1

    # Reload new app instance on same data_dir to prove disk persistence
    app2 = create_app(data_dir=tmp_path)
    with TestClient(app2) as client2:
        r2 = client2.get("/api/navigation/map-edits", params={"map_id": "fast_slam_20260914"})
        assert r2.status_code == 200
        loaded = r2.json()["edits"]
        assert loaded["denoise_level"] == 2
        assert loaded["virtual_walls"][0]["id"] == "vw-1"
        assert loaded["erase_strokes"][0]["id"] == "er-1"

        # Reset back to raw map
        r3 = client2.delete("/api/navigation/map-edits/fast_slam_20260914")
        assert r3.status_code == 200
        assert r3.json()["edits"]["virtual_walls"] == []
        assert r3.json()["edits"]["denoise_level"] == 1


def test_mock_nav_goal_follows_astar_polyline_waypoints(tmp_path: Path) -> None:
    app = create_app(data_dir=tmp_path)
    with TestClient(app) as client:
        assert client.post(
            "/api/robots/r1pro-mock/lease/acquire",
            json={"operator_name": "tester@motphys", "task_permission": "operator", "confirm_safety": True},
        ).status_code == 200
        assert client.post("/api/robots/r1pro-mock/navigation/mode", json={"mode": "NAVIGATION"}).status_code == 200

        # L-shaped corridor polyline (0,0) -> (0,-3) -> (3,-3)
        poly = [[0.0, 0.0], [0.0, -1.5], [0.0, -3.0], [1.5, -3.0], [3.0, -3.0]]
        res = client.post(
            "/api/robots/r1pro-mock/navigation/goal",
            json={"x": 3.0, "y": -3.0, "yaw": 0.0, "path_points": poly},
        )
        assert res.status_code == 200
        body = res.json()
        assert body["path_source"] == "local_astar"
        assert len(body["path_points"]) == 5

        # Advance connector manually and confirm robot tracks down -Y first instead of cutting diagonally
        conn, _ = app.state.ctx.get_robot("r1pro-mock")
        conn.step(0.6)
        pose = conn.get_observation().navigation.pose_map
        assert pose["y"] < -0.5
        assert abs(pose["x"]) < 0.25, f"expected corridor tracking along x≈0 first, got {pose}"


def test_ros2_bridge_parses_live_nav2_plan_topic() -> None:
    desc = _r1pro_descriptor()
    conn = Ros2BridgeConnector(desc, ros2_config=Ros2BridgeConfig(nav_plan_topic="/plan"))
    conn._handle_ros_message(
        "/plan",
        {
            "header": {"frame_id": "map"},
            "poses": [
                {"pose": {"position": {"x": 0.0, "y": 0.0, "z": 0.0}}},
                {"pose": {"position": {"x": 1.2, "y": -0.5, "z": 0.0}}},
                {"pose": {"position": {"x": 2.4, "y": -1.8, "z": 0.0}}},
            ],
        },
    )
    obs = conn.get_observation()
    assert obs.navigation.path_source == "nav2_plan"
    assert obs.navigation.path_points == [[0.0, 0.0], [1.2, -0.5], [2.4, -1.8]]
