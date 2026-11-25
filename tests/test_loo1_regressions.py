"""LOO-1 fail-closed and honesty regressions; no sockets, CAN or hardware clients."""
import pytest
from fastapi.testclient import TestClient
from motrix_robot_web.app import create_app
from motrix_robot_web.connectors import EdgeConnector, R1ProBridgeConnector, PiperLeLabConnector, DexW1ReadOnlyConnector
from motrix_robot_web.connectors.mock import _piper_descriptor, _r1pro_descriptor, build_default_mock_connectors
from motrix_robot_web.safety import RobotSafetyGate, SafetyError


@pytest.fixture
def client(tmp_path):
    with TestClient(create_app(data_dir=tmp_path)) as c:
        yield c


def acquire(client, rid="r1pro-mock"):
    response = client.post(f"/api/robots/{rid}/lease/acquire", json={"confirm_safety": True})
    assert response.status_code == 200
    return response.json()["authority"]


@pytest.mark.parametrize("adapter", [EdgeConnector, R1ProBridgeConnector, PiperLeLabConnector, DexW1ReadOnlyConnector])
def test_adapters_never_fallback_or_claim_delivery(adapter):
    conn = adapter(_piper_descriptor().model_copy(update={"mode": "real", "readonly_only": False}), "http://must-not-connect.invalid")
    assert conn.descriptor.readonly_only
    assert conn.descriptor.integration_status == "unimplemented"
    obs = conn.get_observation()
    assert not obs.connected and obs.measured_hz == 0
    assert obs.telemetry_source == "unavailable" and obs.joint_positions == {}
    for operation, args in [(conn.send_joint_targets, ({"left_arm_1": 0},)), (conn.control_policy, ("start_infer", {})),
                            (conn.control_perception, ("start_yolo", {})), (conn.cancel_navigation, ())]:
        with pytest.raises(SafetyError):
            operation(*args)
    conn.safe_stop("test")
    assert conn.last_local_stop_reason == "test"


@pytest.mark.parametrize("path,body", [
    ("perception/set_head_depth", {"enabled": True}),
    ("perception/start_yolo", {}), ("policy/wizard_step", {"step": "live_armed"}),
    ("policy/switch_checkpoint", {"checkpoint_id": "any"}), ("faults/clear", {}),
])
def test_mutating_routes_require_authority(client, path, body):
    response = client.post(f"/api/robots/r1pro-mock/{path}", json=body)
    assert response.status_code == 403
    assert response.json()["code"] == "lease_required"


def test_heartbeat_cannot_resurrect_expired_lease(client):
    authority = acquire(client)
    gate = client.app.state.ctx.gates["r1pro-mock"]
    gate._last_hb_mono -= 4
    response = client.post("/api/robots/r1pro-mock/lease/heartbeat", json={"lease_id": authority["lease_id"]})
    assert response.status_code in (403, 409)
    assert not gate.snapshot().lease_active
    obs = client.app.state.ctx.connectors["r1pro-mock"].get_observation()
    assert not obs.policy.infer_running


def test_heartbeat_requires_exact_id_and_actions_do_not_renew_it():
    gate = RobotSafetyGate(_r1pro_descriptor())
    authority = gate.acquire_lease("test", confirm_safety=True)
    for value in (None, "", "wrong"):
        with pytest.raises(SafetyError):
            gate.heartbeat(value)
    before = gate._last_hb_mono
    gate.mark_activity()
    assert gate._last_hb_mono == before
    assert gate.heartbeat(authority.lease_id).lease_active


def test_synchronous_timeout_and_idle_stop():
    gate = RobotSafetyGate(_r1pro_descriptor())
    stopped = []
    gate.on_timeout = stopped.append
    gate.acquire_lease("test", confirm_safety=True)
    gate._last_hb_mono -= 4
    with pytest.raises(SafetyError):
        gate.require_control_authority()
    assert len(stopped) == 1 and not gate.snapshot().lease_active
    gate.acquire_lease("test", confirm_safety=True)
    gate._last_action_mono -= 301
    gate.heartbeat  # No action heartbeat side effect.
    with pytest.raises(SafetyError):
        gate.require_control_authority()
    assert "idle_timeout" in stopped[-1]


def test_permission_downgrade_stops_inference_and_new_robot_releases_old(client):
    acquire(client)
    assert client.post("/api/robots/r1pro-mock/policy/start_infer", json={}).status_code == 200
    client.post("/api/robots/r1pro-mock/permission", json={"task_permission": "viewer"})
    ctx = client.app.state.ctx
    assert not ctx.gates["r1pro-mock"].snapshot().lease_active
    assert not ctx.connectors["r1pro-mock"].get_observation().policy.infer_running
    acquire(client)
    acquire(client, "piper-dual-mock")
    assert not ctx.gates["r1pro-mock"].snapshot().lease_active
    assert ctx.gates["piper-dual-mock"].snapshot().lease_active


def test_piper_ordered_mock_flow_and_checkpoint_invalidation():
    piper = build_default_mock_connectors()["piper-dual-mock"]
    for action, payload in [("wizard_step", {"step": "live_armed", "done": True}), ("offline_eval", {}), ("start_infer", {})]:
        with pytest.raises(SafetyError):
            piper.control_policy(action, payload)
    for step in ("homing", "manual_slow_test"):
        piper.control_policy("wizard_step", {"step": step, "done": True})
    with pytest.raises(SafetyError):
        piper.control_policy("wizard_step", {"step": "offline_replay", "done": True})
    result = piper.control_policy("offline_eval", {})["eval"]
    assert result["data_source"] == "synthetic_mock"
    assert result["dataset_parsed"] is False and result["qualified_for_hardware"] is False
    piper.control_policy("wizard_step", {"step": "live_armed", "done": True})
    assert piper.control_policy("start_infer", {})["infer_running"]
    piper.control_policy("stop_infer", {})
    checkpoint = piper.get_observation().policy.active_checkpoint
    piper.control_policy("switch_checkpoint", {"checkpoint_id": checkpoint})
    assert not piper.get_observation().policy.wizard_steps["offline_replay"]
    with pytest.raises(SafetyError):
        piper.control_policy("start_infer", {})


def test_estop_lease_free_latched_and_never_claims_hardware_stop(client):
    response = client.post("/api/robots/r1pro-mock/estop", json={})
    assert response.status_code == 200
    result = response.json()
    assert result["bypass_lease"] and result["authority"]["estop_latched"]
    assert result["remote_stop_confirmed"] is False and result["scope"] == "local_mock"
    assert client.post("/api/robots/r1pro-mock/lease/acquire", json={"confirm_safety": True}).status_code == 409


@pytest.mark.parametrize("path,payload", [
    ("ee/step", {"dx": "NaN"}), ("navigation/chassis", {"vx": "Infinity"}),
    ("navigation/goal", {"x": "NaN", "y": 0}), ("joints/targets", {"targets": {"torso_1": "Infinity"}}),
])
def test_nonfinite_requests_rejected(client, path, payload):
    acquire(client)
    assert client.post(f"/api/robots/r1pro-mock/{path}", json=payload).status_code == 422


def test_capability_rejection_and_local_design_assets(client):
    acquire(client, "piper-dual-mock")
    assert client.post("/api/robots/piper-dual-mock/navigation/mode", json={"mode": "NAVIGATION"}).status_code == 400
    css = client.get("/css/style.css").text.lower()
    assert "#167e83" in css and "#f5f7f8" in css and "#172126" in css
    assert "width: 760px" in css and "height: 64px" in css and "#0b0b0e" in css
    assert "currentSectionCrumb" in client.get("/").text
