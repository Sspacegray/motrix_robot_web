"""LOO-1 safety regressions: local Mock only; never contact a robot endpoint."""
import pytest
from fastapi.testclient import TestClient
from motrix_robot_web.app import create_app
from motrix_robot_web.safety import RobotSafetyGate, SafetyError
from motrix_robot_web.connectors.mock import _r1pro_descriptor

@pytest.fixture
def client(tmp_path):
    with TestClient(create_app(data_dir=tmp_path)) as c:
        yield c

def acquire(client, rid="r1pro-mock"):
    r = client.post(f"/api/robots/{rid}/lease/acquire", json={"confirm_safety": True})
    assert r.status_code == 200
    return r.json()["authority"]

@pytest.mark.parametrize("path,body", [
    ("perception/set_head_depth", {"enabled":True}),
    ("perception/start_yolo", {}),
    ("perception/latch_yolo", {}),
    ("policy/set_config", {"speed_scale":1}),
    ("policy/wizard_step", {"step":"homing"}),
    ("policy/tag_episode", {}),
    ("faults/overheat", {}),
    ("navigation/spin", {"active": True}),
])
def test_mutation_needs_lease(client, path, body):
    r = client.post(f"/api/robots/r1pro-mock/{path}", json=body)
    assert r.status_code == 403
    assert r.json()["code"] == "lease_required"

def test_switch_releases_other_robot(client):
    acquire(client)
    acquire(client, "piper-dual-mock")
    old = client.get("/api/robots/r1pro-mock/state").json()
    assert not old["authority"]["lease_active"]
    assert not old["observation"]["policy"]["infer_running"]

def test_no_silent_lease_takeover(client):
    acquire(client)
    assert client.post("/api/robots/r1pro-mock/lease/acquire", json={"confirm_safety":True}).status_code == 409

def test_heartbeat_cannot_resurrect_expired_lease():
    gate = RobotSafetyGate(_r1pro_descriptor())
    auth = gate.acquire_lease("test", confirm_safety=True)
    calls = []
    gate.on_timeout = calls.append
    gate._last_hb_mono -= 4
    with pytest.raises(SafetyError, match="过期"):
        gate.heartbeat(auth.lease_id)
    assert len(calls) == 1
    assert not gate.snapshot().lease_active

def test_idle_timeout_calls_stop():
    gate = RobotSafetyGate(_r1pro_descriptor())
    gate.acquire_lease("test", confirm_safety=True)
    gate._last_action_mono -= 301
    calls = []
    gate.on_timeout = calls.append
    assert "idle_timeout" in gate.check_watchdogs()
    assert len(calls) == 1
    assert not gate.snapshot().lease_active

def test_estop_is_lease_free_latched_and_clear_not_rearm(client):
    base = "/api/robots/r1pro-mock"
    r = client.post(base+"/estop", json={})
    assert r.status_code == 200
    assert r.json()["remote_stop_confirmed"] is False
    assert client.post(base+"/lease/acquire", json={"confirm_safety": True}).status_code == 409
    assert client.post(base+"/estop/clear", json={}).status_code == 200
    assert not client.get(base+"/state").json()["authority"]["lease_active"]

def test_viewer_cannot_clear_estop(client):
    base = "/api/robots/r1pro-mock"
    client.post(base+"/permission", json={"task_permission":"viewer"})
    client.post(base+"/estop", json={})
    assert client.post(base+"/estop/clear", json={}).status_code == 403

def test_policy_wizard_no_skip_and_synthetic_eval(client):
    base = "/api/robots/piper-dual-mock/policy/"
    acquire(client, "piper-dual-mock")
    r = client.post(base+"wizard_step", json={"step":"live_armed"})
    assert r.status_code == 409
    for step in ("self_check", "homing", "manual_slow_test"):
        assert client.post(base+"wizard_step", json={"step":step}).status_code == 200
    assert client.post(base+"wizard_step", json={"step":"offline_replay"}).status_code == 409
    result = client.post(base+"offline_eval", json={}).json()["eval"]
    assert result["data_source"] == "synthetic_mock"
    assert result["dataset_parsed"] is False
    assert result["qualified_for_hardware"] is False

def test_configured_real_connector_fail_closed(tmp_path):
    config = tmp_path / "robots.yaml"
    # Reserved test hostname: constructing this adapter must never do IO.
    config.write_text("robots:\n  - id: edge-test\n    connector: edge\n    mode: real\n    readonly_only: false\n    base_url: http://robot.invalid\n")
    with TestClient(create_app(data_dir=tmp_path / "data", config_path=config)) as c:
        state = c.get("/api/robots/edge-test/state").json()
        assert state["descriptor"]["integration_status"] == "unimplemented"
        assert state["observation"]["telemetry_source"] == "unavailable"
        assert not state["observation"]["connected"]
        assert c.post("/api/robots/edge-test/lease/acquire", json={"confirm_safety": True}).status_code == 403
