import pytest
from motrix_robot_web.connectors.mock import _r1pro_descriptor
from motrix_robot_web.safety import RobotSafetyGate, SafetyError
from motrix_robot_web.schemas import ObservationSnapshot


def test_dual_layer_authority_and_lease():
    desc = _r1pro_descriptor()
    gate = RobotSafetyGate(desc, hb_timeout_s=3.0, lease_ttl_s=120.0)

    # Initially read-only
    auth = gate.snapshot()
    assert auth.task_permission == "operator"
    assert not auth.lease_active
    assert auth.lease_id is None

    # Cannot acquire without confirm_safety
    with pytest.raises(SafetyError) as exc:
        gate.acquire_lease("operator1", confirm_safety=False)
    assert exc.value.code == "confirm_required"

    # Acquire lease successfully
    auth = gate.acquire_lease("operator1", confirm_safety=True)
    assert auth.lease_active
    assert auth.lease_id is not None
    assert auth.operator_name == "operator1"

    # Viewer cannot acquire lease
    gate.set_task_permission("viewer")
    assert not gate.snapshot().lease_active  # downgrading releases lease
    with pytest.raises(SafetyError) as exc:
        gate.acquire_lease("viewer1", task_permission="viewer", confirm_safety=True)
    assert exc.value.code == "insufficient_task_permission"


def test_heartbeat_watchdog_timeout():
    desc = _r1pro_descriptor()
    gate = RobotSafetyGate(desc, hb_timeout_s=1.0)
    gate.acquire_lease("operator1", confirm_safety=True)
    assert gate.snapshot().lease_active

    # Fast forward time beyond hb_timeout_s
    timed_out = []
    reason = gate.check_watchdogs(now=gate._last_hb_mono + 1.5, on_timeout=lambda r: timed_out.append(r))
    assert reason is not None
    assert "heartbeat_timeout" in reason
    assert not gate.snapshot().lease_active
    assert len(timed_out) == 1


def test_estop_bypass_and_lockout():
    desc = _r1pro_descriptor()
    gate = RobotSafetyGate(desc)
    gate.acquire_lease("operator1", confirm_safety=True)
    assert gate.snapshot().lease_active

    # E-Stop can be triggered without lease, releases lease immediately
    auth = gate.trigger_estop(source="test", reason="emergency button pressed")
    assert auth.estop_latched
    assert not auth.lease_active

    # Operation rejected while estop is latched
    with pytest.raises(SafetyError) as exc:
        gate.require_control_authority()
    assert exc.value.code == "estop_latched"

    # Clearing estop
    gate.clear_estop("operator1")
    assert not gate.snapshot().estop_latched


def test_joint_targets_validation():
    desc = _r1pro_descriptor()
    gate = RobotSafetyGate(desc)

    # Without lease -> 403
    obs = ObservationSnapshot(
        robot_id="r1pro-mock",
        timestamp=100.0,
        joint_positions={},
        joint_targets={},
        joint_velocities={},
        joint_efforts={},
    )
    with pytest.raises(SafetyError) as exc:
        gate.validate_joint_targets({"torso_1": 0.0}, obs)
    assert exc.value.status_code == 403

    # Acquire lease
    gate.acquire_lease("operator1", confirm_safety=True)

    # Valid targets
    res = gate.validate_joint_targets({"torso_1": 0.5, "left_arm_1": -1.2}, obs)
    assert res["torso_1"] == 0.5
    assert res["left_arm_1"] == -1.2

    # NaN / Inf rejection
    with pytest.raises(SafetyError) as exc:
        gate.validate_joint_targets({"torso_1": float("nan")}, obs)
    assert exc.value.code == "invalid_number"

    with pytest.raises(SafetyError) as exc:
        gate.validate_joint_targets({"torso_1": float("inf")}, obs)
    assert exc.value.code == "invalid_number"

    # Out of limits rejection (torso_1 upper is 1.83)
    with pytest.raises(SafetyError) as exc:
        gate.validate_joint_targets({"torso_1": 2.5}, obs)
    assert exc.value.code == "joint_limit_exceeded"

    # Unknown joint rejection
    with pytest.raises(SafetyError) as exc:
        gate.validate_joint_targets({"unknown_arm": 0.0}, obs)
    assert exc.value.code == "unknown_joint"


def test_arms_only_teach_enforcement():
    desc = _r1pro_descriptor()
    gate = RobotSafetyGate(desc)
    gate.acquire_lease("operator1", confirm_safety=True)

    # Allowed groups
    gate.validate_teach_request(["left_arm", "right_arm"])

    # Forbidden groups: torso, chassis, head, leg
    with pytest.raises(SafetyError) as exc:
        gate.validate_teach_request(["left_arm", "torso"])
    assert exc.value.code == "teach_arms_only"

    with pytest.raises(SafetyError) as exc:
        gate.validate_teach_request(["chassis"])
    assert exc.value.code == "teach_arms_only"
