import pytest
from motrix_robot_web.connectors.dex_w1 import DexW1ReadOnlyConnector
from motrix_robot_web.connectors.edge import EdgeConnector
from motrix_robot_web.connectors.mock import build_default_mock_connectors
from motrix_robot_web.connectors.piper_lelab import PiperLeLabConnector
from motrix_robot_web.motrix_bridge import MotrixReadOnlyBridge
from motrix_robot_web.safety import SafetyError


def test_mock_descriptors_and_capabilities():
    conns = build_default_mock_connectors()
    assert set(conns.keys()) == {"r1pro-mock", "piper-dual-mock", "dex-w1-mock"}

    r1 = conns["r1pro-mock"]
    assert len(r1.descriptor.joints) == 20  # 4 torso + 7*2 arm + 2 gripper
    assert r1.descriptor.capabilities.navigation is True
    assert r1.descriptor.capabilities.yolo_foundation_pose is True
    assert r1.descriptor.capabilities.head_depth is True

    piper = conns["piper-dual-mock"]
    assert len(piper.descriptor.joints) == 14  # 6*2 arm + 2 gripper
    assert piper.descriptor.capabilities.navigation is False  # Piper has no mobile base
    assert piper.descriptor.capabilities.piper_safety_wizard is True
    assert piper.descriptor.capabilities.policy_inference is True

    w1 = conns["dex-w1-mock"]
    assert len(w1.descriptor.joints) == 20
    assert w1.descriptor.capabilities.navigation is False
    assert w1.descriptor.capabilities.motrix_mirror is True


def test_strict_three_step_perception_gate():
    conns = build_default_mock_connectors()
    r1 = conns["r1pro-mock"]

    # Step 0: Pre-requisite for head_left_rgb: head_depth must be enabled and ready
    with pytest.raises(SafetyError) as exc:
        r1.control_perception("start_yolo", {"camera": "head_left_rgb", "prompt": "bottle"})
    assert exc.value.code == "head_depth_not_ready"

    # Turn on head depth
    r1.control_perception("set_head_depth", {"enabled": True})
    obs = r1.get_observation()
    assert obs.perception.head_depth_ready is True

    # Attempt to jump straight to Step 3 (FoundationPose) without Step 1 & 2 -> Reject!
    with pytest.raises(SafetyError) as exc:
        r1.control_perception("start_foundation_pose", {"camera": "head_left_rgb", "classes": "bottle"})
    assert exc.value.code == "gate_yolo_required"

    # Step 1: Start YOLO
    r1.control_perception("start_yolo", {"camera": "head_left_rgb", "prompt": "bottle"})
    obs = r1.get_observation()
    assert obs.perception.yolo_running is True
    assert len(obs.perception.yolo_detections) > 0

    # Attempt to start FoundationPose without Step 2 (latching) -> Reject!
    with pytest.raises(SafetyError) as exc:
        r1.control_perception("start_foundation_pose", {"camera": "head_left_rgb", "classes": "bottle"})
    assert exc.value.code == "gate_latch_required"

    # Step 2: Latch best segmentation
    r1.control_perception("latch_yolo", {})
    obs = r1.get_observation()
    assert obs.perception.latched_mask is not None
    assert obs.perception.latched_mask["verified"] is True

    # Step 3: Start FoundationPose
    r1.control_perception("start_foundation_pose", {"camera": "head_left_rgb", "classes": "bottle"})
    obs = r1.get_observation()
    assert obs.perception.foundation_pose_running is True
    assert obs.perception.foundation_pose_phase in ("registering", "waiting")

    # In registration phase, trying to stop YOLO is blocked by safety gate
    with pytest.raises(SafetyError) as exc:
        r1.control_perception("stop_yolo", {})
    assert exc.value.code == "yolo_locked_during_registration"

    # Step simulation forward to allow registration to finish
    r1.step(1.0)
    obs = r1.get_observation()
    assert obs.perception.foundation_pose_phase == "tracking"

    # Now YOLO can be safely stopped
    r1.control_perception("stop_yolo", {})
    assert r1.get_observation().perception.yolo_running is False


def test_piper_checkpoint_layout_contract():
    conns = build_default_mock_connectors()
    piper = conns["piper-dual-mock"]

    # Incompatible checkpoint (20-DOF for 14-DOF robot) should be rejected
    with pytest.raises(SafetyError) as exc:
        piper.control_policy("switch_checkpoint", {"checkpoint_id": "r1pro_v2_upper_body_20dof/050000"})
    assert exc.value.code == "action_layout_mismatch"

    # Compatible 14-DOF checkpoint
    res = piper.control_policy("switch_checkpoint", {"checkpoint_id": "pi05_real_noframe_136_30k/030000"})
    assert res["ok"] is True
    assert res["active_checkpoint"] == "pi05_real_noframe_136_30k/030000"

    # Offline evaluation produces trajectory comparison and safety contract notes
    eval_res = piper.control_policy("offline_eval", {"dataset": "set_the_plates_real_all_mcap_20261119_v1"})
    assert eval_res["ok"] is True
    assert "rmse_rad" in eval_res["eval"]
    assert "safety_contract" in eval_res["eval"]


def test_single_can_writer_invariant():
    # If both motrix-edge and piper_lelab are configured as writable on Piper -> MUST FAIL CLOSED
    with pytest.raises(SafetyError) as exc:
        PiperLeLabConnector.assert_no_dual_can_writer([
            ("piper_dual", "edge", False),
            ("piper_dual", "piper_lelab", False),
        ])
    assert exc.value.code == "dual_can_writer_forbidden"

    # If one is read-only -> Allowed
    PiperLeLabConnector.assert_no_dual_can_writer([
        ("piper_dual", "edge", False),
        ("piper_dual", "piper_lelab", True),
    ])


def test_dex_w1_strictly_read_only():
    conns = build_default_mock_connectors()
    w1_desc = conns["dex-w1-mock"].descriptor
    w1_conn = DexW1ReadOnlyConnector(w1_desc, "http://10.3.130.154:8090")

    # Read-only telemetry works
    obs = w1_conn.get_observation()
    assert obs.robot_id == "dex-w1-mock"

    # Any write control returns 403
    with pytest.raises(SafetyError) as exc:
        w1_conn.send_joint_targets({"WAIST_YAW": 0.1})
    assert exc.value.status_code == 403
    assert exc.value.code == "dex_w1_readonly_only"


def test_motrix_bridge_driver_isolation():
    bridge = MotrixReadOnlyBridge()
    iso = bridge.assert_driver_isolation()
    assert iso["read_only"] is True
    assert iso["isolated"] is True
    assert len(iso["forbidden_modules_loaded"]) == 0

    # Trying to send a write command over WebSocket mirror is rejected
    res = bridge.handle_incoming_message('{"type": "joint_cmd", "targets": [1, 2, 3]}')
    assert res["type"] == "rejected"
    assert res["code"] == "readonly_mirror_violation"
    assert bridge.rejected_writes == 1
