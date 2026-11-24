import json
import pytest
from motrix_robot_web.storage import LocalStore


def test_storage_crud_and_seeding(tmp_path):
    store = LocalStore(tmp_path)

    # Verify seed actions and workflows loaded
    acts = store.list_actions()
    assert len(acts) >= 5
    r1_acts = store.list_actions("r1pro-mock")
    assert any(a.name == "r1_safe_home" for a in r1_acts)

    # Add custom action
    new_act = store.upsert_action({
        "name": "custom_pose",
        "robot_id": "r1pro-mock",
        "kind": "joint_targets",
        "params": {"targets": {"torso_1": 0.2}},
    })
    assert new_act.id.startswith("act-")
    assert store.get_action_by_name("r1pro-mock", "custom_pose") is not None

    # Delete custom action
    assert store.delete_action(new_act.id)
    assert store.get_action_by_name("r1pro-mock", "custom_pose") is None


def test_r1pro_json_compatibility_import_export(tmp_path):
    store = LocalStore(tmp_path)

    sample_r1_actions = {
        "version": 1,
        "actions": [
            {
                "id": "r1-act-001",
                "name": "r1_pickup_box",
                "type": "joint_targets",
                "description": "pickup targets in degrees",
                "params": {
                    "targets": [
                        {"group": "torso", "index": 0, "name": "torso[1]", "unit": "deg", "value": 20.0},
                        {"group": "left_arm", "index": 0, "name": "left_arm[1]", "unit": "deg", "value": -10.0},
                        {"group": "left_gripper", "index": 0, "name": "left_gripper[1]", "unit": "open_0_100", "value": 85.0},
                    ]
                }
            },
            {
                "id": "r1-act-002",
                "name": "r1_nav_sink",
                "type": "navigate_to_pose",
                "params": {"x_m": 2.34, "y_m": 0.21, "yaw_deg": 90.0}
            }
        ]
    }

    imported_acts = store.import_r1pro_actions(sample_r1_actions, robot_id="r1pro-mock")
    assert len(imported_acts) == 2

    act1 = store.get_action_by_name("r1pro-mock", "r1_pickup_box")
    assert act1 is not None
    assert "torso_1" in act1.params["targets"]
    assert "left_arm_1" in act1.params["targets"]
    assert "left_gripper" in act1.params["targets"]
    assert act1.params["targets"]["left_gripper"] == 0.85

    act2 = store.get_action_by_name("r1pro-mock", "r1_nav_sink")
    assert act2 is not None
    assert act2.kind == "navigate_to_pose"
    assert act2.params["x"] == 2.34
    assert abs(act2.params["yaw"] - 1.5708) < 0.01

    sample_r1_sequences = {
        "version": 1,
        "sequences": [
            {
                "id": "r1-seq-001",
                "name": "full_pickup_flow",
                "steps": [
                    {"actions": ["r1_pickup_box"], "delay_s": 0.5, "duration_override_s": 2.0},
                    {"actions": ["r1_nav_sink"], "delay_s": 0.2, "duration_override_s": 1.5}
                ]
            }
        ]
    }

    imported_wfs = store.import_r1pro_sequences(sample_r1_sequences, robot_id="r1pro-mock")
    assert len(imported_wfs) == 1
    wf = imported_wfs[0]
    assert wf.name == "full_pickup_flow"
    assert len(wf.steps) == 2
    assert wf.steps[0].action_name == "r1_pickup_box"
    assert wf.steps[0].delay_s == 0.5
    assert wf.steps[0].duration_s == 2.0

    # Test export bundle
    bundle = store.export_r1pro_bundle("r1pro-mock")
    assert "actions_json" in bundle
    assert "sequences_json" in bundle
    assert len(bundle["actions_json"]["actions"]) >= 2
    assert any(a["name"] == "r1_pickup_box" for a in bundle["actions_json"]["actions"])


def test_loop_records_and_eval_status(tmp_path):
    store = LocalStore(tmp_path)

    records = store.list_loop_records()
    assert len(records) >= 2
    # Check that status differentiates passed, failed, no_result, zero_success
    statuses = {r.eval_status for r in records}
    assert "passed" in statuses
    assert "no_result" in statuses

    # Add new record
    new_rec = store.upsert_loop_record({
        "robot_id": "piper-dual-mock",
        "task_name": "plate_stacking_v1",
        "owner": "wubinxin",
        "eval_status": "zero_success",
        "notes": "Gripper reached but dropped plate"
    })
    assert new_rec.record_id.startswith("loop-rec-")
    assert new_rec.eval_status == "zero_success"
