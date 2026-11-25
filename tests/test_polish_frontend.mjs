// Polish v6 frontend regressions: real telemetry bindings, zero background camera leak, visual workflow builder.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';

const read = (rel) => readFile(new URL(rel, import.meta.url), 'utf8');

test('diagnostics binds to real ObservationSnapshot and AuthorityState schemas', async () => {
  const src = await read('../frontend/js/tabs/diagnostics.js');
  assert.match(src, /obs\.motors/);
  assert.match(src, /obs\.interlocks/);
  assert.match(src, /obs\.topics/);
  assert.match(src, /auth\.heartbeat_age_s/);
  assert.match(src, /auth\.idle_age_s/);
  assert.match(src, /state\.connectors/);
  assert.doesNotMatch(src, /obs\.motor_temperatures|obs\.motor_error_codes|auth\.last_heartbeat_age_ms|auth\.idle_timeout_remaining_s/);
});

test('navigation binds to real obs.navigation, fixes active-tab teleop check, and scales canvas clicks', async () => {
  const src = await read('../frontend/js/tabs/navigation.js');
  assert.match(src, /const nav = obs\.navigation \|\| \{\}/);
  assert.match(src, /nav\.mode/);
  assert.match(src, /nav\.pose_map/);
  assert.match(src, /nav\.chassis_vel/);
  assert.match(src, /nav\.path_points/);
  assert.match(src, /clientToCanvas/);
  assert.match(src, /container\.classList\.contains\("active"\)/);
  assert.doesNotMatch(src, /obs\.navigation_mode|obs\.chassis_pose|obs\.chassis_velocity|obs\.navigation_path|\.tab-pane\.active/);
});

test('perception guards camera polling by active tab and binds real backend PerceptionState', async () => {
  const src = await read('../frontend/js/tabs/perception.js');
  assert.match(src, /if \(!container\.classList\.contains\("active"\) \|\| document\.hidden\) return;/);
  assert.match(src, /p\.yolo_running/);
  assert.match(src, /p\.latched_mask/);
  assert.match(src, /p\.foundation_pose_running/);
  assert.match(src, /p\.foundation_pose_phase/);
  assert.match(src, /res\.xyz_m/);
  assert.match(src, /res\.rpy_deg/);
  assert.match(src, /aruco_start_pose/);
  assert.match(src, /aruco_stop_pose/);
  assert.match(src, /yoloLocked/);
  assert.doesNotMatch(src, /obs\.perception\.yolo_active|obs\.perception\.latched_generation|obs\.perception\.foundation_pose_active/);
});

test('actions enables visual workflow builder and action edit with modern dialog light-dismiss', async () => {
  const src = await read('../frontend/js/tabs/actions.js');
  assert.match(src, /id="workflowEditDialog" closedby="any"/);
  assert.match(src, /id="actionEditDialog" closedby="any"/);
  assert.match(src, /edit-act-btn/);
  assert.match(src, /edit-wf-btn/);
  assert.match(src, /api\.saveWorkflow\(/);
  assert.match(src, /enableDialogLightDismiss/);
});

test('policy reloads loop records on robot switch, wires set_config/tag_episode/delete, and keeps real recording disabled', async () => {
  const src = await read('../frontend/js/tabs/policy.js');
  assert.match(src, /if \(lastRobotId !== desc\.id\)/);
  assert.match(src, /mutatePolicy\("set_config"/);
  assert.match(src, /mutatePolicy\("tag_episode"/);
  assert.match(src, /api\.deleteLoopRecord/);
  assert.match(src, /id="btnToggleRecord" disabled/);
});
