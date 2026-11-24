// Local-only API state regression. No browser, fetch, sockets or robot endpoints.
import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
const text = await readFile(new URL('../frontend/js/api.js', import.meta.url), 'utf8');
const {RobotApi} = await import(`data:text/javascript;base64,${Buffer.from(text).toString('base64')}`);

test('robot switch releases old route before changing target and discards cached authority', async () => {
  const api = new RobotApi(); const seen = [];
  api.cachedState = {authority: {lease_active: true}};
  api.post = async (path) => {seen.push([path, api.currentRobotId]); return {};};
  api.connectSSE = () => {seen.push(['connect', api.currentRobotId]);};
  api.getState = async () => ({descriptor: {id: api.currentRobotId}, authority: {lease_active: false}});
  await api.setRobotId('piper-dual-mock');
  assert.deepEqual(seen[0], ['/api/robots/r1pro-mock/lease/release', 'r1pro-mock']);
  assert.deepEqual(seen[1], ['connect', 'piper-dual-mock']);
  assert.notEqual(api.cachedState?.authority?.lease_active, true);
  api._notify({descriptor: {id: 'r1pro-mock'}, authority: {lease_active: true}});
  assert.notEqual(api.cachedState?.authority?.lease_active, true);
});

test('failed old lease release refuses robot switch', async () => {
  const api = new RobotApi();
  api.post = async () => {throw Error('release failed');};
  api.connectSSE = () => assert.fail('must not connect new target');
  await assert.rejects(api.setRobotId('piper-dual-mock'), /release failed/);
  assert.equal(api.currentRobotId, 'r1pro-mock');
  assert.equal(api.heartbeatTimer, null);
});

const layoutSource = await readFile(new URL('../frontend/js/policy-layout.js', import.meta.url), 'utf8');
const {policyLayout} = await import(`data:text/javascript;base64,${Buffer.from(layoutSource).toString('base64')}`);
test('Action Layout derives dimensions from descriptor/checkpoint, not fixed Piper 14', () => {
  const descriptor = {joints: Array.from({length:20}, () => ({}))};
  const result = policyLayout(descriptor, {action_dim:20, compatible:true});
  assert.equal(result.status, 'matched');
  assert.match(result.text, /模型 20 \/ 机器人 20 DOF/);
  assert.equal(policyLayout(descriptor, {action_dim:14, compatible:true}).status, 'blocked');
  assert.equal(policyLayout(descriptor, null).status, 'unknown');
  assert.equal(policyLayout({}, {action_dim:14, compatible:true}).status, 'unknown');
});
