// Pure offline UI regression: no browser, network, robot or CAN.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
const source = await readFile(new URL('../frontend/js/ui-format.js', import.meta.url), 'utf8');
const {escapeHtml, formatLocalTime, filterJobs} = await import(`data:text/javascript;base64,${Buffer.from(source).toString('base64')}`);
test('job metadata cannot inject markup or attribute values', () => {
  assert.equal(escapeHtml('<img src=x onerror="alert(1)">'), '&lt;img src=x onerror=&quot;alert(1)&quot;&gt;');
  assert.equal(escapeHtml(null), '');
});
test('job filters combine actual state and case insensitive metadata', () => {
  const jobs = [{state:'Succeeded', flow_name:'Pick cup',run_id:'R-123'}, {state:'Running',flow_name:'Pick tray',operator:'Alice'}];
  assert.equal(filterJobs(jobs,'ALL','pick').length,2);
  assert.equal(filterJobs(jobs,'Running',' ALICE ').length,1);
  assert.equal(filterJobs(jobs,'Succeeded','alice').length,0);
  assert.equal(filterJobs(jobs,'ALL','R-123').length,1);
});
test('job dates use Asia/Shanghai, not browser timezone or raw UTC', () => {
  assert.match(formatLocalTime('2026-11-24T05:39:00Z'), /13:39:00/);
  assert.equal(formatLocalTime('bad-date'), '—');
});
test('Jobs page is actual landing view and dry-run uses supported APIs only', async () => {
  const app = await readFile(new URL('../frontend/js/app.js',import.meta.url),'utf8');
  const actions = await readFile(new URL('../frontend/js/tabs/actions.js',import.meta.url),'utf8');
  const drawer = await readFile(new URL('../frontend/js/drawer.js',import.meta.url),'utf8');
  assert.match(app,/this\.switchTab\("actions"\)/);
  assert.match(actions,/api\.runWorkflow\(workflows\[0\]\.id, true\)/);
  assert.match(drawer,/api\.retryJob\(this\.currentJobId, true\)/);
  assert.match(drawer,/this\.panel\.inert = true/);
  assert.match(drawer,/jobId !== this\.currentJobId/);
});
