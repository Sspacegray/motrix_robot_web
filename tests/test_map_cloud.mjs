import test from 'node:test';
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
const load = async file => import(`data:text/javascript;base64,${Buffer.from(await readFile(new URL(file, import.meta.url), 'utf8')).toString('base64')}`);
const { occupancyClass, occupancyToPointCloud, describeMapCloud, MOCK_CLOUD_SOURCE } = await load('../frontend/js/map-cloud.js');

test('occupancy classes follow the local YAML thresholds, not invented lidar returns', () => {
  assert.equal(occupancyClass(0), 'occupied');
  assert.equal(occupancyClass(205), 'unknown');
  assert.equal(occupancyClass(254), 'free');
});

test('point cloud is derived from the occupancy grid and never marked live', () => {
  const width = 4, height = 2, luma = Uint8Array.from([0,0,254,205, 254,254,0,205]);
  const cloud = occupancyToPointCloud({ luma, width, height, resolution: 0.05, origin: [-9.19, -7.85], wallHeight: 0.2, wallStep: 0.2, floorStride: 1 });
  assert.equal(cloud.live, false);
  assert.equal(cloud.source, MOCK_CLOUD_SOURCE);
  assert.equal(cloud.occupiedCells, 3);
  assert.equal(cloud.unknownCells, 2);
  assert.ok(cloud.count >= 3 * 2);
  assert.match(cloud.label, /不是真实激光雷达/);
  // First occupied pixel (col 0,row 0) maps with ROS image origin at top-left.
  assert.ok(Math.abs(cloud.positions[0] - (-9.19 + 0.025)) < 1e-5);
  assert.ok(Math.abs(cloud.positions[1] - (-7.85 + (2-1-0+0.5)*0.05)) < 1e-5);
  let writes = 0; const original = globalThis.fetch; globalThis.fetch = () => { writes++; throw Error('no network'); };
  try { describeMapCloud(true); assert.equal(writes, 0); } finally { globalThis.fetch = original; }
});

test('unsupported robots do not receive another robot\'s mock cloud', () => {
  const denied = describeMapCloud(false, 'Dexforce W1 无移动底盘导航能力');
  assert.equal(denied.supported, false);
  assert.equal(denied.pointcloud.available, false);
  assert.equal(denied.pointcloud.live, false);
  assert.equal(denied.pointcloud.source, null);
});
