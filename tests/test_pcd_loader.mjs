// Pure offline unit test for binary & ASCII PCL .pcd parser and colorizer.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';

const source = await readFile(new URL('../frontend/js/pcd-loader.js', import.meta.url), 'utf8');
const { parsePcdBuffer, filterDynamicObstacles3D, buildColoredPcdCloud, RECORDED_PCD_SOURCE } = await import(
  `data:text/javascript;base64,${Buffer.from(source).toString('base64')}`
);

function makeBinaryPcd(points) {
  const header = [
    '# .PCD v0.7',
    'VERSION 0.7',
    'FIELDS x y z intensity',
    'SIZE 4 4 4 4',
    'TYPE F F F F',
    'COUNT 1 1 1 1',
    `WIDTH ${points.length}`,
    'HEIGHT 1',
    `POINTS ${points.length}`,
    'DATA binary',
    '',
  ].join('\n');
  const headerBytes = new TextEncoder().encode(header);
  const body = new ArrayBuffer(points.length * 16);
  const view = new DataView(body);
  points.forEach(([x, y, z, inten], i) => {
    const off = i * 16;
    view.setFloat32(off, x, true);
    view.setFloat32(off + 4, y, true);
    view.setFloat32(off + 8, z, true);
    view.setFloat32(off + 12, inten, true);
  });
  const out = new Uint8Array(headerBytes.length + body.byteLength);
  out.set(headerBytes, 0);
  out.set(new Uint8Array(body), headerBytes.length);
  return out.buffer;
}

test('parses binary PCD, skips NaN points, and computes accurate bounds', () => {
  const buf = makeBinaryPcd([
    [1.0, 2.0, 0.05, 10],
    [NaN, 0.0, 1.0, 5],
    [-3.5, 4.0, 1.5, 80],
    [0.0, -1.0, 3.2, 40],
  ]);
  const parsed = parsePcdBuffer(buf);
  assert.equal(parsed.count, 3);
  assert.equal(parsed.hasIntensity, true);
  assert.ok(Math.abs(parsed.bounds.minX - (-3.5)) < 1e-5);
  assert.ok(Math.abs(parsed.bounds.maxZ - 3.2) < 1e-5);
});

test('parses ASCII PCD cleanly and rejects malformed headers', () => {
  const ascii = [
    'FIELDS x y z',
    'SIZE 4 4 4',
    'TYPE F F F',
    'COUNT 1 1 1',
    'WIDTH 2',
    'HEIGHT 1',
    'POINTS 2',
    'DATA ascii',
    '0.5 -0.5 0.1',
    '2.0 3.0 1.8',
  ].join('\n');
  const parsed = parsePcdBuffer(new TextEncoder().encode(ascii));
  assert.equal(parsed.count, 2);
  assert.equal(parsed.hasIntensity, false);
  assert.throws(() => parsePcdBuffer(new TextEncoder().encode('BROKEN HEADER\n')), /Invalid PCD header/);
});

test('slices ceiling by zMax and colorizes across height, intensity, and semantic modes with live=false', () => {
  const parsed = parsePcdBuffer(makeBinaryPcd([
    [0, 0, 0.05, 10],
    [1, 1, 1.20, 60],
    [2, 2, 3.10, 90],
  ]));
  const sliced = buildColoredPcdCloud(parsed, { colorMode: 'semantic', zMin: -1.0, zMax: 2.0 });
  assert.equal(sliced.count, 2);
  assert.equal(sliced.totalCount, 3);
  assert.equal(sliced.live, false);
  assert.equal(sliced.source, RECORDED_PCD_SOURCE);
  assert.equal(sliced.positions.length, 6);
  assert.equal(sliced.colors.length, 6);

  // Height & intensity modes also return valid finite RGB colors
  for (const mode of ['height', 'intensity']) {
    const cloud = buildColoredPcdCloud(parsed, { colorMode: mode, zMin: -1.0, zMax: 4.0 });
    assert.equal(cloud.count, 3);
    assert.equal(cloud.live, false);
    assert.ok(Array.from(cloud.colors).every(v => Number.isFinite(v) && v >= 0 && v <= 1.05));
  }
});

test('pcd-loader has zero control API or network fetch dependencies', () => {
  assert.doesNotMatch(source, /import.*api\.js|fetch\(|XMLHttpRequest|WebSocket/);
});

test('filterDynamicObstacles3D removes floating corridor pedestrian ghosts while keeping walls and floor', () => {
  const pts = [];
  // Floor at z=0.0
  for (let x = 0; x <= 6; x += 0.2) {
    for (let y = 0; y <= 6; y += 0.2) {
      pts.push([x, y, 0.0, 20]);
    }
  }
  // Dense continuous static wall along x=0.2 (spanning y=0..6, z=0.2..1.8)
  for (let y = 0; y <= 6; y += 0.1) {
    for (let z = 0.2; z <= 1.8; z += 0.15) {
      pts.push([0.2, y, z, 80]);
    }
  }
  // Sparse transient walking-person ghost cluster floating in open corridor around (3.2, 3.2)
  for (let k = 0; k < 12; k++) {
    pts.push([3.2 + (k % 3) * 0.08, 3.2 + Math.floor(k / 3) * 0.06, 0.55 + (k % 4) * 0.2, 40]);
  }
  const parsed = parsePcdBuffer(makeBinaryPcd(pts));
  const cleaned = filterDynamicObstacles3D(parsed);
  assert.ok(cleaned.dynamicRemoved >= 10, `expected >=10 dynamic ghost pts removed, got ${cleaned.dynamicRemoved}`);
  assert.ok(cleaned.count < parsed.count);

  // Confirm static wall points at x≈0.2 and floor points at z≈0.0 survived intact
  let wallKept = 0;
  let ghostKept = 0;
  for (let i = 0; i < cleaned.count; i++) {
    const x = cleaned.positions[i * 3];
    const y = cleaned.positions[i * 3 + 1];
    const z = cleaned.positions[i * 3 + 2];
    if (Math.abs(x - 0.2) < 0.1 && z > 0.3) wallKept++;
    if (Math.hypot(x - 3.2, y - 3.2) < 0.35 && z > 0.3) ghostKept++;
  }
  assert.ok(wallKept > 300, `static wall points must survive, got ${wallKept}`);
  assert.equal(ghostKept, 0, `floating corridor ghost points must be removed, got ${ghostKept}`);
});

test('buildColoredPcdCloud with hideGround removes tilted floor while keeping static walls', () => {
  const pts = [];
  // Gently tilted floor (z rises from 0.00 at x=0 to 0.14 at x=6) + static wall at x=0.5 (z=0.35..1.8)
  for (let x = 0; x <= 6; x += 0.25) {
    const fz = (x / 6) * 0.14;
    for (let y = 0; y <= 4; y += 0.25) {
      pts.push([x, y, fz, 15]);
    }
  }
  for (let y = 0; y <= 4; y += 0.2) {
    for (let z = 0.38; z <= 1.6; z += 0.2) {
      pts.push([0.5, y, z, 90]);
    }
  }
  const parsed = parsePcdBuffer(makeBinaryPcd(pts));
  const withGnd = buildColoredPcdCloud(parsed, { zMin: -0.5, zMax: 2.0, hideGround: false });
  const noGnd = buildColoredPcdCloud(parsed, { zMin: -0.5, zMax: 2.0, hideGround: true, groundMargin: 0.20 });

  assert.ok(noGnd.groundRemoved >= 350, `expected all floor points removed, got ${noGnd.groundRemoved}`);
  assert.ok(noGnd.count > 100 && noGnd.count < withGnd.count - 300, `walls must remain, got ${noGnd.count}`);
  for (let i = 0; i < noGnd.count; i++) {
    assert.ok(noGnd.positions[i * 3 + 2] >= 0.30, 'no floor points should remain');
  }
});
