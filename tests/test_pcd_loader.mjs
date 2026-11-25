// Pure offline unit test for binary & ASCII PCL .pcd parser and colorizer.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';

const source = await readFile(new URL('../frontend/js/pcd-loader.js', import.meta.url), 'utf8');
const { parsePcdBuffer, buildColoredPcdCloud, RECORDED_PCD_SOURCE } = await import(
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
