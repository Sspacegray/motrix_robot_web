import test from "node:test";
import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import { analyzeOccupancyGrid, queryCellInfo, castLidarScan, planGridPathAStar } from "../frontend/js/map-planner-2d.js";

function makeRoomRgba(w, h, wallPredicate) {
  const rgba = new Uint8ClampedArray(w * h * 4);
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      const p = (y * w + x) * 4;
      const isBorder = x === 0 || y === 0 || x === w - 1 || y === h - 1;
      const isWall = isBorder || wallPredicate(x, y);
      const v = isWall ? 0 : 254;
      rgba[p] = v;
      rgba[p + 1] = v;
      rgba[p + 2] = v;
      rgba[p + 3] = 255;
    }
  }
  return rgba;
}

test("analyzeOccupancyGrid computes accurate Euclidean wall clearance and denoises 1px dust", () => {
  const w = 60, h = 60;
  // Single isolated 1px dust pixel at (30,30), real walls only on border
  const rgba = makeRoomRgba(w, h, (x, y) => x === 30 && y === 30);
  const grid = analyzeOccupancyGrid(rgba, w, h, 0.1);
  const center = queryCellInfo(grid, 30, 30);
  assert.equal(center.state, "free");
  // Distance from (30,30) to nearest border wall (0 or 59) is 29 pixels = 2.9m
  assert.ok(Math.abs(center.clearanceM - 2.9) < 0.25, `expected ~2.9m, got ${center.clearanceM}`);
});

test("planGridPathAStar routes around a solid dividing wall with doorway and rejects goals inside walls", () => {
  const w = 80, h = 60;
  // Solid vertical wall at x=40 from y=0..44, leaving an open doorway at y=45..58
  const rgba = makeRoomRgba(w, h, (x, y) => x >= 39 && x <= 41 && y < 45);
  const grid = analyzeOccupancyGrid(rgba, w, h, 0.1);

  const res = planGridPathAStar(grid, 15, 15, 65, 15, { robotRadiusM: 0.25 });
  assert.equal(res.ok, true);
  assert.ok(res.pointsPx.length >= 4);
  // Path MUST detour down through the doorway (y > 44) rather than crossing the wall!
  const maxY = Math.max(...res.pointsPx.map((p) => p[1]));
  assert.ok(maxY >= 44, `expected route to detour through doorway (y>=44), got maxY=${maxY}`);
  assert.ok(res.lengthM > 8.0, `detour length should exceed straight-line 5m, got ${res.lengthM}`);

  // Goal directly inside the wall must fail explicitly
  const bad = planGridPathAStar(grid, 15, 15, 40, 20, { robotRadiusM: 0.25 });
  assert.equal(bad.ok, false);
  assert.equal(bad.reason, "goal_in_obstacle");
});

test("castLidarScan hits surrounding room walls accurately", () => {
  const w = 60, h = 60;
  const rgba = makeRoomRgba(w, h, () => false);
  const grid = analyzeOccupancyGrid(rgba, w, h, 0.1);
  const hits = castLidarScan(grid, 30, 30, 0, { rays: 36, maxRangeM: 10.0 });
  assert.equal(hits.length, 36);
  assert.ok(hits.every((r) => r.hit), "all rays inside enclosed room should hit walls");
  assert.ok(hits[0].distM >= 2.7 && hits[0].distM <= 3.1, `east wall expected ~2.9m, got ${hits[0].distM}`);
});

test("map-planner-2d stays strictly read-only with zero control API imports", async () => {
  const src = await readFile(new URL("../frontend/js/map-planner-2d.js", import.meta.url), "utf8");
  assert.doesNotMatch(src, /import.*api\.js|fetch\(|\/api\//);
});
