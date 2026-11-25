import test from "node:test";
import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import {
  analyzeOccupancyGrid,
  exportNav2OccupancyRgba,
  queryCellInfo,
  castLidarScan,
  planGridPathAStar,
} from "../frontend/js/map-planner-2d.js";

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

test("denoiseLevel=2 removes floating 3x3 obstacle cluster while preserving real walls", () => {
  const w = 60, h = 60;
  // Floating 3x3 ghost cluster (9 pixels) in center + long real wall segment at x=15
  const rgba = makeRoomRgba(w, h, (x, y) =>
    (x >= 29 && x <= 31 && y >= 29 && y <= 31) || (x === 15 && y >= 5 && y <= 45)
  );
  const light = analyzeOccupancyGrid(rgba, w, h, 0.1, { denoiseLevel: 1 });
  assert.equal(queryCellInfo(light, 30, 30).state, "occupied");

  const strong = analyzeOccupancyGrid(rgba, w, h, 0.1, { denoiseLevel: 2 });
  assert.equal(queryCellInfo(strong, 30, 30).state, "free");
  assert.equal(queryCellInfo(strong, 15, 25).state, "occupied", "structural wall must stay intact");
  assert.ok(strong.stats.removedClusters >= 1 && strong.stats.removedPixels >= 9);
});

test("virtualWalls block A* routing and inflate costmap while physical LiDAR passes through", () => {
  const w = 80, h = 60;
  // Wall with doorway at y=45..58
  const rgba = makeRoomRgba(w, h, (x, y) => x >= 39 && x <= 41 && y < 45);
  // Seal the doorway with a virtual wall from (40, 42) to (40, 59)
  const grid = analyzeOccupancyGrid(rgba, w, h, 0.1, {
    denoiseLevel: 1,
    virtualWalls: [{ x1: 40, y1: 42, x2: 40, y2: 59, thicknessPx: 3.5 }],
  });
  assert.equal(queryCellInfo(grid, 40, 50).state, "occupied");
  assert.equal(grid.physicalCls[50 * w + 40], 1, "physical occupancy stays free under software virtual wall");

  const blocked = planGridPathAStar(grid, 15, 15, 65, 15, { robotRadiusM: 0.25 });
  assert.equal(blocked.ok, false, "A* must refuse to cross virtual wall sealing the doorway");

  // Physical LiDAR shining east at y=50 should pass straight through the virtual wall to x=79
  const scan = castLidarScan(grid, 20, 50, 0, { rays: 4, maxRangeM: 10.0 });
  assert.ok(scan[0].distM > 5.0, `physical LiDAR should pass virtual wall, got ${scan[0].distM}m`);

  // Exported Nav2 RGBA bakes the virtual wall as occupied (0)
  const nav2Rgba = exportNav2OccupancyRgba(grid);
  assert.equal(nav2Rgba[(50 * w + 40) * 4], 0);
  assert.equal(nav2Rgba[(15 * w + 15) * 4], 254);
});

test("eraseStrokes clears obstacles and opens a previously blocked doorway for A*", () => {
  const w = 80, h = 60;
  // Solid wall sealing entire room vertically at x=40
  const rgba = makeRoomRgba(w, h, (x) => x >= 39 && x <= 41);
  const sealed = analyzeOccupancyGrid(rgba, w, h, 0.1);
  assert.equal(planGridPathAStar(sealed, 15, 30, 65, 30, { robotRadiusM: 0.2 }).ok, false);

  // Erase a doorway around (40, 30) with radius 6px
  const opened = analyzeOccupancyGrid(rgba, w, h, 0.1, {
    eraseStrokes: [{ pointsPx: [[38, 30], [42, 30]], radiusPx: 6.5 }],
  });
  assert.ok(opened.stats.erasedPixels > 20);
  const routed = planGridPathAStar(opened, 15, 30, 65, 30, { robotRadiusM: 0.2 });
  assert.equal(routed.ok, true, "erased opening must allow A* through");
});
