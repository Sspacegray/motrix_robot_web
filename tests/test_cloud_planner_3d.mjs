import test from "node:test";
import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import {
  buildCloudTraversabilityGrid,
  queryCloudCell,
  raycastCloudSurface,
  planCloudWaypointsAStar,
} from "../frontend/js/cloud-planner-3d.js";

function makeSyntheticRoomCloud() {
  // 10m x 8m room: x in [0, 10], y in [0, 8], floor at z=0.05, ceiling at z=2.7 (must be ignored!)
  // Dividing wall at x=5 from y=0..5.2 (z=0.3..1.6), open doorway at y=5.5..7.5
  const pts = [];
  for (let x = 0; x <= 10; x += 0.1) {
    for (let y = 0; y <= 8; y += 0.1) {
      pts.push(x, y, 0.05); // floor
      pts.push(x, y, 2.75); // ceiling overhead (must NOT block robot!)
      const isOuterWall = x <= 0.15 || x >= 9.85 || y <= 0.15 || y >= 7.85;
      const isDivider = Math.abs(x - 5.0) <= 0.12 && y <= 5.2;
      if (isOuterWall || isDivider) {
        pts.push(x, y, 0.55);
        pts.push(x, y, 1.15);
      }
    }
  }
  return { positions: new Float32Array(pts), count: pts.length / 3 };
}

test("buildCloudTraversabilityGrid ignores ceiling >1.85m, detects 3D walls, and computes EDT clearance", () => {
  const { positions, count } = makeSyntheticRoomCloud();
  const grid = buildCloudTraversabilityGrid(positions, count, { resolution: 0.12 });
  assert.ok(grid);

  // Open floor under ceiling must stay FREE!
  const floorCell = queryCloudCell(grid, 2.0, 2.0);
  assert.equal(floorCell.state, "free");
  assert.ok(Math.abs(floorCell.z - 0.05) < 0.05);
  assert.ok(floorCell.clearanceM > 1.5, `expected open floor clearance >1.5m, got ${floorCell.clearanceM}`);

  // Dividing wall must be OCCUPIED!
  const wallCell = queryCloudCell(grid, 5.0, 2.5);
  assert.equal(wallCell.state, "occupied");

  // Doorway gap must be FREE!
  const doorCell = queryCloudCell(grid, 5.0, 6.5);
  assert.equal(doorCell.state, "free");
});

test("raycastCloudSurface picks accurate 3D floor coordinates from camera ray", () => {
  const { positions, count } = makeSyntheticRoomCloud();
  const grid = buildCloudTraversabilityGrid(positions, count, { resolution: 0.12 });

  // Camera above (2, 2, 10) looking straight down toward (3, 4, 0.05)
  const ox = 2, oy = 2, oz = 10;
  const vx = 3 - ox, vy = 4 - oy, vz = 0.05 - oz;
  const len = Math.hypot(vx, vy, vz);
  const hit = raycastCloudSurface(grid, [ox, oy, oz], [vx / len, vy / len, vz / len]);
  assert.ok(hit);
  assert.equal(hit.state, "free");
  assert.ok(Math.abs(hit.x - 3.0) < 0.15, `expected x≈3.0, got ${hit.x}`);
  assert.ok(Math.abs(hit.y - 4.0) < 0.15, `expected y≈4.0, got ${hit.y}`);
  assert.ok(Math.abs(hit.z - 0.05) < 0.08, `expected z≈0.05, got ${hit.z}`);
});

test("planCloudWaypointsAStar chains multi-waypoint 3D route through doorway and rejects obstacle goals", () => {
  const { positions, count } = makeSyntheticRoomCloud();
  const grid = buildCloudTraversabilityGrid(positions, count, { resolution: 0.12 });

  // Multi-waypoint route: (1.5, 2.0) -> P1(2.5, 6.5) -> P2(8.2, 2.0)
  const route = planCloudWaypointsAStar(
    grid,
    [1.5, 2.0],
    [{ x: 2.5, y: 6.5, z: 0.05 }, { x: 8.2, y: 2.0, z: 0.05 }],
    { robotRadiusM: 0.25 }
  );
  assert.equal(route.ok, true);
  assert.equal(route.legs.length, 2);
  assert.ok(route.points3d.length >= 8);
  assert.ok(route.points2d.length >= 6);
  // Route must pass through doorway (y > 5.3 around x=5) and hover cleanly above floor z≈0.05
  const nearWallPts = route.points3d.filter(([x]) => Math.abs(x - 5.0) < 0.5);
  assert.ok(nearWallPts.length > 0 && nearWallPts.every(([, y]) => y > 5.2), "route must pass through doorway");
  assert.ok(route.points3d.every(([, , z]) => z >= 0.15 && z <= 0.4));

  // Waypoint inside the dividing wall must fail cleanly
  const blocked = planCloudWaypointsAStar(grid, [1.5, 2.0], [{ x: 5.0, y: 2.0, z: 0.05 }]);
  assert.equal(blocked.ok, false);
  assert.equal(blocked.reason, "goal_in_obstacle");
});

test("virtualWallsWorld fuse into 3D cloud traversability grid and block doorway", () => {
  const { positions, count } = makeSyntheticRoomCloud();
  const grid = buildCloudTraversabilityGrid(positions, count, {
    resolution: 0.12,
    virtualWallsWorld: [{ x1: 5.0, y1: 5.0, x2: 5.0, y2: 8.0, thickness_m: 0.2 }],
  });
  const blocked = planCloudWaypointsAStar(grid, [1.5, 2.0], [{ x: 8.2, y: 2.0, z: 0.05 }]);
  assert.equal(blocked.ok, false);
});

test("cloud-planner-3d stays strictly read-only with zero API or network imports", async () => {
  const src = await readFile(new URL("../frontend/js/cloud-planner-3d.js", import.meta.url), "utf8");
  assert.doesNotMatch(src, /import.*api\.js|fetch\(|XMLHttpRequest|WebSocket|\/api\//);
});

test("Line-of-Sight string-pulling produces taut Euclidean shortest safe path across open diagonal", () => {
  // Open 10x8 room with no interior divider: diagonal from (1.5, 1.5) to (8.5, 4.5)
  const pts = [];
  for (let x = 0; x <= 10; x += 0.12) {
    for (let y = 0; y <= 8; y += 0.12) {
      pts.push(x, y, 0.05);
      if (x <= 0.15 || x >= 9.85 || y <= 0.15 || y >= 7.85) {
        pts.push(x, y, 0.6, x, y, 1.2);
      }
    }
  }
  const grid = buildCloudTraversabilityGrid(new Float32Array(pts), pts.length / 3, { resolution: 0.14 });
  const route = planCloudWaypointsAStar(grid, [1.5, 1.5], [{ x: 8.5, y: 4.5 }]);
  assert.equal(route.ok, true);
  const trueEuclidean = Math.hypot(8.5 - 1.5, 4.5 - 1.5); // 7.6158m
  assert.ok(
    route.lengthM <= trueEuclidean * 1.015,
    `expected taut Euclidean shortest path (~${trueEuclidean.toFixed(2)}m), got ${route.lengthM}m`
  );
  assert.equal(route.points3d.length, 2, "unobstructed diagonal should collapse cleanly to 2 endpoints");
});
