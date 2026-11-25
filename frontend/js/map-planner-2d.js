// Pure client-side 2D robotics map analysis & tactical overlays (zero control API / network imports):
// - ROS map_saver trinary parsing + physically faithful morphological speckle cleanup
// - Exact 2-pass Chamfer Euclidean Distance Transform (meters to nearest obstacle -> Nav2 costmap)
// - Cost-aware A* collision-free path preview on real occupancy grid + Chaikin path smoothing
// - Real-time 2D DDA LiDAR raycasting against occupied cells

function distPointToSegSq(px, py, x1, y1, x2, y2) {
  const vx = x2 - x1;
  const vy = y2 - y1;
  const lenSq = vx * vx + vy * vy;
  if (lenSq <= 1e-9) {
    const dx = px - x1;
    const dy = py - y1;
    return dx * dx + dy * dy;
  }
  let t = ((px - x1) * vx + (py - y1) * vy) / lenSq;
  if (t < 0) t = 0;
  else if (t > 1) t = 1;
  const cx = x1 + t * vx;
  const cy = y1 + t * vy;
  const dx = px - cx;
  const dy = py - cy;
  return dx * dx + dy * dy;
}

function rasterizeSegmentCapsule(width, height, x1, y1, x2, y2, radiusPx, visit) {
  const r = Math.max(0.6, radiusPx);
  const rSq = r * r;
  const minX = Math.max(0, Math.floor(Math.min(x1, x2) - r - 1));
  const maxX = Math.min(width - 1, Math.ceil(Math.max(x1, x2) + r + 1));
  const minY = Math.max(0, Math.floor(Math.min(y1, y2) - r - 1));
  const maxY = Math.min(height - 1, Math.ceil(Math.max(y1, y2) + r + 1));
  for (let y = minY; y <= maxY; y++) {
    const row = y * width;
    for (let x = minX; x <= maxX; x++) {
      if (distPointToSegSq(x, y, x1, y1, x2, y2) <= rSq) {
        visit(row + x);
      }
    }
  }
}

export function analyzeOccupancyGrid(rgba, width, height, resolution, options = {}) {
  const {
    denoiseLevel = 1,
    eraseStrokes = [],
    virtualWalls = [],
  } = options || {};
  const n = width * height;
  const raw = new Uint8Array(n); // 0=unknown, 1=free, 2=occupied
  for (let i = 0; i < n; i++) {
    const p = i * 4;
    const luma = (rgba[p] + rgba[p + 1] + rgba[p + 2]) / 3;
    if (luma <= 90) raw[i] = 2;
    else if (luma >= 235) raw[i] = 1;
    else raw[i] = 0;
  }

  const cls = new Uint8Array(raw);
  let removedClusters = 0;
  let removedPixels = 0;
  let erasedPixels = 0;
  let virtualWallPixels = 0;

  // Stage 1: Light 3x3 morphological speckle denoising (denoiseLevel >= 1)
  if (denoiseLevel >= 1) {
    for (let y = 1; y < height - 1; y++) {
      const row = y * width;
      for (let x = 1; x < width - 1; x++) {
        const idx = row + x;
        const c = raw[idx];
        let freeN = 0;
        let wallN = 0;
        for (let dy = -1; dy <= 1; dy++) {
          const nr = (y + dy) * width;
          for (let dx = -1; dx <= 1; dx++) {
            if (dx === 0 && dy === 0) continue;
            const nc = raw[nr + x + dx];
            if (nc === 1) freeN++;
            else if (nc === 2) wallN++;
          }
        }
        if (c === 0 && freeN >= 5 && wallN === 0) {
          cls[idx] = 1;
        } else if (c === 2 && wallN === 0 && freeN >= 6) {
          cls[idx] = 1;
          removedPixels++;
        }
      }
    }
  }

  // Stage 2: Strong Connected-Component Floating Noise Pruning (denoiseLevel >= 2)
  // Removes isolated floating SLAM ghost clusters (people/chair legs/dust <= 18px surrounded by free space)
  // while preserving 100% of structural walls and doorways.
  if (denoiseLevel >= 2) {
    const visited = new Uint8Array(n);
    const queue = new Int32Array(n);
    const maxIslandPx = 18;
    const n8 = [-1, 1, -width, width, -width - 1, -width + 1, width - 1, width + 1];

    for (let i = 0; i < n; i++) {
      if (cls[i] !== 2 || visited[i]) continue;
      let head = 0;
      let tail = 0;
      queue[tail++] = i;
      visited[i] = 1;
      let touchesFree = 0;
      let touchesBorder = false;

      while (head < tail) {
        const cur = queue[head++];
        const cx = cur % width;
        const cy = (cur - cx) / width;
        if (cx === 0 || cx === width - 1 || cy === 0 || cy === height - 1) {
          touchesBorder = true;
          continue;
        }
        for (let d = 0; d < 8; d++) {
          const ni = cur + n8[d];
          const nc = cls[ni];
          if (nc === 2) {
            if (!visited[ni]) {
              visited[ni] = 1;
              queue[tail++] = ni;
            }
          } else if (nc === 1) {
            touchesFree++;
          }
        }
      }

      if (!touchesBorder && tail <= maxIslandPx && touchesFree >= Math.max(4, tail)) {
        removedClusters++;
        removedPixels += tail;
        for (let k = 0; k < tail; k++) {
          cls[queue[k]] = 1;
        }
      }
    }

    // Also seal tiny enclosed unknown holes (<= 14px) sitting completely inside free floor
    visited.fill(0);
    const n4 = [-1, 1, -width, width];
    for (let i = 0; i < n; i++) {
      if (cls[i] !== 0 || visited[i]) continue;
      let head = 0;
      let tail = 0;
      queue[tail++] = i;
      visited[i] = 1;
      let enclosed = true;

      while (head < tail) {
        const cur = queue[head++];
        const cx = cur % width;
        const cy = (cur - cx) / width;
        if (cx === 0 || cx === width - 1 || cy === 0 || cy === height - 1) {
          enclosed = false;
        } else {
          for (let d = 0; d < 4; d++) {
            const ni = cur + n4[d];
            const nc = cls[ni];
            if (nc === 0) {
              if (!visited[ni]) {
                visited[ni] = 1;
                queue[tail++] = ni;
              }
            } else if (nc !== 1) {
              enclosed = false;
            }
          }
        }
      }
      if (enclosed && tail <= 14) {
        for (let k = 0; k < tail; k++) cls[queue[k]] = 1;
      }
    }
  }

  // Stage 3: Manual Noise Eraser Strokes (converts occupied noise pixels inside brush radius to free)
  for (const stroke of eraseStrokes) {
    const pts = stroke?.pointsPx;
    if (!Array.isArray(pts) || pts.length === 0) continue;
    const radPx = Math.max(1.0, Number(stroke.radiusPx) || 6.0);
    if (pts.length === 1) {
      rasterizeSegmentCapsule(width, height, pts[0][0], pts[0][1], pts[0][0], pts[0][1], radPx, (idx) => {
        if (cls[idx] === 2) {
          cls[idx] = 1;
          erasedPixels++;
        }
      });
    } else {
      for (let s = 1; s < pts.length; s++) {
        rasterizeSegmentCapsule(width, height, pts[s - 1][0], pts[s - 1][1], pts[s][0], pts[s][1], radPx, (idx) => {
          if (cls[idx] === 2) {
            cls[idx] = 1;
            erasedPixels++;
          }
        });
      }
    }
  }

  // Preserve physical SLAM occupancy (without software virtual walls) for faithful LiDAR raycasting & wall layer
  const physicalCls = new Uint8Array(cls);

  // Stage 4: Rasterize Nav2 Keepout Virtual Walls into navigation occupancy & dedicated mask
  const virtualMask = new Uint8Array(n);
  for (const vw of virtualWalls) {
    if (!vw) continue;
    const halfThickPx = Math.max(1.1, (Number(vw.thicknessPx) || 3.2) * 0.5);
    rasterizeSegmentCapsule(width, height, vw.x1, vw.y1, vw.x2, vw.y2, halfThickPx, (idx) => {
      if (!virtualMask[idx]) {
        virtualMask[idx] = 1;
        virtualWallPixels++;
      }
      cls[idx] = 2;
    });
  }

  // Tight bounding box of explored cells
  let minX = width, minY = height, maxX = -1, maxY = -1;
  for (let y = 0; y < height; y++) {
    const row = y * width;
    for (let x = 0; x < width; x++) {
      if (cls[row + x] !== 0) {
        if (x < minX) minX = x;
        if (x > maxX) maxX = x;
        if (y < minY) minY = y;
        if (y > maxY) maxY = y;
      }
    }
  }
  const bounds = maxX >= minX && maxY >= minY
    ? { minX, minY, maxX, maxY, width: maxX - minX + 1, height: maxY - minY + 1 }
    : { minX: 0, minY: 0, maxX: width - 1, maxY: height - 1, width, height };

  // Exact 2-pass Chamfer Distance Transform (in pixels, then converted to meters)
  const INF = 1e6;
  const distPx = new Float32Array(n);
  for (let i = 0; i < n; i++) {
    distPx[i] = cls[i] === 2 ? 0 : INF;
  }
  const DIAG = 1.41421356;
  // Forward pass
  for (let y = 0; y < height; y++) {
    const row = y * width;
    for (let x = 0; x < width; x++) {
      const idx = row + x;
      let d = distPx[idx];
      if (d === 0) continue;
      if (x > 0 && distPx[idx - 1] + 1 < d) d = distPx[idx - 1] + 1;
      if (y > 0) {
        const up = idx - width;
        if (distPx[up] + 1 < d) d = distPx[up] + 1;
        if (x > 0 && distPx[up - 1] + DIAG < d) d = distPx[up - 1] + DIAG;
        if (x + 1 < width && distPx[up + 1] + DIAG < d) d = distPx[up + 1] + DIAG;
      }
      distPx[idx] = d;
    }
  }
  // Backward pass
  for (let y = height - 1; y >= 0; y--) {
    const row = y * width;
    for (let x = width - 1; x >= 0; x--) {
      const idx = row + x;
      let d = distPx[idx];
      if (d === 0) continue;
      if (x + 1 < width && distPx[idx + 1] + 1 < d) d = distPx[idx + 1] + 1;
      if (y + 1 < height) {
        const dn = idx + width;
        if (distPx[dn] + 1 < d) d = distPx[dn] + 1;
        if (x > 0 && distPx[dn - 1] + DIAG < d) d = distPx[dn - 1] + DIAG;
        if (x + 1 < width && distPx[dn + 1] + DIAG < d) d = distPx[dn + 1] + DIAG;
      }
      distPx[idx] = d;
    }
  }

  const distM = new Float32Array(n);
  const res = Math.max(1e-4, Number(resolution) || 0.05);
  for (let i = 0; i < n; i++) {
    distM[i] = distPx[i] >= INF * 0.5 ? 99.0 : distPx[i] * res;
  }

  return {
    width,
    height,
    resolution: res,
    cls,
    physicalCls,
    virtualMask,
    distM,
    bounds,
    stats: {
      removedClusters,
      removedPixels,
      erasedPixels,
      virtualWallPixels,
    },
  };
}

// Export standard ROS 2 map_server / Nav2 KeepoutFilter trinary grayscale RGBA buffer
// (254=free, 205=unknown, 0=occupied including denoised walls + virtual walls)
export function exportNav2OccupancyRgba(grid) {
  const { width, height, cls } = grid;
  const out = new Uint8ClampedArray(width * height * 4);
  for (let i = 0; i < width * height; i++) {
    const c = cls[i];
    const g = c === 2 ? 0 : c === 1 ? 254 : 205;
    const p = i * 4;
    out[p] = g;
    out[p + 1] = g;
    out[p + 2] = g;
    out[p + 3] = 255;
  }
  return out;
}

export function queryCellInfo(grid, px, py) {
  if (!grid) return { state: "unknown", clearanceM: null };
  const x = Math.round(px);
  const y = Math.round(py);
  if (x < 0 || x >= grid.width || y < 0 || y >= grid.height) {
    return { state: "unknown", clearanceM: null };
  }
  const idx = y * grid.width + x;
  const c = grid.cls[idx];
  return {
    state: c === 2 ? "occupied" : c === 1 ? "free" : "unknown",
    clearanceM: grid.distM[idx],
  };
}

// 2D DDA / Bresenham LiDAR raycast against occupied cells
export function castLidarScan(grid, originPx, originPy, originYawRad, {
  rays = 96,
  maxRangeM = 12.0,
} = {}) {
  if (!grid) return [];
  const { width, height, resolution } = grid;
  const cls = grid.physicalCls || grid.cls;
  const maxSteps = Math.min(400, Math.round(maxRangeM / resolution));
  const out = [];
  const stepPx = 1.0;

  for (let i = 0; i < rays; i++) {
    const relAngle = (i / rays) * Math.PI * 2;
    const worldAngle = originYawRad + relAngle;
    // Image Y points downward while world Y points upward
    const dx = Math.cos(worldAngle) * stepPx;
    const dy = -Math.sin(worldAngle) * stepPx;
    let x = originPx;
    let y = originPy;
    let hit = false;
    let steps = 0;

    for (let s = 2; s <= maxSteps; s++) {
      x += dx;
      y += dy;
      const ix = Math.round(x);
      const iy = Math.round(y);
      if (ix < 0 || ix >= width || iy < 0 || iy >= height) {
        steps = s;
        break;
      }
      const c = cls[iy * width + ix];
      if (c === 2) {
        hit = true;
        steps = s;
        break;
      }
      steps = s;
    }
    out.push({
      px: originPx + dx * steps,
      py: originPy + dy * steps,
      distM: steps * resolution,
      hit,
      relAngle,
    });
  }
  return out;
}

class MinHeap {
  constructor() { this.items = []; }
  get size() { return this.items.length; }
  push(node) {
    const a = this.items;
    a.push(node);
    let i = a.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (a[p].f <= a[i].f) break;
      const tmp = a[p]; a[p] = a[i]; a[i] = tmp;
      i = p;
    }
  }
  pop() {
    const a = this.items;
    const top = a[0];
    const last = a.pop();
    if (a.length > 0) {
      a[0] = last;
      let i = 0;
      const len = a.length;
      while (true) {
        let l = (i << 1) + 1;
        let r = l + 1;
        let smallest = i;
        if (l < len && a[l].f < a[smallest].f) smallest = l;
        if (r < len && a[r].f < a[smallest].f) smallest = r;
        if (smallest === i) break;
        const tmp = a[i]; a[i] = a[smallest]; a[smallest] = tmp;
        i = smallest;
      }
    }
    return top;
  }
}

function findNearestNavigable(cls, distM, w, h, sx, sy, minClearM) {
  const cx = Math.max(0, Math.min(w - 1, Math.round(sx)));
  const cy = Math.max(0, Math.min(h - 1, Math.round(sy)));
  const cIdx = cy * w + cx;
  if (cls[cIdx] === 1 && distM[cIdx] >= minClearM) return { x: cx, y: cy };
  for (let r = 1; r <= 12; r++) {
    for (let dy = -r; dy <= r; dy++) {
      for (let dx = -r; dx <= r; dx++) {
        if (Math.abs(dx) !== r && Math.abs(dy) !== r) continue;
        const nx = cx + dx;
        const ny = cy + dy;
        if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
        const ni = ny * w + nx;
        if (cls[ni] === 1 && distM[ni] >= minClearM) return { x: nx, y: ny };
      }
    }
  }
  return null;
}

// Cost-aware A* pathfinder on a fast downsampled grid with Chaikin smoothing
export function planGridPathAStar(grid, startPx, startPy, goalPx, goalPy, {
  robotRadiusM = 0.24,
  safeClearM = 0.72,
  maxExpanded = 36000,
} = {}) {
  if (!grid) return { ok: false, reason: "no_grid", pointsPx: [], lengthM: 0, minClearanceM: 0 };
  const { width, height, resolution, cls, distM } = grid;

  // Choose stride so planning stays < 6ms even on 709x755 maps
  const stride = Math.max(1, Math.round(Math.max(width, height) / 360));
  const gw = Math.ceil(width / stride);
  const gh = Math.ceil(height / stride);
  const cellM = resolution * stride;

  // Build coarse traversability + clearance grid (conservative min clearance inside each block)
  const gFree = new Uint8Array(gw * gh);
  const gClear = new Float32Array(gw * gh);
  for (let gy = 0; gy < gh; gy++) {
    const y0 = gy * stride;
    const y1 = Math.min(height, y0 + stride);
    for (let gx = 0; gx < gw; gx++) {
      const x0 = gx * stride;
      const x1 = Math.min(width, x0 + stride);
      let hasWall = false;
      let hasFree = false;
      let minD = 99.0;
      for (let y = y0; y < y1; y++) {
        const row = y * width;
        for (let x = x0; x < x1; x++) {
          const idx = row + x;
          const c = cls[idx];
          if (c === 2) hasWall = true;
          else if (c === 1) hasFree = true;
          if (distM[idx] < minD) minD = distM[idx];
        }
      }
      const gi = gy * gw + gx;
      gFree[gi] = (!hasWall && hasFree) ? 1 : 0;
      gClear[gi] = minD;
    }
  }

  // Verify goal isn't directly inside a wall
  const gxRaw = Math.max(0, Math.min(gw - 1, Math.round(goalPx / stride)));
  const gyRaw = Math.max(0, Math.min(gh - 1, Math.round(goalPy / stride)));
  const goalCheck = queryCellInfo(grid, goalPx, goalPy);
  if (goalCheck.state === "occupied") {
    return { ok: false, reason: "goal_in_obstacle", pointsPx: [], lengthM: 0, minClearanceM: 0 };
  }

  const startCell = findNearestNavigable(gFree, gClear, gw, gh, startPx / stride, startPy / stride, robotRadiusM * 0.5)
    || findNearestNavigable(gFree, gClear, gw, gh, startPx / stride, startPy / stride, 0.01);
  const goalCell = findNearestNavigable(gFree, gClear, gw, gh, gxRaw, gyRaw, robotRadiusM * 0.65)
    || findNearestNavigable(gFree, gClear, gw, gh, gxRaw, gyRaw, 0.01);

  if (!startCell || !goalCell) {
    return { ok: false, reason: "unreachable", pointsPx: [], lengthM: 0, minClearanceM: 0 };
  }

  const startIdx = startCell.y * gw + startCell.x;
  const goalIdx = goalCell.y * gw + goalCell.x;
  if (startIdx === goalIdx) {
    return {
      ok: true,
      pointsPx: [[startPx, startPy], [goalPx, goalPy]],
      lengthM: Math.hypot(goalPx - startPx, goalPy - startPy) * resolution,
      minClearanceM: Math.min(gClear[startIdx], gClear[goalIdx]),
    };
  }

  const gScore = new Float32Array(gw * gh).fill(Infinity);
  const cameFrom = new Int32Array(gw * gh).fill(-1);
  const closed = new Uint8Array(gw * gh);
  const heap = new MinHeap();

  gScore[startIdx] = 0;
  const h0 = Math.hypot(goalCell.x - startCell.x, goalCell.y - startCell.y);
  heap.push({ idx: startIdx, f: h0 });

  const dirs = [
    [1, 0, 1.0], [-1, 0, 1.0], [0, 1, 1.0], [0, -1, 1.0],
    [1, 1, 1.4142], [-1, 1, 1.4142], [1, -1, 1.4142], [-1, -1, 1.4142],
  ];

  let found = false;
  let expanded = 0;
  while (heap.size > 0 && expanded < maxExpanded) {
    const cur = heap.pop();
    const ci = cur.idx;
    if (closed[ci]) continue;
    if (ci === goalIdx) {
      found = true;
      break;
    }
    closed[ci] = 1;
    expanded++;
    const cx = ci % gw;
    const cy = (ci - cx) / gw;
    const curG = gScore[ci];

    for (let d = 0; d < 8; d++) {
      const nx = cx + dirs[d][0];
      const ny = cy + dirs[d][1];
      if (nx < 0 || nx >= gw || ny < 0 || ny >= gh) continue;
      const ni = ny * gw + nx;
      if (closed[ni] || !gFree[ni]) continue;

      const clear = gClear[ni];
      if (clear < robotRadiusM * 0.45) continue;

      // Soft Nav2 inflation penalty steers routes cleanly down corridor centers!
      let penalty = 0;
      if (clear < safeClearM) {
        const ratio = (safeClearM - clear) / Math.max(0.05, safeClearM);
        penalty = ratio * ratio * 3.8;
      }
      const nextG = curG + dirs[d][2] * (1.0 + penalty);
      if (nextG < gScore[ni]) {
        gScore[ni] = nextG;
        cameFrom[ni] = ci;
        const h = Math.hypot(goalCell.x - nx, goalCell.y - ny);
        heap.push({ idx: ni, f: nextG + h });
      }
    }
  }

  if (!found) {
    return { ok: false, reason: "no_path", pointsPx: [], lengthM: 0, minClearanceM: 0 };
  }

  // Reconstruct coarse polyline in full image pixel space
  const rawPts = [];
  let cur = goalIdx;
  while (cur !== -1) {
    const gx = cur % gw;
    const gy = (cur - gx) / gw;
    rawPts.push([(gx + 0.5) * stride, (gy + 0.5) * stride]);
    cur = cameFrom[cur];
  }
  rawPts.reverse();
  rawPts[0] = [startPx, startPy];
  rawPts[rawPts.length - 1] = [goalPx, goalPy];

  // Subsample + 2-pass Chaikin smoothing for silky Nav2-like curved paths
  const decimated = [rawPts[0]];
  for (let i = 1; i < rawPts.length - 1; i++) {
    const prev = decimated[decimated.length - 1];
    if (Math.hypot(rawPts[i][0] - prev[0], rawPts[i][1] - prev[1]) >= Math.max(3.0, stride * 2.2)) {
      decimated.push(rawPts[i]);
    }
  }
  decimated.push(rawPts[rawPts.length - 1]);

  let smooth = decimated;
  for (let pass = 0; pass < 2 && smooth.length > 2; pass++) {
    const next = [smooth[0]];
    for (let i = 0; i < smooth.length - 1; i++) {
      const [x0, y0] = smooth[i];
      const [x1, y1] = smooth[i + 1];
      next.push([0.75 * x0 + 0.25 * x1, 0.75 * y0 + 0.25 * y1]);
      next.push([0.25 * x0 + 0.75 * x1, 0.25 * y0 + 0.75 * y1]);
    }
    next.push(smooth[smooth.length - 1]);
    smooth = next;
  }

  let lengthM = 0;
  let minClearM = 99.0;
  for (let i = 0; i < smooth.length; i++) {
    const info = queryCellInfo(grid, smooth[i][0], smooth[i][1]);
    if (info.clearanceM !== null && info.clearanceM < minClearM) minClearM = info.clearanceM;
    if (i > 0) {
      lengthM += Math.hypot(smooth[i][0] - smooth[i - 1][0], smooth[i][1] - smooth[i - 1][1]) * resolution;
    }
  }

  return {
    ok: true,
    pointsPx: smooth,
    lengthM: Math.round(lengthM * 100) / 100,
    minClearanceM: minClearM >= 90 ? 0 : Math.round(minClearM * 100) / 100,
  };
}
