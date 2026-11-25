// Pure offline binary/ASCII PCL .pcd parser & colorizer. Zero API or network imports.

export const RECORDED_PCD_SOURCE = "recorded_lidar_pcd";
export const RECORDED_PCD_LABEL = "真实 R1 Pro LiDAR SLAM 3D 点云 (r1pro_20260918 · 离线静态建图，非实时扫描)";

export function parsePcdBuffer(buffer) {
  const bytes = buffer instanceof Uint8Array ? buffer : new Uint8Array(buffer);
  // Locate header end ("\nDATA binary\n" or "\nDATA ascii\n")
  let headerEnd = -1;
  let lineStart = 0;
  const headerLines = [];
  for (let i = 0; i < Math.min(bytes.length, 4096); i++) {
    if (bytes[i] === 0x0a) {
      let end = i;
      if (end > lineStart && bytes[end - 1] === 0x0d) end--;
      const line = String.fromCharCode(...bytes.subarray(lineStart, end)).trim();
      headerLines.push(line);
      lineStart = i + 1;
      if (line.startsWith("DATA ")) {
        headerEnd = i + 1;
        break;
      }
    }
  }
  if (headerEnd < 0) throw new Error("Invalid PCD header: missing DATA line");

  const meta = {};
  for (const line of headerLines) {
    if (!line || line.startsWith("#")) continue;
    const sp = line.indexOf(" ");
    if (sp > 0) meta[line.slice(0, sp).toUpperCase()] = line.slice(sp + 1).trim();
  }

  const fields = (meta.FIELDS || "").split(/\s+/).filter(Boolean);
  const sizes = (meta.SIZE || "").split(/\s+/).map(Number);
  const types = (meta.TYPE || "").split(/\s+/);
  const counts = (meta.COUNT || fields.map(() => "1").join(" ")).split(/\s+/).map(Number);
  const totalPoints = parseInt(meta.POINTS || meta.WIDTH || "0", 10);
  const dataMode = (meta.DATA || "").toLowerCase();

  const xIdx = fields.indexOf("x");
  const yIdx = fields.indexOf("y");
  const zIdx = fields.indexOf("z");
  const iIdx = fields.indexOf("intensity");
  if (xIdx < 0 || yIdx < 0 || zIdx < 0) throw new Error("PCD missing x/y/z fields");

  const offsets = [];
  let stride = 0;
  for (let f = 0; f < fields.length; f++) {
    offsets.push(stride);
    stride += (sizes[f] || 4) * (counts[f] || 1);
  }

  const positions = new Float32Array(totalPoints * 3);
  const intensities = new Float32Array(totalPoints);
  let valid = 0;
  let minX = Infinity, maxX = -Infinity;
  let minY = Infinity, maxY = -Infinity;
  let minZ = Infinity, maxZ = -Infinity;
  let minI = Infinity, maxI = -Infinity;

  if (dataMode === "binary") {
    const view = new DataView(bytes.buffer, bytes.byteOffset + headerEnd, bytes.byteLength - headerEnd);
    const maxPts = Math.min(totalPoints, Math.floor(view.byteLength / stride));
    const xOff = offsets[xIdx], yOff = offsets[yIdx], zOff = offsets[zIdx];
    const iOff = iIdx >= 0 ? offsets[iIdx] : -1;
    const iType = iIdx >= 0 ? types[iIdx] : "";
    const iSize = iIdx >= 0 ? sizes[iIdx] : 4;

    for (let p = 0; p < maxPts; p++) {
      const base = p * stride;
      const x = view.getFloat32(base + xOff, true);
      const y = view.getFloat32(base + yOff, true);
      const z = view.getFloat32(base + zOff, true);
      if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) continue;
      let inten = 0;
      if (iOff >= 0) {
        if (iType === "F" && iSize === 4) inten = view.getFloat32(base + iOff, true);
        else if (iType === "U" && iSize === 1) inten = view.getUint8(base + iOff);
        else if (iType === "U" && iSize === 2) inten = view.getUint16(base + iOff, true);
        if (!Number.isFinite(inten)) inten = 0;
      }
      const o = valid * 3;
      positions[o] = x;
      positions[o + 1] = y;
      positions[o + 2] = z;
      intensities[valid] = inten;
      valid++;
      if (x < minX) minX = x; if (x > maxX) maxX = x;
      if (y < minY) minY = y; if (y > maxY) maxY = y;
      if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;
      if (inten < minI) minI = inten; if (inten > maxI) maxI = inten;
    }
  } else if (dataMode === "ascii") {
    const text = new TextDecoder("ascii").decode(bytes.subarray(headerEnd));
    for (const rawLine of text.split(/\r?\n/)) {
      const line = rawLine.trim();
      if (!line) continue;
      const parts = line.split(/\s+/);
      const x = Number(parts[xIdx]), y = Number(parts[yIdx]), z = Number(parts[zIdx]);
      if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) continue;
      const inten = iIdx >= 0 ? (Number(parts[iIdx]) || 0) : 0;
      if (valid >= totalPoints) break;
      const o = valid * 3;
      positions[o] = x;
      positions[o + 1] = y;
      positions[o + 2] = z;
      intensities[valid] = inten;
      valid++;
      if (x < minX) minX = x; if (x > maxX) maxX = x;
      if (y < minY) minY = y; if (y > maxY) maxY = y;
      if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;
      if (inten < minI) minI = inten; if (inten > maxI) maxI = inten;
    }
  } else {
    throw new Error(`Unsupported PCD DATA encoding: ${dataMode}`);
  }

  return {
    positions: valid === totalPoints ? positions : positions.subarray(0, valid * 3),
    intensities: valid === totalPoints ? intensities : intensities.subarray(0, valid),
    count: valid,
    fields,
    hasIntensity: iIdx >= 0 && maxI > minI,
    bounds: { minX, maxX, minY, maxY, minZ, maxZ, minI, maxI },
  };
}

function elevationColor(t) {
  // Smooth high-contrast engineering elevation ramp: slate-blue -> cyan -> emerald -> amber -> coral
  const c = Math.max(0, Math.min(1, t));
  if (c < 0.25) {
    const u = c / 0.25;
    return [0.22 + 0.04 * u, 0.34 + 0.32 * u, 0.56 + 0.26 * u];
  }
  if (c < 0.55) {
    const u = (c - 0.25) / 0.30;
    return [0.26 - 0.12 * u, 0.66 + 0.20 * u, 0.82 - 0.16 * u];
  }
  if (c < 0.82) {
    const u = (c - 0.55) / 0.27;
    return [0.14 + 0.78 * u, 0.86 - 0.14 * u, 0.66 - 0.48 * u];
  }
  const u = (c - 0.82) / 0.18;
  return [0.92 + 0.06 * u, 0.72 - 0.36 * u, 0.18 + 0.12 * u];
}

function intensityColor(t) {
  const c = Math.max(0, Math.min(1, t));
  return [0.15 + 0.85 * Math.pow(c, 0.7), 0.30 + 0.68 * c, 0.52 + 0.44 * (1 - Math.abs(c - 0.45))];
}

// Dual-stage offline 3D dynamic-obstacle / walking-ghost filter (zero API / network imports):
// Stage A: Pure 3D column density + vertical Z-span + 8-connected blob pruning (removes sparse
//          mid-air trails and small detached pedestrian blobs while keeping static walls, floor & ceiling).
// Stage B: Optional aligned 2D occupancy free-space carving (carves mid-body 3D ghosts hovering above
//          verified ray-carved 2D free corridors far from physical walls, plus honors 2D eraser strokes).
export function filterDynamicObstacles3D(parsed, {
  obsMinH = 0.16,
  obsMaxH = 1.98,
  colRes = 0.14,
  aligned2dGrid = null,
  mapYaml = null,
  freeCarveMarginM = 0.24,
} = {}) {
  if (!parsed || !parsed.count || !parsed.positions) return parsed;
  const pos = parsed.positions;
  const inten = parsed.intensities;
  const n = parsed.count;
  const b = parsed.bounds;

  // Estimate dominant ground floor Z from low points
  let zSum = 0;
  let zCnt = 0;
  const sampleStride = n > 200000 ? 4 : 1;
  for (let i = 0; i < n; i += sampleStride) {
    const z = pos[i * 3 + 2];
    if (z > -0.75 && z < 0.35) {
      zSum += z;
      zCnt++;
    }
  }
  const zFloor = zCnt > 0 ? zSum / zCnt : Math.max(-0.5, Math.min(0.15, b.minZ));

  // Stage A: 3D XY column obstacle density, vertical Z-span, and small-blob pruning
  const ox = b.minX - 0.5;
  const oy = b.minY - 0.5;
  const spanX = Math.max(1.0, b.maxX - b.minX + 1.0);
  const spanY = Math.max(1.0, b.maxY - b.minY + 1.0);
  let res = Math.max(0.08, colRes);
  if (Math.max(spanX, spanY) / res > 420) res = Math.max(spanX, spanY) / 420;
  const w = Math.max(8, Math.ceil(spanX / res));
  const h = Math.max(8, Math.ceil(spanY / res));
  const cells = w * h;
  const inv = 1.0 / res;

  const obsCnt = new Uint16Array(cells);
  const obsMinZ = new Float32Array(cells).fill(Infinity);
  const obsMaxZ = new Float32Array(cells).fill(-Infinity);

  for (let i = 0; i < n; i++) {
    const o = i * 3;
    const z = pos[o + 2];
    const rh = z - zFloor;
    if (rh < obsMinH || rh > obsMaxH) continue;
    const gx = Math.floor((pos[o] - ox) * inv);
    const gy = Math.floor((pos[o + 1] - oy) * inv);
    if (gx < 0 || gx >= w || gy < 0 || gy >= h) continue;
    const idx = gy * w + gx;
    if (obsCnt[idx] < 65535) obsCnt[idx]++;
    if (z < obsMinZ[idx]) obsMinZ[idx] = z;
    if (z > obsMaxZ[idx]) obsMaxZ[idx] = z;
  }

  // Density-adaptive thresholds for dense static walls vs transient pedestrian trails
  const denseThresh = n >= 250000 ? 20 : n >= 40000 ? 6 : 2;
  const tallThresh = n >= 250000 ? 9 : n >= 40000 ? 3 : 2;
  const maxBlobCells = n >= 40000 ? 18 : 6;
  const maxBlobPts = n >= 250000 ? 750 : n >= 40000 ? 180 : 30;

  const isStaticWall = new Uint8Array(cells);
  for (let i = 0; i < cells; i++) {
    if (obsCnt[i] === 0) continue;
    const span = obsMaxZ[i] - obsMinZ[i];
    if (obsCnt[i] >= denseThresh || (obsCnt[i] >= tallThresh && span >= 0.72)) {
      isStaticWall[i] = 1;
    }
  }

  // Connected-component pruning of small detached dynamic blobs inside corridors
  const vis = new Uint8Array(cells);
  const q = new Int32Array(cells);
  const n8 = [-1, 1, -w, w, -w - 1, -w + 1, w - 1, w + 1];
  let prunedBlobs = 0;

  for (let i = 0; i < cells; i++) {
    if (!isStaticWall[i] || vis[i]) continue;
    let hd = 0;
    let tl = 0;
    q[tl++] = i;
    vis[i] = 1;
    let totPts = 0;
    let touchesEdge = false;
    while (hd < tl) {
      const cur = q[hd++];
      totPts += obsCnt[cur];
      const cx = cur % w;
      const cy = (cur - cx) / w;
      if (cx === 0 || cx === w - 1 || cy === 0 || cy === h - 1) {
        touchesEdge = true;
        continue;
      }
      for (let d = 0; d < 8; d++) {
        const ni = cur + n8[d];
        if (isStaticWall[ni] && !vis[ni]) {
          vis[ni] = 1;
          q[tl++] = ni;
        }
      }
    }
    if (!touchesEdge && tl <= maxBlobCells && totPts < maxBlobPts) {
      prunedBlobs++;
      for (let k = 0; k < tl; k++) isStaticWall[q[k]] = 0;
    }
  }

  // 1-cell dilation keeps true structural wall edges crisp & continuous
  const keepObs = new Uint8Array(isStaticWall);
  for (let y = 1; y < h - 1; y++) {
    const row = y * w;
    for (let x = 1; x < w - 1; x++) {
      const idx = row + x;
      if (isStaticWall[idx]) continue;
      if (isStaticWall[idx - 1] || isStaticWall[idx + 1] || isStaticWall[idx - w] || isStaticWall[idx + w]) {
        keepObs[idx] = 1;
      }
    }
  }

  // Stage B: Precompute physical-wall protection mask on aligned 2D occupancy map (ignores virtual walls!)
  let freeCarveMask = null;
  let g2w = 0, g2h = 0, g2res = 0.05, g2ox = 0, g2oy = 0;
  if (aligned2dGrid && mapYaml && Array.isArray(mapYaml.origin)) {
    g2w = aligned2dGrid.width;
    g2h = aligned2dGrid.height;
    g2res = Math.max(1e-4, Number(mapYaml.resolution) || aligned2dGrid.resolution || 0.05);
    g2ox = Number(mapYaml.origin[0]) || 0;
    g2oy = Number(mapYaml.origin[1]) || 0;
    const pCls = aligned2dGrid.physicalCls || aligned2dGrid.cls;
    if (pCls && pCls.length === g2w * g2h) {
      // Dilate physical 2D walls by freeCarveMarginM so real wall points are strictly protected
      const radPx = Math.max(1, Math.round(freeCarveMarginM / g2res));
      const nearWall = new Uint8Array(g2w * g2h);
      // Fast separable horizontal + vertical box dilation of physical walls
      const horiz = new Uint8Array(g2w * g2h);
      for (let y = 0; y < g2h; y++) {
        const row = y * g2w;
        let run = 0;
        for (let x = 0; x < g2w; x++) {
          if (pCls[row + x] === 2) run = radPx + 1;
          else if (run > 0) run--;
          if (run > 0) horiz[row + x] = 1;
        }
        run = 0;
        for (let x = g2w - 1; x >= 0; x--) {
          if (pCls[row + x] === 2) run = radPx + 1;
          else if (run > 0) run--;
          if (run > 0) horiz[row + x] = 1;
        }
      }
      for (let x = 0; x < g2w; x++) {
        let run = 0;
        for (let y = 0; y < g2h; y++) {
          const idx = y * g2w + x;
          if (horiz[idx]) run = radPx + 1;
          else if (run > 0) run--;
          if (run > 0) nearWall[idx] = 1;
        }
        run = 0;
        for (let y = g2h - 1; y >= 0; y--) {
          const idx = y * g2w + x;
          if (horiz[idx]) run = radPx + 1;
          else if (run > 0) run--;
          if (run > 0) nearWall[idx] = 1;
        }
      }
      freeCarveMask = new Uint8Array(g2w * g2h);
      for (let i = 0; i < g2w * g2h; i++) {
        if (pCls[i] === 1 && !nearWall[i]) freeCarveMask[i] = 1;
      }
    }
  }

  // Pass 2: filter points cleanly into compact Float32Arrays
  const keepFlags = new Uint8Array(n);
  let kept = 0;
  let dynamicRemoved = 0;
  let carvedByFreeSpace = 0;

  for (let i = 0; i < n; i++) {
    const o = i * 3;
    const x = pos[o];
    const y = pos[o + 1];
    const z = pos[o + 2];
    const rh = z - zFloor;
    let isDynamic = false;

    if (rh >= obsMinH && rh <= obsMaxH) {
      // Check Stage B (ray-carved 2D open corridor free space) first
      if (freeCarveMask) {
        const px = Math.floor((x - g2ox) / g2res);
        const py = Math.floor(g2h - (y - g2oy) / g2res);
        if (px >= 0 && px < g2w && py >= 0 && py < g2h && freeCarveMask[py * g2w + px]) {
          isDynamic = true;
          carvedByFreeSpace++;
        }
      }
      // Check Stage A (3D geometric density + detached blob pruner)
      if (!isDynamic) {
        const gx = Math.floor((x - ox) * inv);
        const gy = Math.floor((y - oy) * inv);
        if (gx >= 0 && gx < w && gy >= 0 && gy < h && !keepObs[gy * w + gx]) {
          isDynamic = true;
        }
      }
    }

    if (isDynamic) {
      dynamicRemoved++;
    } else {
      keepFlags[i] = 1;
      kept++;
    }
  }

  const outPos = new Float32Array(kept * 3);
  const outInt = new Float32Array(kept);
  let k = 0;
  for (let i = 0; i < n; i++) {
    if (!keepFlags[i]) continue;
    const srcO = i * 3;
    const dstO = k * 3;
    outPos[dstO] = pos[srcO];
    outPos[dstO + 1] = pos[srcO + 1];
    outPos[dstO + 2] = pos[srcO + 2];
    outInt[k] = inten ? inten[i] : 0;
    k++;
  }

  return {
    positions: outPos,
    intensities: outInt,
    count: kept,
    rawCount: n,
    fields: parsed.fields,
    hasIntensity: parsed.hasIntensity,
    bounds: parsed.bounds,
    dynamicRemoved,
    prunedBlobs,
    carvedByFreeSpace,
  };
}

function buildLocalGroundField(positions, count, bounds, tileRes = 0.5) {
  let zSum = 0;
  let zCnt = 0;
  const stride = count > 200000 ? 4 : 1;
  for (let i = 0; i < count; i += stride) {
    const z = positions[i * 3 + 2];
    if (z > -0.75 && z < 0.35) {
      zSum += z;
      zCnt++;
    }
  }
  const defaultFloorZ = zCnt > 0 ? zSum / zCnt : Math.max(-0.5, Math.min(0.15, bounds.minZ));
  const ox = bounds.minX - 0.5;
  const oy = bounds.minY - 0.5;
  const spanX = Math.max(1.0, bounds.maxX - bounds.minX + 1.0);
  const spanY = Math.max(1.0, bounds.maxY - bounds.minY + 1.0);
  const res = Math.max(0.25, tileRes);
  const w = Math.max(4, Math.ceil(spanX / res));
  const h = Math.max(4, Math.ceil(spanY / res));
  const inv = 1.0 / res;
  const n = w * h;

  const sumZ = new Float32Array(n);
  const cntZ = new Uint16Array(n);
  // Reject sub-floor mirror reflections (< defaultFloorZ - 0.18) when estimating real floor surface!
  const zLo = defaultFloorZ - 0.18;
  const zHi = defaultFloorZ + 0.32;

  for (let i = 0; i < count; i++) {
    const o = i * 3;
    const z = positions[o + 2];
    if (z < zLo || z > zHi) continue;
    const gx = Math.floor((positions[o] - ox) * inv);
    const gy = Math.floor((positions[o + 1] - oy) * inv);
    if (gx < 0 || gx >= w || gy < 0 || gy >= h) continue;
    const idx = gy * w + gx;
    if (cntZ[idx] < 65535) {
      sumZ[idx] += z;
      cntZ[idx]++;
    }
  }

  // Smooth & fill empty tiles from 3x3 valid neighbors for tilt-adaptive ground plane
  const floorZ = new Float32Array(n);
  for (let y = 0; y < h; y++) {
    const row = y * w;
    for (let x = 0; x < w; x++) {
      const idx = row + x;
      if (cntZ[idx] > 0) {
        floorZ[idx] = Math.max(defaultFloorZ - 0.08, sumZ[idx] / cntZ[idx]);
        continue;
      }
      let sum = 0;
      let cnt = 0;
      for (let dy = -1; dy <= 1; dy++) {
        const ny = y + dy;
        if (ny < 0 || ny >= h) continue;
        const nr = ny * w;
        for (let dx = -1; dx <= 1; dx++) {
          const nx = x + dx;
          if (nx < 0 || nx >= w) continue;
          const ni = nr + nx;
          if (cntZ[ni] > 0) {
            sum += sumZ[ni] / cntZ[ni];
            cnt++;
          }
        }
      }
      floorZ[idx] = cnt > 0 ? Math.max(defaultFloorZ - 0.08, sum / cnt) : defaultFloorZ;
    }
  }

  return { ox, oy, inv, w, h, floorZ, defaultFloorZ };
}

export function buildColoredPcdCloud(parsed, {
  colorMode = "height",
  zMin = -1.0,
  zMax = 2.6,
  hideGround = false,
  groundMargin = 0.22,
  label = RECORDED_PCD_LABEL,
} = {}) {
  const srcPos = parsed.positions;
  const srcInt = parsed.intensities;
  const n = parsed.count;
  const b = parsed.bounds;
  const groundField = hideGround ? buildLocalGroundField(srcPos, n, b, 0.5) : null;
  const zBase = hideGround
    ? (groundField.defaultFloorZ + groundMargin)
    : Math.max(zMin, b.minZ);
  const zSpan = Math.max(0.25, Math.min(zMax, b.maxZ) - zBase);
  const iSpan = Math.max(1e-3, (b.maxI || 1) - (b.minI || 0));

  const keepMask = new Uint8Array(n);
  let kept = 0;
  let groundRemoved = 0;
  for (let i = 0; i < n; i++) {
    const idx = i * 3;
    const z = srcPos[idx + 2];
    if (z < zMin || z > zMax) continue;
    if (groundField) {
      const gx = Math.floor((srcPos[idx] - groundField.ox) * groundField.inv);
      const gy = Math.floor((srcPos[idx + 1] - groundField.oy) * groundField.inv);
      const localFloor = (gx >= 0 && gx < groundField.w && gy >= 0 && gy < groundField.h)
        ? groundField.floorZ[gy * groundField.w + gx]
        : groundField.defaultFloorZ;
      if (z < localFloor + groundMargin) {
        groundRemoved++;
        continue;
      }
    }
    keepMask[i] = 1;
    kept++;
  }

  const positions = new Float32Array(kept * 3);
  const colors = new Float32Array(kept * 3);
  let out = 0;

  for (let i = 0; i < n; i++) {
    if (!keepMask[i]) continue;
    const idx = i * 3;
    const z = srcPos[idx + 2];
    const o = out * 3;
    positions[o] = srcPos[idx];
    positions[o + 1] = srcPos[idx + 1];
    positions[o + 2] = z;

    let r, g, bl;
    if (colorMode === "intensity" && parsed.hasIntensity) {
      const t = Math.min(1, Math.max(0, (srcInt[i] - b.minI) / Math.min(iSpan, 120)));
      [r, g, bl] = intensityColor(t);
    } else if (colorMode === "semantic") {
      if (z < 0.18) {
        r = 0.38; g = 0.46; bl = 0.56; // Traversable ground
      } else if (z <= 2.1) {
        r = 0.14; g = 0.80; bl = 0.72; // Walls & obstacles
      } else {
        r = 0.94; g = 0.65; bl = 0.24; // Ceiling / overhead
      }
    } else {
      [r, g, bl] = elevationColor((z - zBase) / zSpan);
    }
    colors[o] = r;
    colors[o + 1] = g;
    colors[o + 2] = bl;
    out++;
  }

  return {
    positions,
    colors,
    count: kept,
    totalCount: parsed.rawCount || n,
    dynamicRemoved: parsed.dynamicRemoved || 0,
    groundRemoved,
    source: RECORDED_PCD_SOURCE,
    live: false,
    bounds: b,
    label,
  };
}
