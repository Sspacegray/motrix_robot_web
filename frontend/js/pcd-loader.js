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

export function buildColoredPcdCloud(parsed, { colorMode = "height", zMin = -1.0, zMax = 2.6, label = RECORDED_PCD_LABEL } = {}) {
  const srcPos = parsed.positions;
  const srcInt = parsed.intensities;
  const n = parsed.count;
  const b = parsed.bounds;
  const zSpan = Math.max(0.25, Math.min(zMax, b.maxZ) - Math.max(zMin, b.minZ));
  const zBase = Math.max(zMin, b.minZ);
  const iSpan = Math.max(1e-3, (b.maxI || 1) - (b.minI || 0));

  // Count matching points first to allocate exact typed arrays (fast, zero GC spikes)
  let kept = 0;
  for (let i = 0; i < n; i++) {
    const z = srcPos[i * 3 + 2];
    if (z >= zMin && z <= zMax) kept++;
  }

  const positions = new Float32Array(kept * 3);
  const colors = new Float32Array(kept * 3);
  let out = 0;

  for (let i = 0; i < n; i++) {
    const idx = i * 3;
    const z = srcPos[idx + 2];
    if (z < zMin || z > zMax) continue;
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
    totalCount: n,
    source: RECORDED_PCD_SOURCE,
    live: false,
    bounds: b,
    label,
  };
}
