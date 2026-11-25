// Occupancy-grid helpers for the map page. No robot API or live sensor imports.
export const MOCK_CLOUD_SOURCE = "mock_from_occupancy";
export const MOCK_CLOUD_LABEL = "Mock 示例点云，由本地 2D 占用栅格生成，不是真实激光雷达或现场采集；真实三维点云数据待接入。";

export function occupancyClass(luma, { occupiedThresh = 0.65, freeThresh = 0.25, negate = 0, mode = "trinary" } = {}) {
  // mx_map.yaml is mode: trinary. Standard ROS values: 0 occupied, 254 free, 205 unknown.
  if (mode === "trinary") {
    if (luma <= 10) return "occupied";
    if (luma >= 250) return "free";
    return "unknown";
  }
  const value = (negate ? luma : 255 - luma) / 255;
  if (value > occupiedThresh) return "occupied";
  if (value < freeThresh) return "free";
  return "unknown";
}

export function occupancyToPointCloud({ luma, width, height, resolution, origin, wallHeight = 1.6, wallStep = 0.2, floorStride = 6 }) {
  const [ox, oy] = origin;
  const positions = [];
  const colors = [];
  let occupied = 0, free = 0, unknown = 0;
  const wallLevels = [];
  for (let z = 0; z <= wallHeight + 1e-9; z += wallStep) wallLevels.push(Number(z.toFixed(3)));
  for (let row = 0; row < height; row++) {
    for (let col = 0; col < width; col++) {
      const kind = occupancyClass(luma[row * width + col]);
      const x = ox + (col + 0.5) * resolution;
      const y = oy + (height - 1 - row + 0.5) * resolution;
      if (kind === "unknown") { unknown++; continue; }
      if (kind === "occupied") {
        occupied++;
        for (const z of wallLevels) {
          positions.push(x, y, z);
          const t = wallHeight ? z / wallHeight : 0;
          colors.push(0.08 + 0.15 * t, 0.45 + 0.4 * t, 0.48 + 0.2 * t);
        }
      } else {
        free++;
        if (row % floorStride === 0 && col % floorStride === 0) {
          positions.push(x, y, 0);
          colors.push(0.72, 0.76, 0.8);
        }
      }
    }
  }
  return {
    positions: new Float32Array(positions),
    colors: new Float32Array(colors),
    count: positions.length / 3,
    occupiedCells: occupied,
    freeCells: free,
    unknownCells: unknown,
    source: MOCK_CLOUD_SOURCE,
    live: false,
    frame: "map",
    label: MOCK_CLOUD_LABEL,
  };
}

export function lumaFromRgba(data) {
  const luma = new Uint8Array(data.length / 4);
  for (let i = 0; i < luma.length; i++) luma[i] = data[i * 4];
  return luma;
}

export function describeMapCloud(supported, reason = "") {
  if (!supported) {
    return { supported: false, reason, pointcloud: { available: false, live: false, source: null } };
  }
  return {
    supported: true,
    kind: "occupancy_grid",
    pointcloud: { available: true, live: false, source: MOCK_CLOUD_SOURCE, frame: "map", label: MOCK_CLOUD_LABEL },
  };
}
