// Tab 3: R1 Pro Real 2D Occupancy Maps, Real 3D LiDAR Point Cloud (.pcd), Waypoints & Chassis Teleop
import { api } from "../api.js";
import { occupancyToPointCloud, lumaFromRgba, MOCK_CLOUD_LABEL } from "../map-cloud.js";
import { parsePcdBuffer, buildColoredPcdCloud, RECORDED_PCD_LABEL } from "../pcd-loader.js";
import { createMapCloudViewer } from "../map-cloud-viewer.js";
import { bindMapInteractions } from "../map-interactions.js";
import { analyzeOccupancyGrid, queryCellInfo, castLidarScan, planGridPathAStar } from "../map-planner-2d.js";
import { escapeHtml } from "../ui-format.js";
import { notify } from "../notify.js";

export function initNavigationTab(container) {
  let mapImage = null;
  let mapCatalog = [];
  let activeMap = {
    id: "fast_slam_20260914",
    name: "R1 Pro Fast-SLAM Plus 高精地图 (bag_20260914 · 0.05m/pix + 98.1万点 3D 点云)",
    image_url: "/assets/maps/fast_slam_20260914/bag_20260914_map.png",
    yaml: { resolution: 0.05, origin: [-14.8, -29.1, 0], width: 709, height: 755 },
    pointcloud: {
      available: true,
      live: false,
      source: "recorded_lidar_pcd",
      pcd_url: "/assets/maps/fast_slam_20260914/virtual_machine_map.pcd",
      label: RECORDED_PCD_LABEL,
    },
  };
  let mapYaml = { ...activeMap.yaml };
  let parsedPcdCache = new Map();
  let currentParsedPcd = null;
  let cloudColorMode = "height";
  let cloudZMax = 2.6;
  let goalPose = null; // { x, y, yaw }
  let isDraggingGoal = false;
  let dragStartPos = null;
  let currentNavMode = "MANUAL";
  let mapSupported = false;
  let mapCatalogLoadedFor = null;
  let waypoints = [];
  let holdTimer = null;

  container.innerHTML = `
    <!-- Non-navigation Robot Banner -->
    <div id="navNotSupportedCard" class="card" style="display:none; border-left: 4px solid var(--warning); background:#fffbeb;">
      <div style="font-weight:600; color:#b45309; margin-bottom:4px;">机型未装配移动底盘与导航套件</div>
      <div style="font-size:13px; color:#78350f;">
        当前选中机型（如 Piper 双臂台架或 Dexforce W1 人形）未配置移动底盘。2D 占用栅格与 3D 点云入口保留可见，但按能力禁用，不把 R1 的地图数据当作本机采集。
      </div>
    </div>

    <div id="navMainWrapper" class="grid-2" style="grid-template-columns: 1fr 390px; align-items: start;">
      <!-- Left: 2D Real Map / 3D Real LiDAR Point Cloud & Waypoints -->
      <div style="display:flex; flex-direction:column; gap:16px;">
        <div class="card" style="margin-bottom:0;">
          <div class="card-header" style="padding-bottom:10px; flex-wrap:wrap; gap:8px;">
            <div>
              <div style="display:flex; align-items:center; gap:8px; flex-wrap:wrap;">
                <span class="card-title">地图导航</span>
                <select id="navMapSelect" class="select-clean" aria-label="选择地图数据集" style="font-size:12px; padding:3px 8px; max-width:380px;">
                  <option value="fast_slam_20260914">R1 Pro Fast-SLAM Plus 高精地图 (bag_20260914 · 0.05m/pix + 98.1万点)</option>
                  <option value="mx_map">R1 Pro pc10 Web 现场 2D 地图 (mx_map · 0.05m/pix)</option>
                  <option value="r1pro_20260918">R1 Pro Robust-SLAM 压缩对照图 (r1pro_20260918 · 0.15m/pix)</option>
                </select>
                <span class="badge badge-info" id="navResolutionBadge">分辨率: 0.05m/pix</span>
              </div>
              <div class="map-view-tabs" role="tablist" aria-label="地图视图">
                <button type="button" class="btn btn-sm sub-tab-btn" data-map-view="occupancy2d" aria-selected="true">2D 真实栅格地图</button>
                <button type="button" class="btn btn-sm sub-tab-btn" data-map-view="cloud3d" aria-selected="false">3D 真实点云地图</button>
              </div>
            </div>
            <div style="display:flex; align-items:center; gap:8px;">
              <button class="btn btn-sm" id="navResetViewBtn">重置视角</button>
              <button class="btn btn-sm btn-danger" id="navCancelGoalBtn">取消当前导航</button>
            </div>
          </div>

          <!-- 3D Point Cloud Controls Strip (visible only in 3D view) -->
          <div id="navCloudControls" class="cloud-controls-strip" hidden>
            <label style="display:inline-flex; align-items:center; gap:6px; font-size:12px;">
              着色模式
              <select id="cloudColorModeSelect" class="select-clean" style="font-size:11px; padding:2px 6px;">
                <option value="height" selected>高程色谱 (Height)</option>
                <option value="intensity">反射强度 (Intensity)</option>
                <option value="semantic">地面/墙体/顶棚分层</option>
              </select>
            </label>
            <label style="display:inline-flex; align-items:center; gap:6px; font-size:12px;">
              顶棚切片 Z≤<strong id="cloudZMaxVal" style="font-family:var(--font-mono);">2.6m</strong>
              <input type="range" id="cloudZMaxSlider" min="0.4" max="5.2" step="0.1" value="2.6" style="width:110px; accent-color:var(--accent);">
            </label>
            <label style="display:inline-flex; align-items:center; gap:6px; font-size:12px;">
              点大小
              <input type="range" id="cloudPointSizeSlider" min="0.03" max="0.22" step="0.01" value="0.07" style="width:80px; accent-color:var(--accent);">
            </label>
          </div>

          <!-- Canvas Container -->
          <p class="preview-field-note" id="mapCloudNote">2D 战术导航图 · 实时 Nav2 代价膨胀 + 激光扫描 + A* 避障路径预览；下发仍需显式操作与 Lease。</p>
          <div id="navMapStage" class="nav-map-stage">
            <canvas id="navMapCanvas" width="760" height="540" style="width:100%; height:100%; display:block; cursor:crosshair;"></canvas>
            <canvas id="navCloudCanvas" class="nav-cloud-canvas" width="760" height="540" hidden></canvas>
            <div id="nav2dToolbar" class="nav-2d-toolbar" role="toolbar" aria-label="2D 地图视图控制">
              <button type="button" class="nav-map-tool-btn" id="navZoomInBtn" title="放大地图 (+)">＋</button>
              <button type="button" class="nav-map-tool-btn" id="navZoomOutBtn" title="缩小地图 (-)">－</button>
              <button type="button" class="nav-map-tool-btn" id="navFitMapBtn" title="1:1 等比自适应完整建图区域">适应</button>
              <span class="nav-tool-sep" aria-hidden="true"></span>
              <button type="button" class="nav-map-tool-btn active" id="navCostmapBtn" aria-pressed="true" title="显示/隐藏 Nav2 障碍物膨胀代价层">代价层</button>
              <button type="button" class="nav-map-tool-btn active" id="navLidarBtn" aria-pressed="true" title="显示/隐藏 360° 实时激光扫描与前视相机视锥">雷达</button>
              <button type="button" class="nav-map-tool-btn" id="navMeasureBtn" aria-pressed="false" title="两点物理测距工具 (按 Esc 退出)">测距</button>
              <button type="button" class="nav-map-tool-btn" id="navMapGridBtn" aria-pressed="false" title="显示/隐藏世界坐标米制网格">网格</button>
              <button type="button" class="nav-map-tool-btn" id="navMapThemeBtn" aria-pressed="false" title="切换深色战术图 / 浅色室内图">亮底</button>
            </div>
            <div id="navRouteBanner" class="nav-route-pill" hidden></div>
            <div id="navCloudHud" class="nav-cloud-hud" hidden></div>
            <div class="nav-map-overlay-chip" id="navCoordsOverlay">
              X: 0.00m | Y: 0.00m | 悬停探测壁距
            </div>
            <div id="navMapLegend" class="nav-map-legend">
              <div>青色：位姿/雷达 · 红蓝环：Nav2 代价层 · 橙色：目标</div>
            </div>
          </div>

          <!-- Manual Goal Inputs Bar -->
          <div style="margin-top:12px; display:flex; gap:8px; align-items:center; flex-wrap:wrap; background:var(--bg-subtle); padding:10px 14px; border-radius:var(--radius-sm);">
            <span style="font-size:12px; font-weight:600; color:var(--text-muted);">目标位姿:</span>
            <div style="display:flex; align-items:center; gap:4px;">
              <label style="font-size:12px; color:var(--text-muted);">X</label>
              <input type="number" id="navGoalInputX" step="0.1" value="0.0" style="width:72px; padding:4px 6px; font-size:12px; border:1px solid var(--border); border-radius:var(--radius-sm); font-family:var(--font-mono);">
            </div>
            <div style="display:flex; align-items:center; gap:4px;">
              <label style="font-size:12px; color:var(--text-muted);">Y</label>
              <input type="number" id="navGoalInputY" step="0.1" value="0.0" style="width:72px; padding:4px 6px; font-size:12px; border:1px solid var(--border); border-radius:var(--radius-sm); font-family:var(--font-mono);">
            </div>
            <div style="display:flex; align-items:center; gap:4px;">
              <label style="font-size:12px; color:var(--text-muted);">Yaw°</label>
              <input type="number" id="navGoalInputYaw" step="5" value="0" style="width:62px; padding:4px 6px; font-size:12px; border:1px solid var(--border); border-radius:var(--radius-sm); font-family:var(--font-mono);">
            </div>
            <button class="btn btn-sm" id="navSaveWaypointBtn" title="将当前坐标保存为常用导航点">存为航点</button>
            <button class="btn btn-sm btn-primary" id="navSendGoalBtn" style="margin-left:auto;">下发导航目标</button>
          </div>
        </div>

        <!-- Saved Navigation Waypoints Card -->
        <div class="card" style="margin-bottom:0;">
          <div class="card-header" style="margin-bottom:8px; padding-bottom:6px;">
            <span class="card-title" style="font-size:13px;">常用导航点位 (Saved Waypoints)</span>
            <span style="font-size:11px; color:var(--text-muted);">点击点位自动载入目标坐标并在 2D/3D 地图标记</span>
          </div>
          <div id="navWaypointsList" style="display:flex; flex-wrap:wrap; gap:8px;">
            <span style="font-size:12px; color:var(--text-muted);">加载点位...</span>
          </div>
        </div>
      </div>

      <!-- Right: Navigation Mode State Machine & Chassis Teleop -->
      <div style="display:flex; flex-direction:column; gap:16px;">
        <!-- Control Authority & Mode State Machine -->
        <div class="card" style="margin-bottom:0;">
          <div class="card-header" style="margin-bottom:12px;">
            <span class="card-title">底盘控制状态机</span>
            <span class="badge badge-info" id="navCurrentModeBadge">MANUAL</span>
          </div>
          <div style="font-size:12px; color:var(--text-muted); margin-bottom:12px;">
            严格遵循导航状态机：<code style="font-size:11px;">MANUAL ⇄ SWITCHING ⇄ NAVIGATION ⇄ FAULT</code>。手动底盘下发必须在 MANUAL 模式；自主路径跟踪必须在 NAVIGATION 模式。
          </div>
          <div style="display:flex; gap:8px;">
            <button class="btn btn-sm" id="navSetManualBtn" style="flex:1;">切至 MANUAL</button>
            <button class="btn btn-sm btn-primary" id="navSetNavModeBtn" style="flex:1;">切至 NAVIGATION</button>
          </div>
          <div style="margin-top:12px; padding-top:12px; border-top:1px solid var(--border); display:flex; justify-content:space-between; align-items:center;">
            <span style="font-size:13px; font-weight:500;">原地旋转自定位 (Spin)</span>
            <button class="btn btn-sm" id="navSpinToggleBtn">启动定位旋转</button>
          </div>
        </div>

        <!-- Manual Chassis Teleop D-Pad (Hold-to-Move Dead-man Switch) -->
        <div class="card" style="margin-bottom:0;">
          <div class="card-header" style="margin-bottom:10px;">
            <span class="card-title">底盘手动微调 (Dead-man Teleop)</span>
            <span class="badge badge-idle" id="navTeleopStatusBadge">MANUAL 模式可控</span>
          </div>
          <div style="font-size:12px; color:var(--text-muted); margin-bottom:12px;">
            支持<strong>按住持续微动、松手即停</strong>（死人开关语义）与键盘 <kbd>W</kbd>/<kbd>S</kbd>/<kbd>A</kbd>/<kbd>D</kbd>/<kbd>Q</kbd>/<kbd>E</kbd>/<kbd>X</kbd> 控制。
          </div>

          <div style="margin-bottom:14px; background:var(--bg-subtle); padding:10px; border-radius:var(--radius-sm); font-size:12px;">
            <div style="display:flex; justify-content:space-between; margin-bottom:4px;">
              <span>线速度设定 (Linear Vx/Vy):</span>
              <strong id="navLinearSpeedVal">0.25 m/s</strong>
            </div>
            <input type="range" id="navLinearSpeedSlider" min="0.05" max="0.8" step="0.05" value="0.25" style="width:100%; accent-color:var(--accent);">

            <div style="display:flex; justify-content:space-between; margin-top:8px; margin-bottom:4px;">
              <span>角速度设定 (Angular Omega):</span>
              <strong id="navAngularSpeedVal">0.40 rad/s</strong>
            </div>
            <input type="range" id="navAngularSpeedSlider" min="0.1" max="1.5" step="0.05" value="0.40" style="width:100%; accent-color:var(--accent);">
          </div>

          <div style="display:grid; grid-template-columns: repeat(3, 1fr); gap:8px; width:220px; margin:0 auto;">
            <button class="btn btn-sm" id="btnChassisRotateLeft" title="按住逆时针旋转 (Q)">↺ Q</button>
            <button class="btn btn-sm btn-primary" id="btnChassisForward" title="按住前进 (W)">▲ W</button>
            <button class="btn btn-sm" id="btnChassisRotateRight" title="按住顺时针旋转 (E)">↻ E</button>

            <button class="btn btn-sm btn-primary" id="btnChassisLeft" title="按住左平移 (A)">◀ A</button>
            <button class="btn btn-sm btn-danger" id="btnChassisStop" title="立即归零速度 (X)">■ 停</button>
            <button class="btn btn-sm btn-primary" id="btnChassisRight" title="按住右平移 (D)">▶ D</button>

            <div></div>
            <button class="btn btn-sm btn-primary" id="btnChassisBackward" title="按住后退 (S)">▼ S</button>
            <div></div>
          </div>
        </div>

        <!-- Real-time Odometry & Pose Card -->
        <div class="card" style="margin-bottom:0;">
          <div class="card-header" style="margin-bottom:8px;">
            <span class="card-title">里程计与位姿遥测</span>
          </div>
          <div style="display:grid; grid-template-columns: 1fr 1fr; gap:10px; font-size:12px; font-family:var(--font-mono);">
            <div style="background:var(--bg-subtle); padding:6px 10px; border-radius:var(--radius-sm);">
              <div style="color:var(--text-muted); font-size:11px;">Pose X / Y:</div>
              <strong id="navOdomXY">0.000m, 0.000m</strong>
            </div>
            <div style="background:var(--bg-subtle); padding:6px 10px; border-radius:var(--radius-sm);">
              <div style="color:var(--text-muted); font-size:11px;">Heading Yaw:</div>
              <strong id="navOdomYaw">0.0°</strong>
            </div>
            <div style="background:var(--bg-subtle); padding:6px 10px; border-radius:var(--radius-sm);">
              <div style="color:var(--text-muted); font-size:11px;">Velocity Vx/Vy:</div>
              <strong id="navOdomVel">0.00, 0.00 m/s</strong>
            </div>
            <div style="background:var(--bg-subtle); padding:6px 10px; border-radius:var(--radius-sm);">
              <div style="color:var(--text-muted); font-size:11px;">Angular Omega:</div>
              <strong id="navOdomOmega">0.00 rad/s</strong>
            </div>
          </div>
        </div>
      </div>
    </div>
  `;

  const stageEl = container.querySelector("#navMapStage");
  const canvas = container.querySelector("#navMapCanvas");
  const ctx = canvas.getContext("2d");
  let viewW = 760;
  let viewH = 540;
  let canvasDpr = 1;
  let hasFittedInitialView = false;
  let mapLightTheme = false;
  let mapGridEnabled = false;
  let costmapEnabled = true;
  let lidarEnabled = true;
  let measureMode = false;
  let measurePoints = []; // up to 2 points [{ wx, wy }]
  let measureHover = null; // { wx, wy }
  let previewRoute = null; // { ok, reason, pointsWorld, lengthM, minClearanceM, etaS }
  let lastRouteKey = null;
  let lidarCache = { key: null, rays: [] };
  let animRafId = null;
  let animPhase = 0;
  let lastAnimTs = 0;
  let styledMapCache = { key: null, dark: null, light: null, bounds: null, grid: null };
  let scale = 1.0;
  let offsetX = 100;
  let offsetY = 50;
  let isPanning = false;
  let panStartX = 0;
  let panStartY = 0;

  const cloudCanvas = container.querySelector("#navCloudCanvas");
  const cloudViewer = createMapCloudViewer({ canvas: cloudCanvas, hud: container.querySelector("#navCloudHud") });
  let mapView = "occupancy2d";
  let cloudReadyForMapId = null;

  function buildStyledMap(img) {
    if (!img || !img.complete || !img.naturalWidth || !img.naturalHeight) return null;
    const res = Number(mapYaml.resolution) || 0.05;
    const key = `${activeMap?.id || ""}|${img.src}|${img.naturalWidth}x${img.naturalHeight}|${res}`;
    if (styledMapCache.key === key && styledMapCache.dark && styledMapCache.light && styledMapCache.grid) {
      return styledMapCache;
    }
    const w = img.naturalWidth;
    const h = img.naturalHeight;
    const probe = document.createElement("canvas");
    probe.width = w;
    probe.height = h;
    const pctx = probe.getContext("2d", { willReadFrequently: true });
    pctx.drawImage(img, 0, 0);
    const src = pctx.getImageData(0, 0, w, h).data;

    // 1. Analyze ROS occupancy grid + exact 2-pass Chamfer Euclidean Distance Transform (meters to nearest wall)
    const grid = analyzeOccupancyGrid(src, w, h, res);
    const { cls, distM, bounds } = grid;

    // 2. Precompute 3 crisp layered offscreen textures per theme:
    //    - floor: EDT ambient-occlusion shaded architectural slab
    //    - costmap: Nav2 lethal chassis radius (<=0.34m) + smooth inflation field (0.34m..0.90m)
    //    - wall: 2.5D extruded wall relief (drop extrusion + bright top cap)
    const makeLayers = (isLight) => {
      const floorCanvas = document.createElement("canvas");
      floorCanvas.width = w;
      floorCanvas.height = h;
      const fctx = floorCanvas.getContext("2d");
      const floorImg = fctx.createImageData(w, h);
      const fdst = floorImg.data;

      const costCanvas = document.createElement("canvas");
      costCanvas.width = w;
      costCanvas.height = h;
      const cctx = costCanvas.getContext("2d");
      const costImg = cctx.createImageData(w, h);
      const cdst = costImg.data;

      const wallCanvas = document.createElement("canvas");
      wallCanvas.width = w;
      wallCanvas.height = h;
      const wctx = wallCanvas.getContext("2d");
      const wallImg = wctx.createImageData(w, h);
      const wdst = wallImg.data;

      for (let y = 0; y < h; y++) {
        const row = y * w;
        for (let x = 0; x < w; x++) {
          const idx = row + x;
          const p = idx * 4;
          const c = cls[idx];
          const d = distM[idx];

          // Layer A: Floor with subtle EDT ambient-occlusion shading (darker near walls, brighter in open atrium)
          if (c !== 0) {
            let unkN = 0;
            if (x > 0 && cls[idx - 1] === 0) unkN++;
            if (x + 1 < w && cls[idx + 1] === 0) unkN++;
            if (y > 0 && cls[idx - w] === 0) unkN++;
            if (y + 1 < h && cls[idx + w] === 0) unkN++;

            const ao = Math.min(1, d / 2.2); // 0 right against wall -> 1 in open corridor/hall
            if (isLight) {
              fdst[p] = Math.round(239 + ao * 16);
              fdst[p + 1] = Math.round(244 + ao * 11);
              fdst[p + 2] = Math.round(250 + ao * 5);
            } else {
              fdst[p] = Math.round(18 + ao * 14);
              fdst[p + 1] = Math.round(29 + ao * 19);
              fdst[p + 2] = Math.round(47 + ao * 26);
            }
            fdst[p + 3] = unkN >= 3 ? 170 : 255;

            // Layer B: Real Nav2 Obstacle Inflation Costmap on navigable free space
            if (c === 1 && d > 0 && d <= 0.90) {
              if (d <= 0.34) {
                // Lethal chassis footprint buffer (R1 Pro radius ~0.34m)
                const k = 1 - d / 0.34;
                if (isLight) {
                  cdst[p] = 225;
                  cdst[p + 1] = 29;
                  cdst[p + 2] = 72;
                  cdst[p + 3] = Math.round(32 + k * 66);
                } else {
                  cdst[p] = 244;
                  cdst[p + 1] = 63;
                  cdst[p + 2] = 94;
                  cdst[p + 3] = Math.round(48 + k * 92);
                }
              } else {
                // Smooth Nav2 clearance inflation decay (0.34m .. 0.90m)
                const t = 1 - (d - 0.34) / 0.56;
                const e = t * t;
                const onIso = Math.abs(d - 0.38) < res * 0.72;
                if (isLight) {
                  cdst[p] = onIso ? 99 : Math.round(56 + t * 43);
                  cdst[p + 1] = onIso ? 102 : Math.round(130 - t * 28);
                  cdst[p + 2] = onIso ? 241 : Math.round(246 - t * 5);
                  cdst[p + 3] = onIso ? 86 : Math.round(e * 62);
                } else {
                  cdst[p] = onIso ? 129 : Math.round(38 + t * 72);
                  cdst[p + 1] = onIso ? 140 : Math.round(166 - t * 62);
                  cdst[p + 2] = onIso ? 248 : Math.round(245);
                  cdst[p + 3] = onIso ? 102 : Math.round(e * 86);
                }
              }
            }
          }

          // Layer C: 2.5D Extruded Architectural Walls (offset base extrusion + luminous crisp cap)
          if (c === 2) {
            const topEdge = y > 0 && cls[idx - w] !== 2;
            if (isLight) {
              wdst[p] = topEdge ? 51 : 24;
              wdst[p + 1] = topEdge ? 65 : 35;
              wdst[p + 2] = topEdge ? 85 : 52;
              wdst[p + 3] = 252;
            } else {
              wdst[p] = topEdge ? 248 : 210;
              wdst[p + 1] = topEdge ? 250 : 228;
              wdst[p + 2] = topEdge ? 252 : 248;
              wdst[p + 3] = 252;
            }
          } else {
            // Subtle 2.5D drop-extrusion right below/right of wall pixels + crisp anti-aliased wall bevel
            const wallAbove = (y > 0 && cls[idx - w] === 2) || (y > 1 && x > 0 && cls[idx - w - 1] === 2);
            const wallAdj =
              (x > 0 && cls[idx - 1] === 2) ||
              (x + 1 < w && cls[idx + 1] === 2) ||
              (y > 0 && cls[idx - w] === 2) ||
              (y + 1 < h && cls[idx + w] === 2);
            if (wallAbove) {
              wdst[p] = isLight ? 15 : 2;
              wdst[p + 1] = isLight ? 23 : 6;
              wdst[p + 2] = isLight ? 42 : 16;
              wdst[p + 3] = isLight ? 92 : 175;
            } else if (wallAdj) {
              wdst[p] = isLight ? 51 : 56;
              wdst[p + 1] = isLight ? 65 : 189;
              wdst[p + 2] = isLight ? 85 : 248;
              wdst[p + 3] = isLight ? 78 : 96;
            }
          }
        }
      }

      fctx.putImageData(floorImg, 0, 0);
      cctx.putImageData(costImg, 0, 0);
      wctx.putImageData(wallImg, 0, 0);
      return { floor: floorCanvas, costmap: costCanvas, wall: wallCanvas };
    };

    styledMapCache = {
      key,
      dark: makeLayers(false),
      light: makeLayers(true),
      bounds,
      grid,
    };
    lidarCache = { key: null, rays: [] };
    lastRouteKey = null;
    updateResolutionBadge();
    return styledMapCache;
  }

  function updateResolutionBadge() {
    const resBadge = container.querySelector("#navResolutionBadge");
    if (!resBadge) return;
    const res = Number(mapYaml.resolution) || 0.05;
    const b = styledMapCache.bounds;
    const spanW = ((b ? b.width : (mapImage?.naturalWidth || mapYaml.width || 0)) * res).toFixed(1);
    const spanH = ((b ? b.height : (mapImage?.naturalHeight || mapYaml.height || 0)) * res).toFixed(1);
    resBadge.textContent = `${res}m/pix · ${spanW}m × ${spanH}m · 1:1 等比`;
  }

  function syncCanvasSize(forceRefit = false) {
    if (!stageEl || !canvas) return;
    const rect = stageEl.getBoundingClientRect();
    if (rect.width < 40 || rect.height < 40) return;
    const nextW = Math.round(rect.width);
    const nextH = Math.round(rect.height);
    const nextDpr = Math.min(window.devicePixelRatio || 1, 2);
    const sizeChanged = nextW !== viewW || nextH !== viewH || nextDpr !== canvasDpr;
    if (!sizeChanged && !forceRefit) return;

    // Preserve world-center anchor when resizing an already-fitted map
    const prevCenterPx = (viewW / 2 - offsetX) / Math.max(scale, 1e-6);
    const prevCenterPy = (viewH / 2 - offsetY) / Math.max(scale, 1e-6);

    viewW = nextW;
    viewH = nextH;
    canvasDpr = nextDpr;
    canvas.width = Math.round(viewW * canvasDpr);
    canvas.height = Math.round(viewH * canvasDpr);

    if (forceRefit || !hasFittedInitialView) {
      resetView();
    } else {
      offsetX = viewW / 2 - prevCenterPx * scale;
      offsetY = viewH / 2 - prevCenterPy * scale;
      drawMap();
    }
  }

  const mapResizeObserver = typeof ResizeObserver !== "undefined"
    ? new ResizeObserver(() => {
        if (mapView === "occupancy2d") syncCanvasSize(false);
        else cloudViewer.resize();
      })
    : null;
  if (mapResizeObserver && stageEl) mapResizeObserver.observe(stageEl);

  function load2DImage(url) {
    styledMapCache = { key: null, dark: null, light: null, bounds: null, grid: null };
    lidarCache = { key: null, rays: [] };
    previewRoute = null;
    lastRouteKey = null;
    hasFittedInitialView = false;
    updateRouteBanner();
    mapImage = new Image();
    mapImage.src = url;
    mapImage.onload = () => {
      buildStyledMap(mapImage);
      recomputePreviewRoute(true);
      syncCanvasSize(true);
      drawMap();
      if (mapView === "cloud3d") ensureActiveCloud(false);
    };
  }

  async function ensureActiveCloud(preserveCamera = false) {
    if (!activeMap) return;
    const pc = activeMap.pointcloud || {};
    const hudEl = container.querySelector("#navCloudHud");

    if (pc.source === "recorded_lidar_pcd" && pc.pcd_url) {
      let parsed = parsedPcdCache.get(pc.pcd_url);
      if (!parsed) {
        if (hudEl) hudEl.textContent = "正在加载真实 R1 Pro LiDAR SLAM 点云 (.pcd)...";
        try {
          const resp = await fetch(pc.pcd_url);
          if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
          const buf = await resp.arrayBuffer();
          parsed = parsePcdBuffer(buf);
          parsedPcdCache.set(pc.pcd_url, parsed);
        } catch (err) {
          if (hudEl) hudEl.textContent = `真实点云加载失败: ${err.message}`;
          return;
        }
      }
      currentParsedPcd = parsed;
      const colored = buildColoredPcdCloud(parsed, {
        colorMode: cloudColorMode,
        zMin: Math.min(-0.6, parsed.bounds.minZ),
        zMax: cloudZMax,
        label: pc.label || RECORDED_PCD_LABEL,
      });
      cloudViewer.setCloud(colored, { preserveCamera });
      cloudReadyForMapId = activeMap.id;
      return;
    }

    // Fallback for occupancy-only maps (e.g. mx_map)
    currentParsedPcd = null;
    if (!mapImage || !mapImage.complete || !mapImage.naturalWidth) return;
    if (cloudReadyForMapId === activeMap.id && preserveCamera) return;
    const probe = document.createElement("canvas");
    probe.width = mapImage.naturalWidth;
    probe.height = mapImage.naturalHeight;
    const probeCtx = probe.getContext("2d");
    probeCtx.drawImage(mapImage, 0, 0);
    const imageData = probeCtx.getImageData(0, 0, probe.width, probe.height);
    const cloud = occupancyToPointCloud({
      luma: lumaFromRgba(imageData.data),
      width: imageData.width,
      height: imageData.height,
      resolution: mapYaml.resolution,
      origin: mapYaml.origin,
    });
    cloudViewer.setCloud(cloud, { preserveCamera });
    cloudReadyForMapId = activeMap.id;
  }

  function applySelectedMap(mapEntry) {
    if (!mapEntry) return;
    activeMap = mapEntry;
    mapYaml = {
      resolution: Number(mapEntry.yaml?.resolution) || 0.15,
      origin: Array.isArray(mapEntry.yaml?.origin) ? mapEntry.yaml.origin : [-28.05, -21.75, 0],
      width: Number(mapEntry.yaml?.width) || 274,
      height: Number(mapEntry.yaml?.height) || 317,
    };
    cloudReadyForMapId = null;
    const resBadge = container.querySelector("#navResolutionBadge");
    updateResolutionBadge();
    const isRealPcd = activeMap.pointcloud?.source === "recorded_lidar_pcd";
    const controlsStrip = container.querySelector("#navCloudControls");
    if (controlsStrip) controlsStrip.hidden = !(mapView === "cloud3d" && isRealPcd);
    load2DImage(mapEntry.image_url);
    updateMapNotes();
    loadWaypoints();
    if (mapView === "cloud3d") ensureActiveCloud(false);
  }

  function updateMapNotes() {
    const threeD = mapView === "cloud3d";
    const isRealPcd = activeMap?.pointcloud?.source === "recorded_lidar_pcd";
    container.querySelector("#mapCloudNote").textContent = threeD
      ? (isRealPcd
          ? `${activeMap.pointcloud.label}。旋转/缩放仅观察相机，不改变目标；目标选点请在 2D 视图操作。`
          : MOCK_CLOUD_LABEL + " 旋转/缩放仅观察相机，不改变目标；目标选点仅在2D视图。")
      : `2D 战术导航图 (${activeMap?.id || "map"} · ${mapYaml.resolution}m/pix · 1:1 真实物理比例) · 左键拖拽设目标并实时预览 A* 避障路径，右键/Shift 平移，滚轮缩放。`;
    container.querySelector("#navMapLegend").innerHTML = threeD
      ? (isRealPcd
          ? "<div>色谱：真实 LiDAR 高程/强度/语义 · 支持顶棚 Z 切片</div><div>青色：当前位姿 · 橙色：已有目标（只读）</div>"
          : "<div>灰色：稀疏自由地面 · 绿色：占用格拉伸</div><div>青色：当前位姿 · 橙色：已有目标（只读）</div>")
      : "<div><span style=\"color:#2dd4bf;\">●</span> 位姿/激光雷达 · <span style=\"color:#f43f5e;\">■</span><span style=\"color:#818cf8;\">■</span> Nav2 膨胀代价层</div><div><span style=\"color:#38bdf8;\">➤</span> A* 避障路径 · <span style=\"color:#f59e0b;\">◎</span> 目标 · <span style=\"color:#c084fc;\">●</span> 航点</div>";
  }

  function setMapView(next) {
    mapInteractions.reset();
    mapView = next === "cloud3d" ? "cloud3d" : "occupancy2d";
    container.querySelectorAll("[data-map-view]").forEach(btn => {
      btn.setAttribute("aria-selected", btn.dataset.mapView === mapView ? "true" : "false");
    });
    const threeD = mapView === "cloud3d";
    const isRealPcd = activeMap?.pointcloud?.source === "recorded_lidar_pcd";
    canvas.hidden = threeD;
    cloudCanvas.hidden = !threeD;
    container.querySelector("#navCloudHud").hidden = !threeD;
    container.querySelector("#navCoordsOverlay").hidden = threeD;
    const tb2d = container.querySelector("#nav2dToolbar");
    if (tb2d) tb2d.hidden = threeD;
    const bannerEl = container.querySelector("#navRouteBanner");
    if (bannerEl) bannerEl.hidden = threeD || !goalPose || !previewRoute;
    container.querySelector("#navCloudControls").hidden = !(threeD && isRealPcd);
    if (stageEl) stageEl.classList.toggle("nav-map-light", !threeD && mapLightTheme);
    updateMapNotes();
    if (threeD) {
      stopAnimLoop();
      ensureActiveCloud(false);
      cloudViewer.resize();
      cloudViewer.setActive(true);
      const overlayPath = plannedPath.length > 1 ? plannedPath : (previewRoute?.ok ? previewRoute.pointsWorld : []);
      cloudViewer.setOverlays({ pose: latestRobotPose, goal: goalPose, path: overlayPath });
    } else {
      cloudViewer.setActive(false);
      syncCanvasSize(false);
      drawMap();
      syncAnimLoop();
    }
  }
  container.querySelectorAll("[data-map-view]").forEach(btn => btn.addEventListener("click", () => setMapView(btn.dataset.mapView)));

  // Map dataset switcher
  const mapSelect = container.querySelector("#navMapSelect");
  mapSelect.addEventListener("change", () => {
    const found = mapCatalog.find(m => m.id === mapSelect.value);
    if (found) applySelectedMap(found);
  });

  // Real 3D PCD controls (color mode, ceiling Z-slice, point size)
  container.querySelector("#cloudColorModeSelect").addEventListener("change", (e) => {
    cloudColorMode = e.target.value;
    if (currentParsedPcd) ensureActiveCloud(true);
  });
  container.querySelector("#cloudZMaxSlider").addEventListener("input", (e) => {
    cloudZMax = parseFloat(e.target.value) || 2.4;
    container.querySelector("#cloudZMaxVal").textContent = `${cloudZMax.toFixed(1)}m`;
    if (currentParsedPcd) ensureActiveCloud(true);
  });
  container.querySelector("#cloudPointSizeSlider").addEventListener("input", (e) => {
    cloudViewer.setPointSize(parseFloat(e.target.value) || 0.09);
  });

  function resetView() {
    if (!canvas || !mapImage) return;
    const iw = mapImage.naturalWidth || mapYaml.width || 274;
    const ih = mapImage.naturalHeight || mapYaml.height || 317;
    const b = styledMapCache.bounds || { minX: 0, minY: 0, width: iw, height: ih };
    const padX = Math.min(44, viewW * 0.08);
    const padY = Math.min(44, viewH * 0.09);
    const usableW = Math.max(80, viewW - padX * 2);
    const usableH = Math.max(80, viewH - padY * 2);
    // Strict 1:1 isotropic scale (pixels per map pixel identical in X and Y!)
    scale = Math.min(usableW / Math.max(1, b.width), usableH / Math.max(1, b.height));
    const cxImg = b.minX + b.width * 0.5;
    const cyImg = b.minY + b.height * 0.5;
    offsetX = viewW * 0.5 - cxImg * scale;
    offsetY = viewH * 0.5 - cyImg * scale;
    hasFittedInitialView = true;
    drawMap();
  }

  function zoomAtCenter(factor) {
    const cx = viewW * 0.5;
    const cy = viewH * 0.5;
    const newScale = Math.min(Math.max(scale * factor, 0.25), 14.0);
    offsetX = cx - (cx - offsetX) * (newScale / scale);
    offsetY = cy - (cy - offsetY) * (newScale / scale);
    scale = newScale;
    drawMap();
  }

  container.querySelector("#navResetViewBtn").addEventListener("click", () => {
    if (mapView === "cloud3d") cloudViewer.resetView();
    else resetView();
  });
  container.querySelector("#navZoomInBtn")?.addEventListener("click", () => zoomAtCenter(1.25));
  container.querySelector("#navZoomOutBtn")?.addEventListener("click", () => zoomAtCenter(0.8));
  container.querySelector("#navFitMapBtn")?.addEventListener("click", () => resetView());

  const costmapBtn = container.querySelector("#navCostmapBtn");
  costmapBtn?.addEventListener("click", () => {
    costmapEnabled = !costmapEnabled;
    costmapBtn.setAttribute("aria-pressed", costmapEnabled ? "true" : "false");
    costmapBtn.classList.toggle("active", costmapEnabled);
    drawMap();
  });

  const lidarBtn = container.querySelector("#navLidarBtn");
  lidarBtn?.addEventListener("click", () => {
    lidarEnabled = !lidarEnabled;
    lidarBtn.setAttribute("aria-pressed", lidarEnabled ? "true" : "false");
    lidarBtn.classList.toggle("active", lidarEnabled);
    drawMap();
    syncAnimLoop();
  });

  const measureBtn = container.querySelector("#navMeasureBtn");
  function setMeasureMode(next) {
    measureMode = !!next;
    if (!measureMode) {
      measurePoints = [];
      measureHover = null;
    }
    if (measureBtn) {
      measureBtn.setAttribute("aria-pressed", measureMode ? "true" : "false");
      measureBtn.classList.toggle("active-measure", measureMode);
    }
    if (canvas) canvas.style.cursor = measureMode ? "cell" : "crosshair";
    drawMap();
  }
  measureBtn?.addEventListener("click", () => setMeasureMode(!measureMode));

  const gridBtn = container.querySelector("#navMapGridBtn");
  gridBtn?.addEventListener("click", () => {
    mapGridEnabled = !mapGridEnabled;
    gridBtn.setAttribute("aria-pressed", mapGridEnabled ? "true" : "false");
    gridBtn.classList.toggle("active", mapGridEnabled);
    drawMap();
  });
  const themeBtn = container.querySelector("#navMapThemeBtn");
  themeBtn?.addEventListener("click", () => {
    mapLightTheme = !mapLightTheme;
    themeBtn.textContent = mapLightTheme ? "暗底" : "亮底";
    themeBtn.setAttribute("aria-pressed", mapLightTheme ? "true" : "false");
    themeBtn.classList.toggle("active", mapLightTheme);
    if (stageEl) stageEl.classList.toggle("nav-map-light", mapView === "occupancy2d" && mapLightTheme);
    drawMap();
  });

  // Coordinate transforms (all in logical 1:1 CSS pixel space)
  function worldToImage(wx, wy) {
    const ih = mapImage && mapImage.naturalHeight ? mapImage.naturalHeight : mapYaml.height;
    const px = (wx - mapYaml.origin[0]) / mapYaml.resolution;
    const py = ih - (wy - mapYaml.origin[1]) / mapYaml.resolution;
    return { px, py };
  }

  function imageToWorld(px, py) {
    const ih = mapImage && mapImage.naturalHeight ? mapImage.naturalHeight : mapYaml.height;
    const wx = mapYaml.origin[0] + px * mapYaml.resolution;
    const wy = mapYaml.origin[1] + (ih - py) * mapYaml.resolution;
    return { wx, wy };
  }

  function canvasToImage(cx, cy) {
    return { px: (cx - offsetX) / scale, py: (cy - offsetY) / scale };
  }

  function imageToCanvas(px, py) {
    return { cx: offsetX + px * scale, cy: offsetY + py * scale };
  }

  function worldToCanvas(wx, wy) {
    const { px, py } = worldToImage(wx, wy);
    return imageToCanvas(px, py);
  }

  function pickNiceMetricStep(pxPerMeter, minPx = 56) {
    const candidates = [0.25, 0.5, 1, 2, 5, 10, 20, 50];
    for (const c of candidates) {
      if (c * pxPerMeter >= minPx) return c;
    }
    return 50;
  }

  function drawRoundedRect(c, x, y, w, h, r) {
    c.beginPath();
    if (typeof c.roundRect === "function") {
      c.roundRect(x, y, w, h, r);
    } else {
      c.rect(x, y, w, h);
    }
  }

  // Drawing & Real-Time Tactical Overlays
  let latestRobotPose = { x: 0, y: 0, yaw: 0 };
  let plannedPath = [];

  function updateRouteBanner() {
    const banner = container.querySelector("#navRouteBanner");
    if (!banner) return;
    if (mapView !== "occupancy2d" || !goalPose || !previewRoute) {
      banner.hidden = true;
      banner.classList.remove("is-blocked");
      return;
    }
    banner.hidden = false;
    if (previewRoute.ok) {
      banner.classList.remove("is-blocked");
      const modeLabel = plannedPath && plannedPath.length > 1 ? "导航执行路径" : "A* 避障路径预览";
      const clrVal = Number(previewRoute.minClearanceM || 0);
      const clrCls = clrVal < 0.34 ? "nrp-clr-tight" : clrVal >= 0.5 ? "nrp-clr-safe" : "";
      banner.innerHTML = `<span class="nrp-dot"></span><span>${modeLabel} </span><strong>${previewRoute.lengthM.toFixed(1)} m</strong><span class="nrp-sep"> · </span><span>预计 ${Math.max(1, Math.round(previewRoute.etaS))} s</span><span class="nrp-sep"> · </span><span>最小壁距 <strong class="${clrCls}">${clrVal.toFixed(2)} m</strong></span>`;
    } else {
      banner.classList.add("is-blocked");
      const msg = previewRoute.reason === "goal_in_obstacle"
        ? "目标位于墙体障碍物内 · 请拖拽至蓝色安全通行区"
        : "目标暂不可达（通道被阻断或超出已建图区域）";
      banner.innerHTML = `<span class="nrp-dot"></span><span>${msg}</span>`;
    }
  }

  function recomputePreviewRoute(force = false) {
    if (!goalPose || !styledMapCache.grid) {
      previewRoute = null;
      lastRouteKey = null;
      updateRouteBanner();
      return;
    }
    const rImg = worldToImage(latestRobotPose.x, latestRobotPose.y);
    const gImg = worldToImage(goalPose.x, goalPose.y);
    const key = `${styledMapCache.key}|${Math.round(rImg.px / 3)},${Math.round(rImg.py / 3)}->${Math.round(gImg.px)},${Math.round(gImg.py)}`;
    if (!force && key === lastRouteKey && previewRoute) return;
    lastRouteKey = key;

    const res = planGridPathAStar(styledMapCache.grid, rImg.px, rImg.py, gImg.px, gImg.py, {
      robotRadiusM: 0.24,
      safeClearM: 0.72,
      maxExpanded: 70000,
    });
    if (res.ok) {
      const pointsWorld = res.pointsPx.map(([px, py]) => {
        const w = imageToWorld(px, py);
        return [w.wx, w.wy];
      });
      previewRoute = {
        ok: true,
        pointsWorld,
        lengthM: res.lengthM,
        minClearanceM: res.minClearanceM,
        etaS: res.lengthM / 0.55,
      };
    } else {
      previewRoute = {
        ok: false,
        reason: res.reason,
        pointsWorld: [],
        lengthM: 0,
        minClearanceM: 0,
        etaS: 0,
      };
    }
    updateRouteBanner();
    syncAnimLoop();
  }

  function getCachedLidarRays(grid) {
    if (!grid) return [];
    const rImg = worldToImage(latestRobotPose.x, latestRobotPose.y);
    const key = `${styledMapCache.key}|${Math.round(rImg.px * 2)},${Math.round(rImg.py * 2)},${Math.round(latestRobotPose.yaw * 40)}`;
    if (lidarCache.key === key) return lidarCache.rays;
    const rays = castLidarScan(grid, rImg.px, rImg.py, latestRobotPose.yaw, {
      rays: 120,
      maxRangeM: 12.0,
    });
    lidarCache = { key, rays };
    return rays;
  }

  function stopAnimLoop() {
    if (animRafId !== null) {
      cancelAnimationFrame(animRafId);
      animRafId = null;
    }
  }

  function shouldAnimate2d() {
    if (!mapSupported || mapView !== "occupancy2d" || document.hidden) return false;
    if (!container.classList.contains("active")) return false;
    return lidarEnabled || (plannedPath && plannedPath.length > 1) || !!(previewRoute && previewRoute.ok) || !!goalPose;
  }

  function animTick(ts) {
    if (!shouldAnimate2d()) {
      animRafId = null;
      return;
    }
    if (ts - lastAnimTs >= 42) {
      const dt = lastAnimTs ? Math.min(0.12, (ts - lastAnimTs) / 1000) : 0.042;
      lastAnimTs = ts;
      animPhase = (animPhase + dt) % 1000;
      drawMap();
    }
    animRafId = requestAnimationFrame(animTick);
  }

  function syncAnimLoop() {
    if (shouldAnimate2d()) {
      if (animRafId === null) {
        lastAnimTs = 0;
        animRafId = requestAnimationFrame(animTick);
      }
    } else {
      stopAnimLoop();
    }
  }

  function drawAnimatedRoute(points, { isActiveNav, isLight }) {
    if (!points || points.length < 2) return;
    const pts = points.map(([wx, wy]) => worldToCanvas(wx, wy));

    ctx.save();
    ctx.lineCap = "round";
    ctx.lineJoin = "round";

    // Soft outer halo glow
    ctx.strokeStyle = isActiveNav
      ? (isLight ? "rgba(13, 148, 136, 0.22)" : "rgba(45, 212, 191, 0.24)")
      : (isLight ? "rgba(2, 132, 199, 0.20)" : "rgba(56, 189, 248, 0.24)");
    ctx.lineWidth = 9;
    ctx.beginPath();
    ctx.moveTo(pts[0].cx, pts[0].cy);
    for (let i = 1; i < pts.length; i++) ctx.lineTo(pts[i].cx, pts[i].cy);
    ctx.stroke();

    // Base continuous ribbon
    ctx.strokeStyle = isActiveNav
      ? (isLight ? "rgba(13, 148, 136, 0.45)" : "rgba(45, 212, 191, 0.45)")
      : (isLight ? "rgba(2, 132, 199, 0.36)" : "rgba(56, 189, 248, 0.40)");
    ctx.lineWidth = 3.2;
    ctx.stroke();

    // Marching luminous flow dashes
    ctx.strokeStyle = isActiveNav
      ? (isLight ? "#0d9488" : "#5eead4")
      : (isLight ? "#0284c7" : "#7dd3fc");
    ctx.lineWidth = 2.4;
    ctx.setLineDash([8, 8]);
    ctx.lineDashOffset = -(animPhase * 28);
    ctx.stroke();
    ctx.setLineDash([]);

    // Directional flow chevrons every ~46px along path
    let acc = (animPhase * 28) % 46;
    ctx.strokeStyle = isLight ? "#ffffff" : "#e0f2fe";
    ctx.lineWidth = 1.7;
    for (let i = 1; i < pts.length; i++) {
      const dx = pts[i].cx - pts[i - 1].cx;
      const dy = pts[i].cy - pts[i - 1].cy;
      const seg = Math.hypot(dx, dy);
      if (seg < 1e-3) continue;
      let d = 46 - acc;
      while (d <= seg) {
        const t = d / seg;
        const x = pts[i - 1].cx + dx * t;
        const y = pts[i - 1].cy + dy * t;
        const ang = Math.atan2(dy, dx);
        ctx.save();
        ctx.translate(x, y);
        ctx.rotate(ang);
        ctx.beginPath();
        ctx.moveTo(-4, -3.6);
        ctx.lineTo(2.4, 0);
        ctx.lineTo(-4, 3.6);
        ctx.stroke();
        ctx.restore();
        d += 46;
      }
      acc = (acc + seg) % 46;
    }
    ctx.restore();
  }

  function drawMap() {
    if (!ctx) return;
    ctx.save();
    ctx.setTransform(canvasDpr, 0, 0, canvasDpr, 0, 0);
    ctx.clearRect(0, 0, viewW, viewH);

    const isLight = mapLightTheme;
    const res = Number(mapYaml.resolution) || 0.05;
    const pxPerMeter = scale / res;

    // 1. Sleek Tactical Matte Backdrop (Deep Obsidian Slate default / Cool Matte Light)
    const bgGrad = ctx.createRadialGradient(
      viewW * 0.5, viewH * 0.46, Math.min(viewW, viewH) * 0.08,
      viewW * 0.5, viewH * 0.5, Math.max(viewW, viewH) * 0.78
    );
    if (isLight) {
      bgGrad.addColorStop(0, "#eff3f8");
      bgGrad.addColorStop(1, "#dfe6ef");
    } else {
      bgGrad.addColorStop(0, "#0f1a2d");
      bgGrad.addColorStop(0.65, "#09111f");
      bgGrad.addColorStop(1, "#050912");
    }
    ctx.fillStyle = bgGrad;
    ctx.fillRect(0, 0, viewW, viewH);

    // 2. Optional True-World Metric Grid (toggleable via '网格')
    const majorStepM = pickNiceMetricStep(pxPerMeter, 72);
    const minorStepM = majorStepM / 2;
    const topLeftW = imageToWorld((0 - offsetX) / scale, (0 - offsetY) / scale);
    const botRightW = imageToWorld((viewW - offsetX) / scale, (viewH - offsetY) / scale);
    const minWx = Math.min(topLeftW.wx, botRightW.wx);
    const maxWx = Math.max(topLeftW.wx, botRightW.wx);
    const minWy = Math.min(topLeftW.wy, botRightW.wy);
    const maxWy = Math.max(topLeftW.wy, botRightW.wy);

    if (mapGridEnabled) {
      if (minorStepM * pxPerMeter >= 24) {
        ctx.strokeStyle = isLight ? "rgba(100, 116, 139, 0.10)" : "rgba(148, 163, 184, 0.055)";
        ctx.lineWidth = 1;
        ctx.beginPath();
        for (let wx = Math.floor(minWx / minorStepM) * minorStepM; wx <= maxWx; wx += minorStepM) {
          const cx = Math.round(worldToCanvas(wx, 0).cx) + 0.5;
          ctx.moveTo(cx, 0);
          ctx.lineTo(cx, viewH);
        }
        for (let wy = Math.floor(minWy / minorStepM) * minorStepM; wy <= maxWy; wy += minorStepM) {
          const cy = Math.round(worldToCanvas(0, wy).cy) + 0.5;
          ctx.moveTo(0, cy);
          ctx.lineTo(viewW, cy);
        }
        ctx.stroke();
      }
      ctx.strokeStyle = isLight ? "rgba(100, 116, 139, 0.20)" : "rgba(148, 163, 184, 0.13)";
      ctx.lineWidth = 1;
      ctx.beginPath();
      for (let wx = Math.floor(minWx / majorStepM) * majorStepM; wx <= maxWx; wx += majorStepM) {
        const cx = Math.round(worldToCanvas(wx, 0).cx) + 0.5;
        ctx.moveTo(cx, 0);
        ctx.lineTo(cx, viewH);
      }
      for (let wy = Math.floor(minWy / majorStepM) * majorStepM; wy <= maxWy; wy += majorStepM) {
        const cy = Math.round(worldToCanvas(0, wy).cy) + 0.5;
        ctx.moveTo(0, cy);
        ctx.lineTo(viewW, cy);
      }
      ctx.stroke();
    }

    // 3. Three-Layer Architectural Floor + Real Nav2 EDT Costmap + 2.5D Extruded Walls
    let activeGrid = null;
    if (mapImage && mapImage.complete && mapImage.naturalWidth !== 0) {
      const styled = buildStyledMap(mapImage);
      activeGrid = styled?.grid || null;
      const layers = styled ? (isLight ? styled.light : styled.dark) : null;
      const drawW = mapImage.naturalWidth * scale;
      const drawH = mapImage.naturalHeight * scale;
      ctx.imageSmoothingEnabled = scale < 1.85;
      if (ctx.imageSmoothingEnabled) ctx.imageSmoothingQuality = "high";

      if (layers && layers.floor && layers.wall) {
        // Layer A: Elevated ambient-occlusion floor slab with soft shadow
        ctx.save();
        ctx.shadowColor = isLight ? "rgba(15, 23, 42, 0.12)" : "rgba(0, 0, 0, 0.62)";
        ctx.shadowBlur = isLight ? 16 : 24;
        ctx.shadowOffsetX = 0;
        ctx.shadowOffsetY = isLight ? 4 : 7;
        ctx.drawImage(layers.floor, offsetX, offsetY, drawW, drawH);
        ctx.restore();

        // Layer B: Real Nav2 Obstacle Inflation & Clearance Costmap (toggleable)
        if (costmapEnabled && layers.costmap) {
          ctx.drawImage(layers.costmap, offsetX, offsetY, drawW, drawH);
        }

        // Layer C: 2.5D Extruded Architectural Walls
        ctx.drawImage(layers.wall, offsetX, offsetY, drawW, drawH);
      } else {
        ctx.drawImage(mapImage, offsetX, offsetY, drawW, drawH);
      }
    }

    // 4. Optional World-Meter Axis Ruler Ticks (only when metric grid is enabled)
    if (mapGridEnabled) {
      ctx.font = "500 10px ui-monospace, SFMono-Regular, Menlo, monospace";
      ctx.fillStyle = isLight ? "rgba(71, 85, 105, 0.72)" : "rgba(148, 163, 184, 0.58)";
      ctx.textAlign = "center";
      for (let wx = Math.floor(minWx / majorStepM) * majorStepM; wx <= maxWx; wx += majorStepM) {
        const cx = worldToCanvas(wx, 0).cx;
        if (cx > 260 && cx < viewW - 155) {
          const label = Math.abs(wx) < 1e-6 ? "0m" : `${Number(wx.toFixed(1))}m`;
          ctx.fillText(label, cx, viewH - 6);
        }
      }
      ctx.textAlign = "left";
      for (let wy = Math.floor(minWy / majorStepM) * majorStepM; wy <= maxWy; wy += majorStepM) {
        const cy = worldToCanvas(0, wy).cy;
        if (cy > 88 && cy < viewH - 44) {
          const label = Math.abs(wy) < 1e-6 ? "0m" : `${Number(wy.toFixed(1))}m`;
          ctx.fillText(label, 8, cy - 3);
        }
      }
    }

    // 5. Real-Time 360° Raycasted 2D LiDAR Scan + Forward Head Stereo Camera FOV Frustum
    const rC = worldToCanvas(latestRobotPose.x, latestRobotPose.y);
    const rAngle = -latestRobotPose.yaw;
    if (lidarEnabled && activeGrid) {
      const rays = getCachedLidarRays(activeGrid);
      if (rays.length > 4) {
        ctx.save();
        // 5a. Translucent 360° line-of-sight visibility polygon
        const maxVisR = Math.max(40, 12.0 * pxPerMeter);
        const visGrad = ctx.createRadialGradient(rC.cx, rC.cy, 4, rC.cx, rC.cy, maxVisR);
        if (isLight) {
          visGrad.addColorStop(0, "rgba(13, 148, 136, 0.14)");
          visGrad.addColorStop(0.55, "rgba(14, 165, 233, 0.06)");
          visGrad.addColorStop(1, "rgba(14, 165, 233, 0.0)");
        } else {
          visGrad.addColorStop(0, "rgba(45, 212, 191, 0.16)");
          visGrad.addColorStop(0.55, "rgba(56, 189, 248, 0.06)");
          visGrad.addColorStop(1, "rgba(56, 189, 248, 0.0)");
        }
        ctx.fillStyle = visGrad;
        ctx.beginPath();
        for (let i = 0; i < rays.length; i++) {
          const pt = imageToCanvas(rays[i].px, rays[i].py);
          if (i === 0) ctx.moveTo(pt.cx, pt.cy);
          else ctx.lineTo(pt.cx, pt.cy);
        }
        ctx.closePath();
        ctx.fill();

        // 5b. Forward Head Stereo Camera FOV Frustum (±42°, 4.4m range)
        const fovHalf = (42 * Math.PI) / 180;
        const fovMaxM = 4.4;
        const fovR = fovMaxM * pxPerMeter;
        const fovGrad = ctx.createRadialGradient(rC.cx, rC.cy, 6, rC.cx, rC.cy, Math.max(12, fovR));
        if (isLight) {
          fovGrad.addColorStop(0, "rgba(2, 132, 199, 0.22)");
          fovGrad.addColorStop(1, "rgba(2, 132, 199, 0.02)");
        } else {
          fovGrad.addColorStop(0, "rgba(56, 189, 248, 0.26)");
          fovGrad.addColorStop(1, "rgba(56, 189, 248, 0.02)");
        }
        ctx.fillStyle = fovGrad;
        ctx.beginPath();
        ctx.moveTo(rC.cx, rC.cy);
        const steps = 18;
        for (let s = 0; s <= steps; s++) {
          const rel = -fovHalf + (s / steps) * (fovHalf * 2);
          const idxRay = Math.round((((rel % (Math.PI * 2)) + Math.PI * 2) % (Math.PI * 2)) / (Math.PI * 2) * rays.length) % rays.length;
          const distM = Math.min(fovMaxM, rays[idxRay]?.distM ?? fovMaxM);
          const angCanvas = rAngle - rel;
          ctx.lineTo(rC.cx + Math.cos(angCanvas) * distM * pxPerMeter, rC.cy + Math.sin(angCanvas) * distM * pxPerMeter);
        }
        ctx.closePath();
        ctx.fill();

        // Crisp FOV boundary rays
        ctx.strokeStyle = isLight ? "rgba(2, 132, 199, 0.36)" : "rgba(125, 211, 252, 0.36)";
        ctx.lineWidth = 1;
        ctx.setLineDash([4, 4]);
        for (const side of [-fovHalf, fovHalf]) {
          const angCanvas = rAngle - side;
          ctx.beginPath();
          ctx.moveTo(rC.cx, rC.cy);
          ctx.lineTo(rC.cx + Math.cos(angCanvas) * fovR, rC.cy + Math.sin(angCanvas) * fovR);
          ctx.stroke();
        }
        ctx.setLineDash([]);

        // 5c. Subtle rotating LiDAR sweep highlight
        const sweepAngle = (animPhase * 1.45) % (Math.PI * 2);

        // 5d. Live Laser Hit Points on Real Walls
        for (let i = 0; i < rays.length; i++) {
          const ray = rays[i];
          if (!ray.hit) continue;
          const pt = imageToCanvas(ray.px, ray.py);
          let delta = Math.abs(((ray.relAngle - sweepAngle + Math.PI * 3) % (Math.PI * 2)) - Math.PI);
          const boost = Math.max(0, 1 - delta / 0.65);
          ctx.fillStyle = isLight
            ? `rgba(13, 148, 136, ${(0.55 + boost * 0.42).toFixed(2)})`
            : `rgba(45, 212, 191, ${(0.52 + boost * 0.46).toFixed(2)})`;
          ctx.beginPath();
          ctx.arc(pt.cx, pt.cy, 1.85 + boost * 1.1, 0, Math.PI * 2);
          ctx.fill();
        }
        ctx.restore();
      }
    }

    // 6. World Origin Triad (0, 0) — Subtle Red +X / Green +Y
    const origC = worldToCanvas(0, 0);
    const axisLen = 18;
    ctx.lineWidth = 1.6;
    ctx.strokeStyle = "rgba(239, 68, 68, 0.85)";
    ctx.beginPath();
    ctx.moveTo(origC.cx, origC.cy);
    ctx.lineTo(origC.cx + axisLen, origC.cy);
    ctx.stroke();
    ctx.strokeStyle = "rgba(34, 197, 94, 0.85)";
    ctx.beginPath();
    ctx.moveTo(origC.cx, origC.cy);
    ctx.lineTo(origC.cx, origC.cy - axisLen);
    ctx.stroke();
    ctx.fillStyle = isLight ? "#0f172a" : "#f8fafc";
    ctx.beginPath();
    ctx.arc(origC.cx, origC.cy, 2.2, 0, Math.PI * 2);
    ctx.fill();

    // 7. Real Cost-Aware A* Preview Route (or Active Backend Planned Path)
    const hasBackendPath = plannedPath && plannedPath.length > 1;
    if (hasBackendPath) {
      drawAnimatedRoute(plannedPath, { isActiveNav: true, isLight });
    } else if (previewRoute && previewRoute.ok && previewRoute.pointsWorld.length > 1) {
      drawAnimatedRoute(previewRoute.pointsWorld, { isActiveNav: false, isLight });
    } else if (goalPose && previewRoute && !previewRoute.ok) {
      // Blocked / inside-obstacle warning chord
      const gC = worldToCanvas(goalPose.x, goalPose.y);
      ctx.save();
      ctx.strokeStyle = "rgba(244, 63, 94, 0.72)";
      ctx.lineWidth = 2;
      ctx.setLineDash([5, 5]);
      ctx.beginPath();
      ctx.moveTo(rC.cx, rC.cy);
      ctx.lineTo(gC.cx, gC.cy);
      ctx.stroke();
      ctx.restore();
    }

    // 8. Saved Waypoints (sleek glowing pins + glass pills)
    ctx.font = "600 11px ui-sans-serif, system-ui, -apple-system, sans-serif";
    ctx.textAlign = "left";
    for (const wp of waypoints) {
      const wC = worldToCanvas(wp.x, wp.y);
      ctx.fillStyle = "rgba(192, 132, 252, 0.22)";
      ctx.beginPath();
      ctx.arc(wC.cx, wC.cy, 9.5, 0, Math.PI * 2);
      ctx.fill();

      ctx.fillStyle = "#a855f7";
      ctx.beginPath();
      ctx.arc(wC.cx, wC.cy, 4.6, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 1.5;
      ctx.stroke();

      const tw = ctx.measureText(wp.name).width;
      const pillX = wC.cx + 13;
      const pillY = wC.cy - 10;
      const pillW = tw + 12;
      const pillH = 20;
      drawRoundedRect(ctx, pillX, pillY, pillW, pillH, 6);
      ctx.fillStyle = isLight ? "rgba(255, 255, 255, 0.92)" : "rgba(10, 17, 32, 0.86)";
      ctx.fill();
      ctx.strokeStyle = isLight ? "rgba(168, 85, 247, 0.45)" : "rgba(192, 132, 252, 0.50)";
      ctx.lineWidth = 1;
      ctx.stroke();
      ctx.fillStyle = isLight ? "#581c87" : "#f3e8ff";
      ctx.fillText(wp.name, pillX + 6, pillY + 13.5);
    }

    // 9. Goal Pose Target Beacon + Interactive Compass Heading Dial
    if (goalPose) {
      const gC = worldToCanvas(goalPose.x, goalPose.y);
      const angle = -goalPose.yaw;
      const blocked = previewRoute && !previewRoute.ok;
      const accent = blocked ? "#f43f5e" : "#f59e0b";
      const pulse = (Math.sin(animPhase * 4.2) + 1) * 0.5;

      // Pulsing acquisition halo
      ctx.fillStyle = blocked
        ? `rgba(244, 63, 94, ${(0.14 + pulse * 0.12).toFixed(2)})`
        : `rgba(245, 158, 11, ${(0.14 + pulse * 0.12).toFixed(2)})`;
      ctx.beginPath();
      ctx.arc(gC.cx, gC.cy, 15 + pulse * 4, 0, Math.PI * 2);
      ctx.fill();

      // Compass heading dial ring (expanded with degree ticks while dragging)
      const dialR = isDraggingGoal ? 26 : 13;
      ctx.save();
      ctx.strokeStyle = blocked ? "rgba(244, 63, 94, 0.78)" : "rgba(251, 191, 36, 0.78)";
      ctx.lineWidth = 1.4;
      if (isDraggingGoal) ctx.setLineDash([3, 3]);
      ctx.beginPath();
      ctx.arc(gC.cx, gC.cy, dialR, 0, Math.PI * 2);
      ctx.stroke();
      ctx.setLineDash([]);
      if (isDraggingGoal) {
        for (let k = 0; k < 8; k++) {
          const a = (k * Math.PI) / 4;
          ctx.beginPath();
          ctx.moveTo(gC.cx + Math.cos(a) * (dialR - 3), gC.cy + Math.sin(a) * (dialR - 3));
          ctx.lineTo(gC.cx + Math.cos(a) * (dialR + 3), gC.cy + Math.sin(a) * (dialR + 3));
          ctx.stroke();
        }
      }
      ctx.restore();

      const vecLen = isDraggingGoal ? 34 : 26;
      const tipX = gC.cx + Math.cos(angle) * vecLen;
      const tipY = gC.cy + Math.sin(angle) * vecLen;
      ctx.strokeStyle = accent;
      ctx.lineWidth = 2.6;
      ctx.lineCap = "round";
      ctx.beginPath();
      ctx.moveTo(gC.cx, gC.cy);
      ctx.lineTo(tipX, tipY);
      ctx.stroke();

      const headLen = 7.5;
      ctx.fillStyle = accent;
      ctx.beginPath();
      ctx.moveTo(tipX, tipY);
      ctx.lineTo(tipX - headLen * Math.cos(angle - 0.5), tipY - headLen * Math.sin(angle - 0.5));
      ctx.lineTo(tipX - headLen * Math.cos(angle + 0.5), tipY - headLen * Math.sin(angle + 0.5));
      ctx.closePath();
      ctx.fill();

      ctx.fillStyle = accent;
      ctx.beginPath();
      ctx.arc(gC.cx, gC.cy, 5.8, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 2;
      ctx.stroke();

      if (isDraggingGoal) {
        const deg = Math.round((goalPose.yaw * 180) / Math.PI);
        const label = `Yaw ${deg > 0 ? "+" : ""}${deg}°`;
        ctx.font = "600 10px ui-monospace, SFMono-Regular, Menlo, monospace";
        const lw = ctx.measureText(label).width + 12;
        drawRoundedRect(ctx, gC.cx - lw / 2, gC.cy - dialR - 24, lw, 18, 5);
        ctx.fillStyle = "rgba(9, 15, 28, 0.90)";
        ctx.fill();
        ctx.strokeStyle = "rgba(251, 191, 36, 0.65)";
        ctx.lineWidth = 1;
        ctx.stroke();
        ctx.fillStyle = "#fde68a";
        ctx.textAlign = "center";
        ctx.fillText(label, gC.cx, gC.cy - dialR - 11.5);
      }
    }

    // 10. True-Scale Robot Chassis Footprint (~0.68m diameter) + Directional Chevron
    const chassisR = Math.max(9.5, Math.min(34, 0.34 * pxPerMeter));
    const rPulse = (Math.sin(animPhase * 3.0) + 1) * 0.5;

    ctx.fillStyle = `rgba(45, 212, 191, ${(0.12 + rPulse * 0.10).toFixed(2)})`;
    ctx.beginPath();
    ctx.arc(rC.cx, rC.cy, chassisR + 4 + rPulse * 3, 0, Math.PI * 2);
    ctx.fill();

    ctx.fillStyle = isLight ? "rgba(13, 148, 136, 0.24)" : "rgba(20, 184, 166, 0.30)";
    ctx.beginPath();
    ctx.arc(rC.cx, rC.cy, chassisR, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = "#2dd4bf";
    ctx.lineWidth = 2;
    ctx.stroke();

    ctx.save();
    ctx.translate(rC.cx, rC.cy);
    ctx.rotate(rAngle);
    const arrowR = Math.max(8, chassisR * 0.86);
    ctx.fillStyle = "#14b8a6";
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 1.8;
    ctx.beginPath();
    ctx.moveTo(arrowR + 3, 0);
    ctx.lineTo(-arrowR * 0.65, -arrowR * 0.58);
    ctx.lineTo(-arrowR * 0.28, 0);
    ctx.lineTo(-arrowR * 0.65, arrowR * 0.58);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
    ctx.restore();

    // 11. Interactive 2-Point Metric Tape Measure Overlay
    const mA = measurePoints[0];
    const mB = measurePoints[1] || (measureMode && measurePoints.length === 1 ? measureHover : null);
    if (mA) {
      const pA = worldToCanvas(mA.wx, mA.wy);
      ctx.save();
      if (mB) {
        const pB = worldToCanvas(mB.wx, mB.wy);
        const dWx = mB.wx - mA.wx;
        const dWy = mB.wy - mA.wy;
        const distM = Math.hypot(dWx, dWy);
        const deg = Math.round((Math.atan2(dWy, dWx) * 180) / Math.PI);

        ctx.strokeStyle = isLight ? "rgba(217, 119, 6, 0.35)" : "rgba(251, 191, 36, 0.28)";
        ctx.lineWidth = 6;
        ctx.beginPath();
        ctx.moveTo(pA.cx, pA.cy);
        ctx.lineTo(pB.cx, pB.cy);
        ctx.stroke();

        ctx.strokeStyle = isLight ? "#d97706" : "#fbbf24";
        ctx.lineWidth = 2;
        ctx.setLineDash([6, 4]);
        ctx.beginPath();
        ctx.moveTo(pA.cx, pA.cy);
        ctx.lineTo(pB.cx, pB.cy);
        ctx.stroke();
        ctx.setLineDash([]);

        // Midpoint metric dimension pill
        const midX = (pA.cx + pB.cx) * 0.5;
        const midY = (pA.cy + pB.cy) * 0.5 - 14;
        const text = `${distM.toFixed(2)} m  (ΔX ${dWx >= 0 ? "+" : ""}${dWx.toFixed(1)} · ΔY ${dWy >= 0 ? "+" : ""}${dWy.toFixed(1)} · ${deg}°)`;
        ctx.font = "600 11px ui-monospace, SFMono-Regular, Menlo, monospace";
        const tw = ctx.measureText(text).width + 16;
        drawRoundedRect(ctx, midX - tw / 2, midY - 12, tw, 22, 6);
        ctx.fillStyle = isLight ? "rgba(255, 251, 235, 0.96)" : "rgba(15, 23, 42, 0.92)";
        ctx.fill();
        ctx.strokeStyle = isLight ? "rgba(217, 119, 6, 0.65)" : "rgba(251, 191, 36, 0.70)";
        ctx.lineWidth = 1.2;
        ctx.stroke();
        ctx.fillStyle = isLight ? "#92400e" : "#fde68a";
        ctx.textAlign = "center";
        ctx.fillText(text, midX, midY + 3);

        ctx.fillStyle = "#f59e0b";
        ctx.beginPath();
        ctx.arc(pB.cx, pB.cy, 4.8, 0, Math.PI * 2);
        ctx.fill();
        ctx.strokeStyle = "#ffffff";
        ctx.lineWidth = 1.6;
        ctx.stroke();
      }
      ctx.fillStyle = "#f59e0b";
      ctx.beginPath();
      ctx.arc(pA.cx, pA.cy, 4.8, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 1.6;
      ctx.stroke();
      ctx.restore();
    }

    // 12. Mini Compass Rose (N / +Y) + Dynamic CAD Metric Scale Bar (bottom-right)
    const scaleMeters = pickNiceMetricStep(pxPerMeter, 72);
    const barPx = Math.round(scaleMeters * pxPerMeter);
    const scaleBoxH = 26;
    if (barPx >= 36 && barPx <= viewW * 0.36) {
      const boxW = barPx + 22;
      const bx = viewW - boxW - 12;
      const by = viewH - scaleBoxH - 12;
      drawRoundedRect(ctx, bx, by, boxW, scaleBoxH, 7);
      ctx.fillStyle = isLight ? "rgba(255, 255, 255, 0.92)" : "rgba(9, 15, 28, 0.86)";
      ctx.fill();
      ctx.strokeStyle = isLight ? "rgba(148, 163, 184, 0.45)" : "rgba(148, 163, 184, 0.24)";
      ctx.lineWidth = 1;
      ctx.stroke();

      const x0 = bx + 11;
      const x1 = x0 + barPx;
      const yBar = by + 18;
      ctx.strokeStyle = isLight ? "#0f172a" : "#e2e8f0";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(x0, yBar - 4);
      ctx.lineTo(x0, yBar);
      ctx.lineTo(x1, yBar);
      ctx.lineTo(x1, yBar - 4);
      ctx.stroke();

      ctx.font = "600 10px ui-monospace, SFMono-Regular, Menlo, monospace";
      ctx.fillStyle = isLight ? "#0f172a" : "#e2e8f0";
      ctx.textAlign = "center";
      ctx.fillText(`${scaleMeters} m`, (x0 + x1) * 0.5, by + 12);
    }

    // Mini Compass Badge right above scale bar
    const cmpX = viewW - 28;
    const cmpY = viewH - scaleBoxH - 36;
    ctx.save();
    ctx.beginPath();
    ctx.arc(cmpX, cmpY, 15, 0, Math.PI * 2);
    ctx.fillStyle = isLight ? "rgba(255, 255, 255, 0.90)" : "rgba(9, 15, 28, 0.84)";
    ctx.fill();
    ctx.strokeStyle = isLight ? "rgba(148, 163, 184, 0.45)" : "rgba(148, 163, 184, 0.24)";
    ctx.lineWidth = 1;
    ctx.stroke();
    // North (+Y) pointer
    ctx.fillStyle = "#ef4444";
    ctx.beginPath();
    ctx.moveTo(cmpX, cmpY - 10);
    ctx.lineTo(cmpX - 3.5, cmpY + 1);
    ctx.lineTo(cmpX + 3.5, cmpY + 1);
    ctx.closePath();
    ctx.fill();
    ctx.fillStyle = isLight ? "#94a3b8" : "#475569";
    ctx.beginPath();
    ctx.moveTo(cmpX, cmpY + 10);
    ctx.lineTo(cmpX - 3.5, cmpY + 1);
    ctx.lineTo(cmpX + 3.5, cmpY + 1);
    ctx.closePath();
    ctx.fill();
    ctx.font = "700 8px ui-monospace, SFMono-Regular, Menlo, monospace";
    ctx.fillStyle = isLight ? "#334155" : "#cbd5e1";
    ctx.textAlign = "center";
    ctx.fillText("+Y", cmpX, cmpY - 18);
    ctx.restore();

    ctx.restore();
  }

  function clientToCanvas(e) {
    const rect = canvas.getBoundingClientRect();
    const scaleX = rect.width > 0 ? viewW / rect.width : 1;
    const scaleY = rect.height > 0 ? viewH / rect.height : 1;
    return {
      cx: (e.clientX - rect.left) * scaleX,
      cy: (e.clientY - rect.top) * scaleY,
    };
  }

  function onMapMouseDown(e) {
    const { cx, cy } = clientToCanvas(e);
    if (e.button === 2 || e.shiftKey || e.altKey) {
      isPanning = true;
      panStartX = cx - offsetX;
      panStartY = cy - offsetY;
      return;
    }
    if (e.button === 0) {
      const { px, py } = canvasToImage(cx, cy);
      const { wx, wy } = imageToWorld(px, py);
      if (measureMode) {
        if (measurePoints.length === 0 || measurePoints.length >= 2) {
          measurePoints = [{ wx, wy }];
          measureHover = { wx, wy };
        } else {
          measurePoints = [measurePoints[0], { wx, wy }];
          measureHover = null;
        }
        drawMap();
        return;
      }
      isDraggingGoal = true;
      dragStartPos = { wx, wy };
      goalPose = { x: Math.round(wx * 100) / 100, y: Math.round(wy * 100) / 100, yaw: 0 };
      container.querySelector("#navGoalInputX").value = goalPose.x;
      container.querySelector("#navGoalInputY").value = goalPose.y;
      container.querySelector("#navGoalInputYaw").value = 0;
      recomputePreviewRoute(true);
      drawMap();
    }
  }

  function onMapMouseMove(e) {
    if (!canvas) return;
    const { cx, cy } = clientToCanvas(e);
    if (cx >= 0 && cx <= viewW && cy >= 0 && cy <= viewH) {
      const { px, py } = canvasToImage(cx, cy);
      const { wx, wy } = imageToWorld(px, py);
      const overlay = container.querySelector("#navCoordsOverlay");
      if (overlay) {
        const info = queryCellInfo(styledMapCache.grid, px, py);
        let probeTag = "未探索区";
        if (info.state === "occupied") {
          probeTag = "墙体占用";
        } else if (info.state === "free" && info.clearanceM != null) {
          const status = info.clearanceM < 0.34 ? "碰撞风险" : info.clearanceM < 0.65 ? "窄通道" : "安全通行";
          probeTag = `壁距 ${info.clearanceM.toFixed(2)}m · ${status}`;
        }
        overlay.textContent = `X: ${wx.toFixed(2)}m | Y: ${wy.toFixed(2)}m | ${probeTag}`;
      }
      if (measureMode && measurePoints.length === 1) {
        measureHover = { wx, wy };
        drawMap();
      }
    }
    if (isPanning) {
      offsetX = cx - panStartX;
      offsetY = cy - panStartY;
      drawMap();
      return;
    }
    if (isDraggingGoal && dragStartPos) {
      const { px, py } = canvasToImage(cx, cy);
      const { wx, wy } = imageToWorld(px, py);
      const dx = wx - dragStartPos.wx;
      const dy = wy - dragStartPos.wy;
      if (Math.hypot(dx, dy) > 0.1) {
        const yaw = Math.atan2(dy, dx);
        goalPose.yaw = Math.round(yaw * 100) / 100;
        container.querySelector("#navGoalInputYaw").value = Math.round((yaw * 180) / Math.PI);
        drawMap();
      }
    }
  }

  function onMapMouseUp() {
    const wasDragging = isDraggingGoal;
    isPanning = false;
    isDraggingGoal = false;
    dragStartPos = null;
    if (wasDragging) drawMap();
  }

  function onMapWheel(e) {
    e.preventDefault();
    const { cx, cy } = clientToCanvas(e);
    const zoomFactor = e.deltaY < 0 ? 1.15 : 0.87;
    const newScale = Math.min(Math.max(scale * zoomFactor, 0.35), 10.0);
    offsetX = cx - (cx - offsetX) * (newScale / scale);
    offsetY = cy - (cy - offsetY) * (newScale / scale);
    scale = newScale;
    drawMap();
  }

  const mapInteractions = bindMapInteractions({
    canvas, cloudCanvas, windowTarget: window,
    isEnabled:() => mapSupported && mapView === "occupancy2d" && !canvas.hidden,
    onDown: onMapMouseDown, onMove: onMapMouseMove, onUp: onMapMouseUp, onWheel: onMapWheel,
  });

  // Saved Waypoints
  async function loadWaypoints() {
    const listEl = container.querySelector("#navWaypointsList");
    if (!listEl) return;
    try {
      const res = await api.listWaypoints(activeMap?.id);
      waypoints = res.waypoints || [];
      if (!waypoints.length) {
        listEl.innerHTML = `<span style="font-size:12px; color:var(--text-muted);">当前地图暂无常用航点，可在上方选点后点击「存为航点」。</span>`;
      } else {
        listEl.innerHTML = waypoints.map(wp => `
          <div class="wp-chip" data-wpid="${escapeHtml(wp.id)}">
            <button type="button" class="wp-select-btn" data-wpid="${escapeHtml(wp.id)}" title="${escapeHtml(wp.note || "")}">
              <strong>${escapeHtml(wp.name)}</strong>
              <span style="font-family:var(--font-mono); color:var(--text-muted);">(${Number(wp.x).toFixed(2)}, ${Number(wp.y).toFixed(2)})</span>
            </button>
            <button type="button" class="wp-del-btn" data-wpid="${escapeHtml(wp.id)}" aria-label="删除航点">✕</button>
          </div>
        `).join("");

        listEl.querySelectorAll(".wp-select-btn").forEach(btn => {
          btn.addEventListener("click", () => {
            const wp = waypoints.find(w => w.id === btn.dataset.wpid);
            if (!wp) return;
            goalPose = { x: wp.x, y: wp.y, yaw: wp.yaw || 0 };
            container.querySelector("#navGoalInputX").value = wp.x;
            container.querySelector("#navGoalInputY").value = wp.y;
            container.querySelector("#navGoalInputYaw").value = Math.round(((wp.yaw || 0) * 180) / Math.PI);
            recomputePreviewRoute(true);
            drawMap();
            const overlayPath = plannedPath.length > 1 ? plannedPath : (previewRoute?.ok ? previewRoute.pointsWorld : []);
            if (mapView === "cloud3d") cloudViewer.setOverlays({ pose: latestRobotPose, goal: goalPose, path: overlayPath });
            notify(`已载入航点「${wp.name}」并生成 A* 避障预览路径`, "info");
          });
        });
        listEl.querySelectorAll(".wp-del-btn").forEach(btn => {
          btn.addEventListener("click", async () => {
            try {
              await api.deleteWaypoint(btn.dataset.wpid);
              notify("已删除导航航点", "info");
              loadWaypoints();
            } catch (err) {
              notify(`删除航点失败: ${err.message}`, "error");
            }
          });
        });
      }
      drawMap();
    } catch {
      waypoints = [];
    }
  }

  container.querySelector("#navSaveWaypointBtn").addEventListener("click", async () => {
    const x = parseFloat(container.querySelector("#navGoalInputX").value) || 0;
    const y = parseFloat(container.querySelector("#navGoalInputY").value) || 0;
    const yawDeg = parseFloat(container.querySelector("#navGoalInputYaw").value) || 0;
    const name = `航点_${new Date().toISOString().slice(14, 19).replace(":", "")}`;
    try {
      await api.saveWaypoint({
        robot_id: api.currentRobotId,
        map_id: activeMap?.id || "r1pro_20260918",
        name,
        x,
        y,
        yaw: (yawDeg * Math.PI) / 180,
        note: `保存自 ${activeMap?.id || "map"}`,
      });
      notify(`已保存航点「${name}」(${x.toFixed(2)}, ${y.toFixed(2)})`, "success");
      loadWaypoints();
    } catch (err) {
      notify(`保存航点失败: ${err.message}`, "error");
    }
  });

  // Live goal input sync -> instant A* route preview
  for (const sel of ["#navGoalInputX", "#navGoalInputY", "#navGoalInputYaw"]) {
    container.querySelector(sel)?.addEventListener("input", () => {
      const x = parseFloat(container.querySelector("#navGoalInputX").value);
      const y = parseFloat(container.querySelector("#navGoalInputY").value);
      const yawDeg = parseFloat(container.querySelector("#navGoalInputYaw").value) || 0;
      if (Number.isFinite(x) && Number.isFinite(y)) {
        goalPose = { x, y, yaw: (yawDeg * Math.PI) / 180 };
        recomputePreviewRoute(true);
        drawMap();
      }
    });
  }

  // Goal Inputs & Actions
  container.querySelector("#navSendGoalBtn").addEventListener("click", async () => {
    const x = parseFloat(container.querySelector("#navGoalInputX").value) || 0;
    const y = parseFloat(container.querySelector("#navGoalInputY").value) || 0;
    const yawDeg = parseFloat(container.querySelector("#navGoalInputYaw").value) || 0;
    const yawRad = (yawDeg * Math.PI) / 180;
    try {
      const res = await api.sendNavGoal(x, y, yawRad);
      goalPose = { x, y, yaw: yawRad };
      if (res.path_points) plannedPath = res.path_points;
      recomputePreviewRoute(true);
      drawMap();
      const overlayPath = plannedPath.length > 1 ? plannedPath : (previewRoute?.ok ? previewRoute.pointsWorld : []);
      if (mapView === "cloud3d") cloudViewer.setOverlays({ pose: latestRobotPose, goal: goalPose, path: overlayPath });
      notify(`已下发导航目标: (${x.toFixed(2)}m, ${y.toFixed(2)}m, ${yawDeg.toFixed(0)}°)`, "success");
    } catch (err) {
      notify(`下发导航目标失败: ${err.message}`, "error");
    }
  });

  container.querySelector("#navCancelGoalBtn").addEventListener("click", async () => {
    try {
      await api.cancelNav();
      goalPose = null;
      plannedPath = [];
      previewRoute = null;
      lastRouteKey = null;
      updateRouteBanner();
      drawMap();
      if (mapView === "cloud3d") cloudViewer.setOverlays({ pose: latestRobotPose, goal: null, path: [] });
      notify("已取消当前导航任务", "info");
    } catch (err) {
      notify(`取消导航失败: ${err.message}`, "error");
    }
  });

  // State Machine Mode Switches
  container.querySelector("#navSetManualBtn").addEventListener("click", async () => {
    try {
      await api.setNavMode("MANUAL");
      notify("底盘控制模式已切换至 MANUAL", "info");
    } catch (err) {
      notify(`切换至 MANUAL 失败: ${err.message}`, "error");
    }
  });

  container.querySelector("#navSetNavModeBtn").addEventListener("click", async () => {
    try {
      await api.setNavMode("NAVIGATION");
      notify("底盘控制模式已切换至 NAVIGATION", "success");
    } catch (err) {
      notify(`切换至 NAVIGATION 失败: ${err.message}`, "error");
    }
  });

  let isSpinning = false;
  container.querySelector("#navSpinToggleBtn").addEventListener("click", async () => {
    try {
      const next = !isSpinning;
      await api.startLocalizationSpin(next);
      isSpinning = next;
      container.querySelector("#navSpinToggleBtn").textContent = isSpinning ? "停止定位旋转 (运行中)" : "启动定位旋转";
      notify(isSpinning ? "已启动原地 360° 定位旋转" : "已停止定位旋转", "info");
    } catch (err) {
      notify(`旋转定位指令失败: ${err.message}`, "error");
    }
  });

  // Speed Sliders
  const linSlider = container.querySelector("#navLinearSpeedSlider");
  const angSlider = container.querySelector("#navAngularSpeedSlider");
  linSlider.addEventListener("input", () => {
    container.querySelector("#navLinearSpeedVal").textContent = `${parseFloat(linSlider.value).toFixed(2)} m/s`;
  });
  angSlider.addEventListener("input", () => {
    container.querySelector("#navAngularSpeedVal").textContent = `${parseFloat(angSlider.value).toFixed(2)} rad/s`;
  });

  // Chassis Manual Dead-man Hold & Click
  let lastTeleopWarnTs = 0;
  function triggerChassis(vx, vy, omega, durationS = 0.4) {
    api.sendChassisVelocity(vx, vy, omega, durationS).catch((err) => {
      stopHold();
      const now = Date.now();
      if (now - lastTeleopWarnTs > 1500) {
        lastTeleopWarnTs = now;
        notify(`底盘点动被拒: ${err.message}`, "error");
      }
    });
  }

  function stopHold(sendZero = false) {
    if (holdTimer) {
      clearInterval(holdTimer);
      holdTimer = null;
      if (sendZero) api.sendChassisVelocity(0, 0, 0, 0.2).catch(() => {});
    }
  }

  function bindHoldButton(selector, getVec) {
    const btn = container.querySelector(selector);
    if (!btn) return;
    btn.addEventListener("pointerdown", (e) => {
      if (e.button !== 0) return;
      stopHold();
      const send = () => {
        const [vx, vy, om] = getVec();
        triggerChassis(vx, vy, om, 0.35);
      };
      send();
      holdTimer = setInterval(send, 240);
    });
    for (const ev of ["pointerup", "pointerleave", "pointercancel"]) {
      btn.addEventListener(ev, () => stopHold(true));
    }
  }

  const getLin = () => parseFloat(linSlider.value);
  const getAng = () => parseFloat(angSlider.value);

  bindHoldButton("#btnChassisForward", () => [getLin(), 0, 0]);
  bindHoldButton("#btnChassisBackward", () => [-getLin(), 0, 0]);
  bindHoldButton("#btnChassisLeft", () => [0, getLin(), 0]);
  bindHoldButton("#btnChassisRight", () => [0, -getLin(), 0]);
  bindHoldButton("#btnChassisRotateLeft", () => [0, 0, getAng()]);
  bindHoldButton("#btnChassisRotateRight", () => [0, 0, -getAng()]);
  container.querySelector("#btnChassisStop").addEventListener("click", () => {
    stopHold();
    triggerChassis(0, 0, 0, 0.2);
  });

  const onWindowBlur = () => stopHold(true);
  window.addEventListener("blur", onWindowBlur);

  const onKeyDown = (e) => {
    if (!container.classList.contains("active") || !mapSupported) return;
    if (e.key === "Escape" && measureMode) {
      setMeasureMode(false);
      return;
    }
    if (["INPUT", "TEXTAREA", "SELECT"].includes(e.target.tagName)) return;
    const key = e.key.toLowerCase();
    if (key === "w") triggerChassis(getLin(), 0, 0);
    else if (key === "s") triggerChassis(-getLin(), 0, 0);
    else if (key === "a") triggerChassis(0, getLin(), 0);
    else if (key === "d") triggerChassis(0, -getLin(), 0);
    else if (key === "q") triggerChassis(0, 0, getAng());
    else if (key === "e") triggerChassis(0, 0, -getAng());
    else if (key === "x") triggerChassis(0, 0, 0, 0.2);
  };
  window.addEventListener("keydown", onKeyDown);

  async function refreshMapCatalog(robotId) {
    mapCatalogLoadedFor = robotId;
    try {
      const meta = await api.getNavMap();
      if (!meta.supported || !meta.maps?.length) return;
      mapCatalog = meta.maps;
      mapSelect.innerHTML = mapCatalog.map(m => `<option value="${escapeHtml(m.id)}">${escapeHtml(m.name)}</option>`).join("");
      const def = mapCatalog.find(m => m.id === meta.default_map_id) || mapCatalog[0];
      mapSelect.value = def.id;
      applySelectedMap(def);
    } catch (err) {
      console.warn("Load nav map metadata failed:", err);
    }
  }

  return {
    update(state) {
      if (!state) return;
      const desc = state.descriptor || {};
      const obs = state.observation || {};
      const auth = state.authority || {};
      const nav = obs.navigation || {};
      const navCap = !!(desc.capabilities && desc.capabilities.navigation);
      mapSupported = navCap;
      if (!navCap) {
        stopHold();
        mapInteractions.reset();
        cloudViewer.setActive(false);
      }

      const notSupportedEl = container.querySelector("#navNotSupportedCard");
      const mainWrapper = container.querySelector("#navMainWrapper");
      if (!navCap) {
        if (notSupportedEl) notSupportedEl.style.display = "block";
        if (mainWrapper) mainWrapper.style.opacity = "0.4";
        return;
      } else {
        if (notSupportedEl) notSupportedEl.style.display = "none";
        if (mainWrapper) mainWrapper.style.opacity = "1.0";
      }

      if (mapCatalogLoadedFor !== desc.id) {
        refreshMapCatalog(desc.id);
      }

      currentNavMode = nav.mode || "MANUAL";
      const modeBadge = container.querySelector("#navCurrentModeBadge");
      if (modeBadge) {
        modeBadge.textContent = currentNavMode;
        modeBadge.className = `badge ${currentNavMode === "NAVIGATION" ? "badge-running" : currentNavMode === "FAULT" ? "badge-failed" : "badge-no_result"}`;
      }

      isSpinning = !!nav.localization_spin_active;
      const spinBtn = container.querySelector("#navSpinToggleBtn");
      if (spinBtn) {
        spinBtn.textContent = isSpinning ? "停止定位旋转 (运行中)" : "启动定位旋转";
        spinBtn.className = isSpinning ? "btn btn-sm btn-danger" : "btn btn-sm";
      }

      const teleopBadge = container.querySelector("#navTeleopStatusBadge");
      if (teleopBadge) {
        if (!auth.lease_active) {
          teleopBadge.textContent = "未获取 Lease (只读)";
          teleopBadge.className = "badge badge-no_result";
        } else if (currentNavMode === "MANUAL") {
          teleopBadge.textContent = "MANUAL 允许按住微动";
          teleopBadge.className = "badge badge-passed";
        } else {
          teleopBadge.textContent = `${currentNavMode} 模式中`;
          teleopBadge.className = "badge badge-pending";
        }
      }

      if (nav.pose_map) {
        latestRobotPose = {
          x: Number(nav.pose_map.x) || 0,
          y: Number(nav.pose_map.y) || 0,
          yaw: Number(nav.pose_map.yaw) || 0,
        };
        const odomXY = container.querySelector("#navOdomXY");
        const odomYaw = container.querySelector("#navOdomYaw");
        if (odomXY) odomXY.textContent = `${latestRobotPose.x.toFixed(3)}m, ${latestRobotPose.y.toFixed(3)}m`;
        if (odomYaw) odomYaw.textContent = `${((latestRobotPose.yaw * 180) / Math.PI).toFixed(1)}°`;
      }

      if (nav.chassis_vel) {
        const velEl = container.querySelector("#navOdomVel");
        const omegaEl = container.querySelector("#navOdomOmega");
        if (velEl) velEl.textContent = `${Number(nav.chassis_vel.vx || 0).toFixed(2)}, ${Number(nav.chassis_vel.vy || 0).toFixed(2)} m/s`;
        if (omegaEl) omegaEl.textContent = `${Number(nav.chassis_vel.omega || 0).toFixed(2)} rad/s`;
      }

      if (Array.isArray(nav.path_points)) {
        plannedPath = nav.path_points;
      }
      if (nav.goal_map && !isDraggingGoal) {
        goalPose = {
          x: Number(nav.goal_map.x) || 0,
          y: Number(nav.goal_map.y) || 0,
          yaw: Number(nav.goal_map.yaw) || 0,
        };
      }

      if (goalPose) recomputePreviewRoute(false);
      if (mapView === "occupancy2d") {
        syncCanvasSize(false);
        syncAnimLoop();
      }
      drawMap();
      const overlayPath = plannedPath.length > 1 ? plannedPath : (previewRoute?.ok ? previewRoute.pointsWorld : []);
      if (mapView === "cloud3d") cloudViewer.setOverlays({ pose: latestRobotPose, goal: goalPose, path: overlayPath });
    },
    destroy() {
      stopHold();
      stopAnimLoop();
      if (mapResizeObserver) mapResizeObserver.disconnect();
      window.removeEventListener("keydown", onKeyDown);
      window.removeEventListener("blur", onWindowBlur);
      mapInteractions.destroy();
      cloudViewer.destroy();
    },
  };
}
