// Tab 3: R1 Pro Real 2D Occupancy Maps, Real 3D LiDAR Point Cloud (.pcd), Waypoints & Chassis Teleop
import { api } from "../api.js";
import { occupancyToPointCloud, lumaFromRgba, MOCK_CLOUD_LABEL } from "../map-cloud.js";
import { parsePcdBuffer, buildColoredPcdCloud, RECORDED_PCD_LABEL } from "../pcd-loader.js";
import { createMapCloudViewer } from "../map-cloud-viewer.js";
import { bindMapInteractions } from "../map-interactions.js";
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
          <p class="preview-field-note" id="mapCloudNote">2D 真实占用栅格 · 拖动仅设置本地目标；下发仍需显式操作与 Lease。</p>
          <div style="position:relative; width:100%; height:480px; background:#0f172a; border-radius:var(--radius-sm); overflow:hidden; border:1px solid var(--border);">
            <canvas id="navMapCanvas" width="760" height="480" style="width:100%; height:100%; display:block; cursor:crosshair;"></canvas>
            <canvas id="navCloudCanvas" class="nav-cloud-canvas" width="760" height="480" hidden></canvas>
            <div id="navCloudHud" class="nav-cloud-hud" hidden></div>
            <div style="position:absolute; bottom:12px; left:12px; background:rgba(15,23,42,0.84); color:#f8fafc; font-size:11px; padding:4px 8px; border-radius:4px; font-family:var(--font-mono); pointer-events:none;" id="navCoordsOverlay">
              X: 0.00m | Y: 0.00m | Goal Yaw: 0°
            </div>
            <div id="navMapLegend" style="position:absolute; top:12px; right:12px; display:flex; flex-direction:column; gap:4px; font-size:11px; color:#cbd5e1; background:rgba(15,23,42,0.78); padding:6px 10px; border-radius:4px;">
              <div>青色：当前位姿 · 橙色：本地目标 · 蓝色虚线：规划路径</div>
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

  const canvas = container.querySelector("#navMapCanvas");
  const ctx = canvas.getContext("2d");
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

  function load2DImage(url) {
    mapImage = new Image();
    mapImage.src = url;
    mapImage.onload = () => {
      resetView();
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
    if (resBadge) resBadge.textContent = `分辨率: ${mapYaml.resolution}m/pix`;
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
      : `2D 真实占用栅格 (${activeMap?.id || "map"} · ${mapYaml.resolution}m/pix) · 左键拖拽设目标，右键/Shift 平移，滚轮缩放。`;
    container.querySelector("#navMapLegend").innerHTML = threeD
      ? (isRealPcd
          ? "<div>色谱：真实 LiDAR 高程/强度/语义 · 支持顶棚 Z 切片</div><div>青色：当前位姿 · 橙色：已有目标（只读）</div>"
          : "<div>灰色：稀疏自由地面 · 绿色：占用格拉伸</div><div>青色：当前位姿 · 橙色：已有目标（只读）</div>")
      : "<div>青色：当前位姿 · 橙色：本地目标 · 紫点：常用航点</div><div>蓝色虚线：规划路径</div>";
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
    container.querySelector("#navCloudControls").hidden = !(threeD && isRealPcd);
    updateMapNotes();
    if (threeD) {
      ensureActiveCloud(false);
      cloudViewer.resize();
      cloudViewer.setActive(true);
      cloudViewer.setOverlays({ pose: latestRobotPose, goal: goalPose, path: plannedPath });
    } else {
      cloudViewer.setActive(false);
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
    scale = Math.min(canvas.width / iw, canvas.height / ih) * 0.94;
    offsetX = (canvas.width - iw * scale) / 2;
    offsetY = (canvas.height - ih * scale) / 2;
    drawMap();
  }

  container.querySelector("#navResetViewBtn").addEventListener("click", () => {
    if (mapView === "cloud3d") cloudViewer.resetView();
    else resetView();
  });

  // Coordinate transforms
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

  // Drawing
  let latestRobotPose = { x: 0, y: 0, yaw: 0 };
  let plannedPath = [];

  function drawMap() {
    if (!ctx) return;
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    ctx.save();
    ctx.fillStyle = "#0f172a";
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    if (mapImage && mapImage.complete && mapImage.naturalWidth !== 0) {
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(mapImage, offsetX, offsetY, mapImage.naturalWidth * scale, mapImage.naturalHeight * scale);
    }

    // Origin crosshair
    const origC = worldToCanvas(0, 0);
    ctx.strokeStyle = "rgba(255, 255, 255, 0.35)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(origC.cx - 10, origC.cy);
    ctx.lineTo(origC.cx + 10, origC.cy);
    ctx.moveTo(origC.cx, origC.cy - 10);
    ctx.lineTo(origC.cx, origC.cy + 10);
    ctx.stroke();

    // Draw saved waypoints on 2D map with readable pill tags
    for (const wp of waypoints) {
      const wC = worldToCanvas(wp.x, wp.y);
      ctx.fillStyle = "rgba(168, 85, 247, 0.95)";
      ctx.beginPath();
      ctx.arc(wC.cx, wC.cy, 5, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 1.5;
      ctx.stroke();
      ctx.font = "10px sans-serif";
      const tw = ctx.measureText(wp.name).width;
      ctx.fillStyle = "rgba(15, 23, 42, 0.82)";
      ctx.fillRect(wC.cx + 7, wC.cy - 8, tw + 8, 15);
      ctx.fillStyle = "#f3e8ff";
      ctx.fillText(wp.name, wC.cx + 11, wC.cy + 3);
    }

    // Draw planned path
    if (plannedPath && plannedPath.length > 1) {
      ctx.strokeStyle = "#38bdf8";
      ctx.lineWidth = 2.5;
      ctx.setLineDash([5, 4]);
      ctx.beginPath();
      for (let i = 0; i < plannedPath.length; i++) {
        const pt = worldToCanvas(plannedPath[i][0], plannedPath[i][1]);
        if (i === 0) ctx.moveTo(pt.cx, pt.cy);
        else ctx.lineTo(pt.cx, pt.cy);
      }
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // Draw goal pose
    if (goalPose) {
      const gC = worldToCanvas(goalPose.x, goalPose.y);
      ctx.fillStyle = "#f59e0b";
      ctx.beginPath();
      ctx.arc(gC.cx, gC.cy, 7, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 2;
      ctx.stroke();

      const angle = -goalPose.yaw;
      ctx.strokeStyle = "#f59e0b";
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(gC.cx, gC.cy);
      ctx.lineTo(gC.cx + Math.cos(angle) * 22, gC.cy + Math.sin(angle) * 22);
      ctx.stroke();
    }

    // Draw robot pose
    const rC = worldToCanvas(latestRobotPose.x, latestRobotPose.y);
    ctx.fillStyle = "#14b8a6";
    ctx.beginPath();
    ctx.arc(rC.cx, rC.cy, 8, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 2;
    ctx.stroke();

    const rAngle = -latestRobotPose.yaw;
    ctx.strokeStyle = "#14b8a6";
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(rC.cx, rC.cy);
    ctx.lineTo(rC.cx + Math.cos(rAngle) * 24, rC.cy + Math.sin(rAngle) * 24);
    ctx.stroke();

    ctx.restore();
  }

  function clientToCanvas(e) {
    const rect = canvas.getBoundingClientRect();
    const scaleX = rect.width > 0 ? canvas.width / rect.width : 1;
    const scaleY = rect.height > 0 ? canvas.height / rect.height : 1;
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
      isDraggingGoal = true;
      const { px, py } = canvasToImage(cx, cy);
      const { wx, wy } = imageToWorld(px, py);
      dragStartPos = { wx, wy };
      goalPose = { x: Math.round(wx * 100) / 100, y: Math.round(wy * 100) / 100, yaw: 0 };
      container.querySelector("#navGoalInputX").value = goalPose.x;
      container.querySelector("#navGoalInputY").value = goalPose.y;
      container.querySelector("#navGoalInputYaw").value = 0;
      drawMap();
    }
  }

  function onMapMouseMove(e) {
    if (!canvas) return;
    const { cx, cy } = clientToCanvas(e);
    if (cx >= 0 && cx <= canvas.width && cy >= 0 && cy <= canvas.height) {
      const { px, py } = canvasToImage(cx, cy);
      const { wx, wy } = imageToWorld(px, py);
      const overlay = container.querySelector("#navCoordsOverlay");
      if (overlay) {
        const yawDeg = goalPose ? ((goalPose.yaw * 180) / Math.PI).toFixed(0) : "0";
        overlay.textContent = `X: ${wx.toFixed(2)}m | Y: ${wy.toFixed(2)}m | Goal Yaw: ${yawDeg}°`;
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
    isPanning = false;
    isDraggingGoal = false;
    dragStartPos = null;
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
            drawMap();
            if (mapView === "cloud3d") cloudViewer.setOverlays({ pose: latestRobotPose, goal: goalPose, path: plannedPath });
            notify(`已载入航点「${wp.name}」到目标框（点击下发导航目标执行）`, "info");
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
      drawMap();
      if (mapView === "cloud3d") cloudViewer.setOverlays({ pose: latestRobotPose, goal: goalPose, path: plannedPath });
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

      drawMap();
      if (mapView === "cloud3d") cloudViewer.setOverlays({ pose: latestRobotPose, goal: goalPose, path: plannedPath });
    },
    destroy() {
      stopHold();
      window.removeEventListener("keydown", onKeyDown);
      window.removeEventListener("blur", onWindowBlur);
      mapInteractions.destroy();
      cloudViewer.destroy();
    },
  };
}
