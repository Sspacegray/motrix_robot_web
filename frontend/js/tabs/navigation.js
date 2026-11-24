// Tab 3: 2D occupancy map, Mock 3D point cloud, and chassis control
import { api } from "../api.js";
import { occupancyToPointCloud, lumaFromRgba, MOCK_CLOUD_LABEL } from "../map-cloud.js";
import { createMapCloudViewer } from "../map-cloud-viewer.js";
import { bindMapInteractions } from "../map-interactions.js";

export function initNavigationTab(container) {
  let mapImage = null;
  let mapYaml = {
    resolution: 0.05,
    origin: [-9.19, -7.85, 0],
    width: 321,
    height: 349,
  };
  let goalPose = null; // { x, y, yaw }
  let isDraggingGoal = false;
  let dragStartPos = null;
  let currentNavMode = "MANUAL";
  let manualInterval = null;
  let mapSupported = false;

  container.innerHTML = `
    <!-- Non-navigation Robot Banner (hidden if navigation supported) -->
    <div id="navNotSupportedCard" class="card" style="display:none; border-left: 4px solid var(--warning); background:#fffbeb;">
      <div style="font-weight:600; color:#b45309; margin-bottom:4px;">机型未装配移动底盘与导航套件</div>
      <div style="font-size:13px; color:#78350f;">
        当前选中机型（如 Piper 双臂台架或 Dexforce W1 人形）未配置移动底盘。2D 占用栅格与 3D 点云入口保留可见，但按能力禁用，不把 R1 的 mx_map 或其他机型数据当作本机采集。
      </div>
    </div>

    <div id="navMainWrapper" class="grid-2" style="grid-template-columns: 1fr 380px; align-items: start;">
      <!-- Left: 2D Map Canvas & Goal Selection -->
      <div class="card" style="margin-bottom:0;">
        <div class="card-header" style="padding-bottom:10px;">
          <div>
            <div style="display:flex; align-items:center; gap:8px;">
              <span class="card-title">地图导航 (mx_map)</span>
              <span class="badge badge-info" id="navResolutionBadge">分辨率: 0.05m/pix</span>
            </div>
            <div class="map-view-tabs" role="tablist" aria-label="地图视图">
              <button type="button" class="btn btn-sm sub-tab-btn" data-map-view="occupancy2d" aria-selected="true">2D 占用栅格</button>
              <button type="button" class="btn btn-sm sub-tab-btn" data-map-view="cloud3d" aria-selected="false">3D 点云</button>
            </div>
          </div>
          <div style="display:flex; align-items:center; gap:8px;">
            <button class="btn btn-sm" id="navResetViewBtn">重置视角</button>
            <button class="btn btn-sm btn-danger" id="navCancelGoalBtn">取消当前导航</button>
          </div>
        </div>

        <!-- Canvas Container -->
        <p class="preview-field-note" id="mapCloudNote">2D 占用栅格 · 本地 mx_map 参考图，非实时采集。拖动仅设置本地目标；下发仍需显式操作与 Lease。</p>
        <div style="position:relative; width:100%; height:460px; background:#1e293b; border-radius:var(--radius-sm); overflow:hidden; border:1px solid var(--border);">
          <canvas id="navMapCanvas" width="700" height="460" style="width:100%; height:100%; display:block; cursor:crosshair;"></canvas>
          <canvas id="navCloudCanvas" class="nav-cloud-canvas" width="700" height="460" hidden></canvas>
          <div id="navCloudHud" class="nav-cloud-hud" hidden></div>
          <div style="position:absolute; bottom:12px; left:12px; background:rgba(15,23,42,0.8); color:#f8fafc; font-size:11px; padding:4px 8px; border-radius:4px; font-family:var(--font-mono); pointer-events:none;" id="navCoordsOverlay">
            X: 0.00m | Y: 0.00m | Yaw: 0.0°
          </div>
          <div id="navMapLegend" style="position:absolute; top:12px; right:12px; display:flex; flex-direction:column; gap:4px; font-size:11px; color:#cbd5e1; background:rgba(15,23,42,0.75); padding:6px 10px; border-radius:4px;">
            <div><span style="display:inline-block; width:10px; height:10px; border-radius:50%; background:#0d9488; margin-right:4px;"></span>Mock 机器人位姿</div>
            <div><span style="display:inline-block; width:10px; height:10px; border-radius:50%; background:#f59e0b; margin-right:4px;"></span>目标位姿 (Goal)</div>
            <div><span style="display:inline-block; width:12px; height:2px; background:#38bdf8; margin-right:4px; vertical-align:middle;"></span>Mock 规划路径</div>
          </div>
        </div>

        <!-- Manual Goal Inputs Bar -->
        <div style="margin-top:12px; display:flex; gap:10px; align-items:center; background:var(--bg-subtle); padding:10px 14px; border-radius:var(--radius-sm);">
          <span style="font-size:12px; font-weight:600; color:var(--text-muted);">目标位姿:</span>
          <div style="display:flex; align-items:center; gap:4px;">
            <label style="font-size:12px; color:var(--text-muted);">X (m)</label>
            <input type="number" id="navGoalInputX" step="0.1" value="0.0" style="width:70px; padding:4px 6px; font-size:12px; border:1px solid var(--border); border-radius:var(--radius-sm); font-family:var(--font-mono);">
          </div>
          <div style="display:flex; align-items:center; gap:4px;">
            <label style="font-size:12px; color:var(--text-muted);">Y (m)</label>
            <input type="number" id="navGoalInputY" step="0.1" value="0.0" style="width:70px; padding:4px 6px; font-size:12px; border:1px solid var(--border); border-radius:var(--radius-sm); font-family:var(--font-mono);">
          </div>
          <div style="display:flex; align-items:center; gap:4px;">
            <label style="font-size:12px; color:var(--text-muted);">Yaw (°)</label>
            <input type="number" id="navGoalInputYaw" step="5" value="0" style="width:65px; padding:4px 6px; font-size:12px; border:1px solid var(--border); border-radius:var(--radius-sm); font-family:var(--font-mono);">
          </div>
          <button class="btn btn-sm btn-primary" id="navSendGoalBtn" style="margin-left:auto;">下发导航目标</button>
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
          <div style="margin-top:12px; pt-2; border-top:1px solid var(--border); display:flex; justify-content:space-between; align-items:center;">
            <span style="font-size:13px; font-weight:500;">原地旋转自定位 (Spin)</span>
            <button class="btn btn-sm" id="navSpinToggleBtn">启动定位旋转</button>
          </div>
        </div>

        <!-- Manual Chassis Teleop D-Pad -->
        <div class="card" style="margin-bottom:0;">
          <div class="card-header" style="margin-bottom:10px;">
            <span class="card-title">底盘手动微调 (Teleoperation)</span>
            <span class="badge badge-idle" id="navTeleopStatusBadge">MANUAL 模式可控</span>
          </div>
          <div style="font-size:12px; color:var(--text-muted); margin-bottom:12px;">
            全向/差速点动控制。支持键盘 <kbd>W</kbd>/<kbd>S</kbd>/<kbd>A</kbd>/<kbd>D</kbd>/<kbd>Q</kbd>/<kbd>E</kbd> 快捷点动。单次触发持续 0.4s。
          </div>

          <!-- Speed sliders -->
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

          <!-- D-Pad Grid -->
          <div style="display:grid; grid-template-columns: repeat(3, 1fr); gap:8px; width:220px; margin:0 auto;">
            <button class="btn btn-sm" id="btnChassisRotateLeft" title="逆时针旋转 (Q)">↺ Q</button>
            <button class="btn btn-sm btn-primary" id="btnChassisForward" title="前进 (W)">▲ W</button>
            <button class="btn btn-sm" id="btnChassisRotateRight" title="顺时针旋转 (E)">↻ E</button>

            <button class="btn btn-sm btn-primary" id="btnChassisLeft" title="左平移 (A)">◀ A</button>
            <button class="btn btn-sm btn-danger" id="btnChassisStop" title="停止底盘 (X)">■ 停</button>
            <button class="btn btn-sm btn-primary" id="btnChassisRight" title="右平移 (D)">▶ D</button>

            <div></div>
            <button class="btn btn-sm btn-primary" id="btnChassisBackward" title="后退 (S)">▼ S</button>
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

  // Load map image
  mapImage = new Image();
  mapImage.src = "/assets/mx_map.png";
  mapImage.onload = () => {
    resetView();
    drawMap();
    maybeBuildCloud();
  };

  const cloudCanvas = container.querySelector("#navCloudCanvas");
  const cloudViewer = createMapCloudViewer({ canvas: cloudCanvas, hud: container.querySelector("#navCloudHud") });
  let mapView = "occupancy2d";
  let cloudReady = false;

  function maybeBuildCloud() {
    if (cloudReady || !mapImage.complete || !mapImage.naturalWidth) return;
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
    cloudViewer.setCloud(cloud);
    cloudReady = true;
  }

  function setMapView(next) {
    mapInteractions.reset();
    mapView = next === "cloud3d" ? "cloud3d" : "occupancy2d";
    container.querySelectorAll("[data-map-view]").forEach(btn => {
      btn.setAttribute("aria-selected", btn.dataset.mapView === mapView ? "true" : "false");
    });
    const threeD = mapView === "cloud3d";
    canvas.hidden = threeD;
    cloudCanvas.hidden = !threeD;
    container.querySelector("#navCloudHud").hidden = !threeD;
    container.querySelector("#navCoordsOverlay").hidden = threeD;
    container.querySelector("#mapCloudNote").textContent = threeD
      ? MOCK_CLOUD_LABEL + " 旋转/缩放仅观察相机，不改变目标；目标选点仅在2D视图。"
      : "2D 占用栅格 · 本地 mx_map 参考图，非实时采集。拖动仅设置本地目标；下发仍需显式操作与 Lease。";
    container.querySelector("#navMapLegend").innerHTML = threeD
      ? "<div>灰色：稀疏自由地面 · 绿色：占用格拉伸</div><div>青色：Mock位姿 · 橙色：已有目标（只读）</div>"
      : "<div>青色：Mock位姿 · 橙色：本地目标</div><div>蓝色虚线：Mock规划路径</div>";
    if (threeD) { maybeBuildCloud(); cloudViewer.resize(); cloudViewer.setActive(true); cloudViewer.setOverlays({ pose: latestRobotPose, goal: goalPose, path: plannedPath }); }
    else cloudViewer.setActive(false);
  }
  container.querySelectorAll("[data-map-view]").forEach(btn => btn.addEventListener("click", () => setMapView(btn.dataset.mapView)));

  function resetView() {
    if (!canvas || !mapImage) return;
    const aspect = canvas.width / canvas.height;
    scale = Math.min(canvas.width / (mapImage.width || 321), canvas.height / (mapImage.height || 349)) * 1.1;
    offsetX = (canvas.width - (mapImage.width || 321) * scale) / 2;
    offsetY = (canvas.height - (mapImage.height || 349) * scale) / 2;
    drawMap();
  }

  container.querySelector("#navResetViewBtn").addEventListener("click", () => { if (mapView === "cloud3d") cloudViewer.resetView(); else resetView(); });

  // Coordinate transforms
  // world (meters) -> pixel on original image
  function worldToImage(wx, wy) {
    const px = (wx - mapYaml.origin[0]) / mapYaml.resolution;
    const py = (mapImage ? mapImage.height : mapYaml.height) - (wy - mapYaml.origin[1]) / mapYaml.resolution;
    return { px, py };
  }

  // pixel on original image -> world (meters)
  function imageToWorld(px, py) {
    const wx = mapYaml.origin[0] + px * mapYaml.resolution;
    const wy = mapYaml.origin[1] + ((mapImage ? mapImage.height : mapYaml.height) - py) * mapYaml.resolution;
    return { wx, wy };
  }

  // screen canvas (x, y) -> original image pixel (px, py)
  function canvasToImage(cx, cy) {
    const px = (cx - offsetX) / scale;
    const py = (cy - offsetY) / scale;
    return { px, py };
  }

  function imageToCanvas(px, py) {
    const cx = offsetX + px * scale;
    const cy = offsetY + py * scale;
    return { cx, cy };
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

    // Background grid
    ctx.save();
    ctx.fillStyle = "#1e293b";
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    // Draw Map Image
    if (mapImage && mapImage.complete && mapImage.naturalWidth !== 0) {
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(mapImage, offsetX, offsetY, mapImage.width * scale, mapImage.height * scale);
    }

    // Origin crosshair
    const origC = worldToCanvas(0, 0);
    ctx.strokeStyle = "rgba(255, 255, 255, 0.25)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(origC.cx - 10, origC.cy);
    ctx.lineTo(origC.cx + 10, origC.cy);
    ctx.moveTo(origC.cx, origC.cy - 10);
    ctx.lineTo(origC.cx, origC.cy + 10);
    ctx.stroke();

    // Draw planned path
    if (plannedPath && plannedPath.length > 1) {
      ctx.strokeStyle = "#38bdf8";
      ctx.lineWidth = 2.5;
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      for (let i = 0; i < plannedPath.length; i++) {
        const pt = worldToCanvas(plannedPath[i][0], plannedPath[i][1]);
        if (i === 0) ctx.moveTo(pt.cx, pt.cy);
        else ctx.lineTo(pt.cx, pt.cy);
      }
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // Draw goal pose (if set)
    if (goalPose) {
      const gC = worldToCanvas(goalPose.x, goalPose.y);
      ctx.fillStyle = "#f59e0b";
      ctx.beginPath();
      ctx.arc(gC.cx, gC.cy, 7, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 2;
      ctx.stroke();

      // Heading arrow
      const arrowLen = 22;
      const angle = -goalPose.yaw; // canvas Y is inverted
      const endX = gC.cx + Math.cos(angle) * arrowLen;
      const endY = gC.cy + Math.sin(angle) * arrowLen;
      ctx.strokeStyle = "#f59e0b";
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(gC.cx, gC.cy);
      ctx.lineTo(endX, endY);
      ctx.stroke();
    }

    // Draw robot pose
    const rC = worldToCanvas(latestRobotPose.x, latestRobotPose.y);
    ctx.fillStyle = "#0d9488";
    ctx.beginPath();
    ctx.arc(rC.cx, rC.cy, 9, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = "#ffffff";
    ctx.lineWidth = 2;
    ctx.stroke();

    // Robot heading direction ray
    const rArrowLen = 26;
    const rAngle = -latestRobotPose.yaw;
    const rEndX = rC.cx + Math.cos(rAngle) * rArrowLen;
    const rEndY = rC.cy + Math.sin(rAngle) * rArrowLen;
    ctx.strokeStyle = "#0d9488";
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(rC.cx, rC.cy);
    ctx.lineTo(rEndX, rEndY);
    ctx.stroke();

    ctx.restore();
  }

  // Canvas Mouse Interactions: Click & Drag to set Goal Pose, Right-Click / Alt-Click to Pan
  function onMapMouseDown(e) {
    const rect = canvas.getBoundingClientRect();
    const cx = e.clientX - rect.x;
    const cy = e.clientY - rect.y;

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
    const rect = canvas.getBoundingClientRect();
    const cx = e.clientX - rect.x;
    const cy = e.clientY - rect.y;

    // Hover coords
    if (cx >= 0 && cx <= canvas.width && cy >= 0 && cy <= canvas.height) {
      const { px, py } = canvasToImage(cx, cy);
      const { wx, wy } = imageToWorld(px, py);
      const overlay = container.querySelector("#navCoordsOverlay");
      if (overlay) {
        overlay.textContent = `X: ${wx.toFixed(2)}m | Y: ${wy.toFixed(2)}m`;
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
        const deg = Math.round((yaw * 180) / Math.PI);
        container.querySelector("#navGoalInputYaw").value = deg;
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
    const rect = canvas.getBoundingClientRect();
    const cx = e.clientX - rect.x;
    const cy = e.clientY - rect.y;
    const zoomFactor = e.deltaY < 0 ? 1.15 : 0.87;

    const newScale = Math.min(Math.max(scale * zoomFactor, 0.4), 8.0);
    offsetX = cx - (cx - offsetX) * (newScale / scale);
    offsetY = cy - (cy - offsetY) * (newScale / scale);
    scale = newScale;
    drawMap();
  }

  const mapInteractions = bindMapInteractions({
    canvas, cloudCanvas, windowTarget:window,
    isEnabled:() => mapSupported && mapView === "occupancy2d" && !canvas.hidden,
    onDown:onMapMouseDown, onMove:onMapMouseMove, onUp:onMapMouseUp, onWheel:onMapWheel,
  });

  // Goal Inputs & Actions
  container.querySelector("#navSendGoalBtn").addEventListener("click", async () => {
    const x = parseFloat(container.querySelector("#navGoalInputX").value) || 0;
    const y = parseFloat(container.querySelector("#navGoalInputY").value) || 0;
    const yawDeg = parseFloat(container.querySelector("#navGoalInputYaw").value) || 0;
    const yawRad = (yawDeg * Math.PI) / 180;
    try {
      await api.sendNavGoal(x, y, yawRad);
      goalPose = { x, y, yaw: yawRad };
      drawMap();
    } catch (err) {
      alert(`下发导航目标失败: ${err.message}`);
    }
  });

  container.querySelector("#navCancelGoalBtn").addEventListener("click", async () => {
    try {
      await api.cancelNav();
      goalPose = null;
      plannedPath = [];
      drawMap();
    } catch (err) {
      alert(`取消导航失败: ${err.message}`);
    }
  });

  // State Machine Mode Switches
  container.querySelector("#navSetManualBtn").addEventListener("click", async () => {
    try {
      await api.setNavMode("MANUAL");
    } catch (err) {
      alert(`切换至 MANUAL 失败: ${err.message}`);
    }
  });

  container.querySelector("#navSetNavModeBtn").addEventListener("click", async () => {
    try {
      await api.setNavMode("NAVIGATION");
    } catch (err) {
      alert(`切换至 NAVIGATION 失败: ${err.message}`);
    }
  });

  let isSpinning = false;
  container.querySelector("#navSpinToggleBtn").addEventListener("click", async () => {
    try {
      isSpinning = !isSpinning;
      await api.startLocalizationSpin(isSpinning);
      container.querySelector("#navSpinToggleBtn").textContent = isSpinning ? "停止定位旋转" : "启动定位旋转";
    } catch (err) {
      alert(`旋转定位指令失败: ${err.message}`);
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

  // Chassis Manual Buttons
  function triggerChassis(vx, vy, omega) {
    api.sendChassisVelocity(vx, vy, omega, 0.4).catch((err) => {
      console.warn("Chassis command failed:", err.message);
    });
  }

  const getLin = () => parseFloat(linSlider.value);
  const getAng = () => parseFloat(angSlider.value);

  container.querySelector("#btnChassisForward").addEventListener("click", () => triggerChassis(getLin(), 0, 0));
  container.querySelector("#btnChassisBackward").addEventListener("click", () => triggerChassis(-getLin(), 0, 0));
  container.querySelector("#btnChassisLeft").addEventListener("click", () => triggerChassis(0, getLin(), 0));
  container.querySelector("#btnChassisRight").addEventListener("click", () => triggerChassis(0, -getLin(), 0));
  container.querySelector("#btnChassisRotateLeft").addEventListener("click", () => triggerChassis(0, 0, getAng()));
  container.querySelector("#btnChassisRotateRight").addEventListener("click", () => triggerChassis(0, 0, -getAng()));
  container.querySelector("#btnChassisStop").addEventListener("click", () => triggerChassis(0, 0, 0));

  // Keyboard shortcut listener when tab active
  window.addEventListener("keydown", (e) => {
    if (container.closest(".tab-pane.active") === null) return;
    if (e.target.tagName === "INPUT" || e.target.tagName === "TEXTAREA") return;
    const key = e.key.toLowerCase();
    if (key === "w") triggerChassis(getLin(), 0, 0);
    else if (key === "s") triggerChassis(-getLin(), 0, 0);
    else if (key === "a") triggerChassis(0, getLin(), 0);
    else if (key === "d") triggerChassis(0, -getLin(), 0);
    else if (key === "q") triggerChassis(0, 0, getAng());
    else if (key === "e") triggerChassis(0, 0, -getAng());
    else if (key === "x") triggerChassis(0, 0, 0);
  });

  return {
    update(state) {
      if (!state) return;
      const desc = state.descriptor || {};
      const obs = state.observation || {};
      const auth = state.authority || {};
      const navCap = !!(desc.capabilities && desc.capabilities.navigation);
      mapSupported = navCap;
      if (!navCap) { mapInteractions.reset(); cloudViewer.setActive(false); }

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

      // Update Nav Mode Badge
      currentNavMode = obs.navigation_mode || "MANUAL";
      const modeBadge = container.querySelector("#navCurrentModeBadge");
      if (modeBadge) {
        modeBadge.textContent = currentNavMode;
        modeBadge.className = `badge ${currentNavMode === "NAVIGATION" ? "badge-running" : currentNavMode === "FAULT" ? "badge-failed" : "badge-idle"}`;
      }

      // Teleop Status Badge
      const teleopBadge = container.querySelector("#navTeleopStatusBadge");
      if (teleopBadge) {
        if (!auth.lease_active) {
          teleopBadge.textContent = "未获取 Lease (只读)";
          teleopBadge.className = "badge badge-idle";
        } else if (currentNavMode === "MANUAL") {
          teleopBadge.textContent = "MANUAL 允许点动";
          teleopBadge.className = "badge badge-passed";
        } else {
          teleopBadge.textContent = `${currentNavMode} 模式中`;
          teleopBadge.className = "badge badge-warning";
        }
      }

      // Update Robot Pose & planned path
      if (obs.chassis_pose) {
        latestRobotPose = {
          x: obs.chassis_pose.x || 0,
          y: obs.chassis_pose.y || 0,
          yaw: obs.chassis_pose.yaw || 0,
        };
        const odomXY = container.querySelector("#navOdomXY");
        const odomYaw = container.querySelector("#navOdomYaw");
        if (odomXY) odomXY.textContent = `${latestRobotPose.x.toFixed(3)}m, ${latestRobotPose.y.toFixed(3)}m`;
        if (odomYaw) odomYaw.textContent = `${((latestRobotPose.yaw * 180) / Math.PI).toFixed(1)}°`;
      }

      if (obs.chassis_velocity) {
        const velEl = container.querySelector("#navOdomVel");
        const omegaEl = container.querySelector("#navOdomOmega");
        if (velEl) velEl.textContent = `${obs.chassis_velocity.vx.toFixed(2)}, ${obs.chassis_velocity.vy.toFixed(2)} m/s`;
        if (omegaEl) omegaEl.textContent = `${obs.chassis_velocity.omega.toFixed(2)} rad/s`;
      }

      if (obs.navigation_path) {
        plannedPath = obs.navigation_path;
      }

      drawMap();
      if (mapView === "cloud3d") cloudViewer.setOverlays({ pose: latestRobotPose, goal: goalPose, path: plannedPath });
    },
    destroy() { mapInteractions.destroy(); cloudViewer.destroy(); },
  };
}
