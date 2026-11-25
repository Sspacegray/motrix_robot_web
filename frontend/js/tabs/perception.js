// Tab 4: Visual Perception Workbench
import { api } from "../api.js";
import { escapeHtml } from "../ui-format.js";
import { notify } from "../notify.js";

export function initPerceptionTab(container) {
  let refreshTimer = null;
  let clicked3DPoint = null;
  let currentCameras = [];
  let currentRobotId = null;
  let slotStreams = ["head_left_rgb", "head_right_rgb", "wrist_left_rgb", "wrist_right_rgb"];
  let imgTick = 0;

  let depthEnabled = false;
  let depthReady = false;
  let yoloRunning = false;
  let latchedMask = null;
  let sixDRunning = false;
  let sixDPhase = "idle";
  let arucoRunning = false;
  let arucoFineEnabled = false;

  container.innerHTML = `
    <!-- Top Status Bar & Quick Actions -->
    <div class="card" style="margin-bottom:16px;">
      <div style="display:flex; justify-content:space-between; align-items:center; flex-wrap:wrap; gap:12px;">
        <div style="display:flex; align-items:center; gap:10px; flex-wrap:wrap;">
          <span class="card-title" style="margin:0;">视觉感知工作台 (Visual Perception)</span>
          <span class="badge badge-no_result" id="percepPipelineBadge">流水线: IDLE</span>
          <span class="badge badge-no_result" id="percepDepthGateBadge">头部深度: OFFLINE</span>
          <span class="badge badge-no_result" id="percepArucoBadge">ArUco: 未启动</span>
        </div>
        <div style="display:flex; gap:8px; flex-wrap:wrap;">
          <button class="btn btn-sm" id="percepToggleDepthBtn">启动头部深度相机</button>
          <button class="btn btn-sm btn-danger" id="percepResetPipelineBtn">重置感知流水线</button>
        </div>
      </div>
    </div>

    <!-- Main Grid: Left Camera Streams Grid + Right 3-Step Perception Pipeline -->
    <div class="grid-2" style="grid-template-columns: 1fr 430px; align-items:start;">
      <!-- Left: Camera Streams (4 Configurable Slots) -->
      <div style="display:flex; flex-direction:column; gap:16px;">
        <div class="card" style="margin-bottom:0;">
          <div class="card-header" style="margin-bottom:12px;">
            <div style="display:flex; align-items:center; gap:10px; flex-wrap:wrap;">
              <span class="card-title">多相机实时视流 (Configurable Stream Grid)</span>
              <span style="font-size:12px; color:var(--text-muted);">点击任意画面反算 base_link 3D 坐标</span>
            </div>
            <div style="display:flex; gap:6px;">
              <button class="btn btn-sm" data-cam-layout="grid" id="btnCamViewAll" style="background:var(--accent-light); color:var(--accent);">多画面平铺</button>
              <button class="btn btn-sm" data-cam-layout="single" id="btnCamViewHead">主视角单屏</button>
            </div>
          </div>

          <div id="cameraStreamsContainer" class="camera-grid">
            ${[0, 1, 2, 3].map(idx => `
              <div class="camera-slot" data-slot="${idx}">
                <div class="camera-slot-header">
                  <select class="cam-slot-select" data-slot="${idx}" aria-label="选择相机流槽位 ${idx + 1}" style="background:#0f172a; color:#e2e8f0; border:1px solid #334155; border-radius:4px; padding:2px 6px; font-size:11px; font-family:var(--font-mono); max-width:210px;"></select>
                  <span class="badge badge-running cam-slot-meta" data-slot="${idx}" style="font-size:10px; padding:2px 6px;">--</span>
                </div>
                <div class="camera-frame-wrap" data-slot="${idx}">
                  <img class="cam-slot-img" data-slot="${idx}" alt="camera slot ${idx + 1}">
                  <div class="cam-slot-crosshair" data-slot="${idx}" style="display:none; position:absolute; width:16px; height:16px; border:2px solid #f43f5e; border-radius:50%; transform:translate(-50%, -50%); pointer-events:none;"></div>
                </div>
              </div>
            `).join("")}
          </div>

          <!-- 3D Ray Backprojection Point Result Display -->
          <div style="margin-top:12px; background:var(--bg-subtle); padding:10px 14px; border-radius:var(--radius-sm); display:flex; align-items:center; justify-content:space-between; gap:10px; border:1px solid var(--border);">
            <div style="min-width:0;">
              <div style="font-size:11px; color:var(--text-muted);">像素点击反算 3D 点 (Camera Ray &amp; Depth → base_link):</div>
              <div style="font-family:var(--font-mono); font-size:12px; font-weight:600; color:var(--accent); overflow-wrap:anywhere;" id="percepCalculatedPointText">
                尚未点击画面。点击上方任意相机画面可拾取 metric 3D 坐标（head_left 需先开启头部深度 READY）。
              </div>
            </div>
            <button class="btn btn-sm" id="btnCopy3DPoint" style="flex-shrink:0;">复制 3D 坐标</button>
          </div>
        </div>

        <!-- Perception Event Logs -->
        <div class="card" style="margin-bottom:0;">
          <div class="card-header" style="margin-bottom:8px; padding-bottom:6px;">
            <span class="card-title" style="font-size:13px;">感知管线事件日志 (Perception Logs)</span>
            <span style="font-size:11px; color:var(--text-muted);">实时同步后端状态机</span>
          </div>
          <pre id="percepLogsPre" style="background:#0b0b0e; color:#e6e6ea; font-family:var(--font-mono); font-size:11px; height:128px; overflow-y:auto; padding:10px 12px; border-radius:var(--radius-sm); white-space:pre-wrap;">等待感知管线事件...</pre>
        </div>
      </div>

      <!-- Right: Strict 3-Step Perception Gate & ArUco -->
      <div style="display:flex; flex-direction:column; gap:16px;">
        <!-- Strict 3-Step Pipeline -->
        <div class="card" style="margin-bottom:0;">
          <div class="card-header" style="margin-bottom:10px;">
            <span class="card-title">三步门禁流水线 (3-Step Gate)</span>
            <span class="badge badge-info" id="percepStepGateBadge">门禁严格保护</span>
          </div>
          <div style="font-size:12px; color:var(--text-muted); margin-bottom:12px;">
            规则：① 开启头部深度 &amp; 启动 YOLO → ② 锁存最佳分割（带代次校验） → ③ 启动 FoundationPose 6D。注册完成前禁止停止 YOLO。
          </div>

          <!-- Step 1 Card -->
          <div style="background:var(--bg-subtle); border-radius:var(--radius-sm); padding:12px; margin-bottom:10px; border-left:3px solid var(--accent);">
            <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:6px;">
              <strong style="font-size:13px;">第 ① 步：YOLO 目标检测与分割</strong>
              <span class="badge badge-no_result" id="step1StatusBadge">未启动</span>
            </div>
            <div style="display:flex; gap:6px; margin-bottom:8px;">
              <input type="text" id="yoloPromptInput" class="form-input" value="bottle" placeholder="目标类别 (如 bottle/cup)" style="padding:4px 8px; font-size:12px; font-family:var(--font-mono); flex:1;">
              <input type="number" id="yoloConfInput" class="form-input" value="0.4" min="0.1" max="0.95" step="0.05" title="置信度阈值" style="padding:4px 6px; font-size:12px; font-family:var(--font-mono); width:72px;">
            </div>
            <div style="display:flex; gap:8px;">
              <button class="btn btn-sm btn-primary" id="btnStartYolo" style="flex:1;">启动 YOLO 检测</button>
              <button class="btn btn-sm" id="btnStopYolo" style="flex:1;">停止 YOLO</button>
            </div>
            <div id="yoloResultBox" style="margin-top:8px; font-size:11px; font-family:var(--font-mono); color:#334155; display:none;"></div>
          </div>

          <!-- Step 2 Card -->
          <div style="background:var(--bg-subtle); border-radius:var(--radius-sm); padding:12px; margin-bottom:10px; border-left:3px solid var(--info);">
            <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:6px;">
              <strong style="font-size:13px;">第 ② 步：锁存最佳目标分割 (Latch)</strong>
              <span class="badge badge-no_result" id="step2StatusBadge">未锁存</span>
            </div>
            <div style="font-size:11px; color:var(--text-muted); margin-bottom:8px;">
              锁存当前最高置信度 mask 并绑定 generation 代次标签；重启 YOLO 将使旧锁存自动失效。
            </div>
            <button class="btn btn-sm btn-primary" id="btnLatchSegment" style="width:100%;">锁存最佳分割目标</button>
            <div id="latchResultBox" style="margin-top:8px; font-size:11px; font-family:var(--font-mono); color:#334155; display:none;">
              已锁存: <span id="latchedGenTag" style="color:var(--accent); font-weight:600;">--</span>
            </div>
          </div>

          <!-- Step 3 Card -->
          <div style="background:var(--bg-subtle); border-radius:var(--radius-sm); padding:12px; border-left:3px solid var(--success);">
            <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:6px;">
              <strong style="font-size:13px;">第 ③ 步：FoundationPose 6D 姿态估计</strong>
              <span class="badge badge-no_result" id="step3StatusBadge">等待锁存</span>
            </div>
            <div style="font-size:11px; color:var(--text-muted); margin-bottom:8px;">
              基于锁存的目标区域与深度反算物体在 base_link 下的完整 6D 位姿。
            </div>
            <div style="display:flex; gap:8px;">
              <button class="btn btn-sm btn-primary" id="btnStart6D" style="flex:1;">启动 6D 姿态解算</button>
              <button class="btn btn-sm" id="btnStop6D" style="flex:1;">停止 6D 解算</button>
            </div>
            <div id="sixDResultBox" style="margin-top:8px; font-size:11px; font-family:var(--font-mono); background:#ffffff; padding:8px; border-radius:4px; border:1px solid var(--border); display:none;"></div>
          </div>
        </div>

        <!-- ArUco Detection & Discrete Chassis Fine Adjustment -->
        <div class="card" style="margin-bottom:0;">
          <div class="card-header" style="margin-bottom:10px;">
            <span class="card-title">ArUco 标定与离散微调对齐</span>
            <span class="badge badge-no_result" id="arucoDetectedBadge">未检测</span>
          </div>
          <div style="font-size:12px; color:var(--text-muted); margin-bottom:10px;">
            先启动 ArUco 位姿识别，再开启底盘离散微调将目标中心锁定在相机光轴 ±5mm 内。
          </div>
          <div style="display:flex; gap:8px; margin-bottom:10px;">
            <button class="btn btn-sm btn-primary" id="btnStartAruco" style="flex:1;">启动 ArUco 位姿</button>
            <button class="btn btn-sm" id="btnStopAruco" style="flex:1;">停止 ArUco</button>
          </div>
          <div style="background:var(--bg-subtle); padding:8px 12px; border-radius:var(--radius-sm); font-size:11px; font-family:var(--font-mono); margin-bottom:10px; color:var(--text-muted);" id="arucoPoseDisplay">
            尚未启动 ArUco 位姿检测。
          </div>
          <div style="display:flex; justify-content:space-between; align-items:center;">
            <span style="font-size:12px; font-weight:500;">闭环离散对齐微调</span>
            <button class="btn btn-sm btn-primary" id="btnArucoFineAdjust">开启 5mm 居中微调</button>
          </div>
        </div>
      </div>
    </div>
  `;

  function refreshVisibleFrames() {
    if (!container.classList.contains("active") || document.hidden) return;
    if (!currentCameras.length) return;
    imgTick++;
    const robotId = api.currentRobotId;
    container.querySelectorAll(".camera-slot").forEach((slotEl) => {
      if (slotEl.style.display === "none") return;
      const idx = Number(slotEl.dataset.slot);
      const streamKey = slotStreams[idx];
      if (!streamKey) return;
      const img = slotEl.querySelector(".cam-slot-img");
      if (img) {
        img.src = `/api/robots/${encodeURIComponent(robotId)}/camera/${encodeURIComponent(streamKey)}?_t=${imgTick}`;
      }
    });
  }

  function configureCameraSlots(cameras) {
    currentCameras = cameras || [];
    const slotEls = container.querySelectorAll(".camera-slot");
    slotEls.forEach((slotEl, idx) => {
      const sel = slotEl.querySelector(".cam-slot-select");
      const meta = slotEl.querySelector(".cam-slot-meta");
      if (!currentCameras.length) {
        slotEl.style.display = "none";
        return;
      }
      if (idx >= currentCameras.length) {
        slotEl.style.display = "none";
        slotStreams[idx] = null;
        return;
      }
      slotEl.style.display = "flex";
      const chosen = currentCameras[idx].key;
      slotStreams[idx] = chosen;
      sel.innerHTML = currentCameras.map(c =>
        `<option value="${escapeHtml(c.key)}" ${c.key === chosen ? "selected" : ""}>${escapeHtml(c.label)} (${escapeHtml(c.key)})</option>`
      ).join("");
      const camObj = currentCameras[idx];
      meta.textContent = `${camObj.resolution} · ${camObj.fps_target}fps${camObj.has_depth ? " · D" : ""}`;
    });
    refreshVisibleFrames();
  }

  // Slot selector change
  container.querySelectorAll(".cam-slot-select").forEach(sel => {
    sel.addEventListener("change", (e) => {
      const idx = Number(sel.dataset.slot);
      const nextKey = e.target.value;
      slotStreams[idx] = nextKey;
      const camObj = currentCameras.find(c => c.key === nextKey);
      const meta = container.querySelector(`.cam-slot-meta[data-slot="${idx}"]`);
      if (meta && camObj) {
        meta.textContent = `${camObj.resolution} · ${camObj.fps_target}fps${camObj.has_depth ? " · D" : ""}`;
      }
      refreshVisibleFrames();
    });
  });

  // Click any camera frame -> 3D Coordinate Ray Backprojection (aligned to 480x270 SVG frame)
  const pointText = container.querySelector("#percepCalculatedPointText");
  container.querySelectorAll(".camera-frame-wrap").forEach(wrap => {
    wrap.addEventListener("click", async (e) => {
      const idx = Number(wrap.dataset.slot);
      const streamKey = slotStreams[idx];
      if (!streamKey) return;
      const rect = wrap.getBoundingClientRect();
      if (!rect.width || !rect.height) return;
      const u = Math.max(0, Math.min(480, Math.round(((e.clientX - rect.left) / rect.width) * 480)));
      const v = Math.max(0, Math.min(270, Math.round(((e.clientY - rect.top) / rect.height) * 270)));

      const crosshair = wrap.querySelector(".cam-slot-crosshair");
      if (crosshair) {
        crosshair.style.display = "block";
        crosshair.style.left = `${e.clientX - rect.left}px`;
        crosshair.style.top = `${e.clientY - rect.top}px`;
      }

      try {
        const res = await api.controlPerception("pick_3d", { u, v, stream_key: streamKey });
        const pick = res.pick || {};
        if (pick.base_link_xyz_m) {
          const [x, y, z] = pick.base_link_xyz_m;
          clicked3DPoint = { x, y, z, stream_key: streamKey, depth_m: pick.depth_m };
          pointText.innerHTML = `${escapeHtml(streamKey)} (u=${u}, v=${v}, depth=${Number(pick.depth_m || 0).toFixed(3)}m) &rarr; <span style="color:#0f172a; font-weight:700;">[X: ${x.toFixed(3)}m, Y: ${y.toFixed(3)}m, Z: ${z.toFixed(3)}m]</span>`;
          refreshVisibleFrames();
        }
      } catch (err) {
        notify(`3D 拾取被门禁拦截: ${err.message}`, "error");
      }
    });
  });

  container.querySelector("#btnCopy3DPoint").addEventListener("click", async () => {
    if (!clicked3DPoint) {
      notify("请先点击上方画面拾取 3D 点", "info");
      return;
    }
    const text = JSON.stringify({
      frame: "base_link",
      stream: clicked3DPoint.stream_key,
      x: Number(clicked3DPoint.x.toFixed(4)),
      y: Number(clicked3DPoint.y.toFixed(4)),
      z: Number(clicked3DPoint.z.toFixed(4)),
    });
    try {
      await navigator.clipboard.writeText(text);
      notify(`已复制 3D 坐标: ${text}`, "success");
    } catch (err) {
      notify(`复制失败: ${err.message}`, "error");
    }
  });

  // Camera layout switcher (4-grid vs single primary stream)
  const btnAll = container.querySelector("#btnCamViewAll");
  const btnHead = container.querySelector("#btnCamViewHead");
  const camGrid = container.querySelector("#cameraStreamsContainer");
  btnAll.addEventListener("click", () => {
    camGrid.style.gridTemplateColumns = "1fr 1fr";
    container.querySelectorAll(".camera-slot").forEach((el, i) => {
      el.style.display = i < currentCameras.length ? "flex" : "none";
    });
    btnAll.style.background = "var(--accent-light)";
    btnAll.style.color = "var(--accent)";
    btnHead.style.background = "";
    btnHead.style.color = "";
    refreshVisibleFrames();
  });
  btnHead.addEventListener("click", () => {
    camGrid.style.gridTemplateColumns = "1fr";
    container.querySelectorAll(".camera-slot").forEach((el, i) => {
      el.style.display = i === 0 && currentCameras.length > 0 ? "flex" : "none";
    });
    btnHead.style.background = "var(--accent-light)";
    btnHead.style.color = "var(--accent)";
    btnAll.style.background = "";
    btnAll.style.color = "";
    refreshVisibleFrames();
  });

  // Head Depth Gate Toggle
  container.querySelector("#percepToggleDepthBtn").addEventListener("click", async () => {
    try {
      const next = !depthEnabled;
      const res = await api.controlPerception("set_head_depth", { enabled: next });
      syncPerceptionState(res.perception);
      refreshVisibleFrames();
      notify(next ? "头部在线深度处理已开启 (READY)" : "头部在线深度处理已关闭", "info");
    } catch (err) {
      notify(`切换深度门禁失败: ${err.message}`, "error");
    }
  });

  // Step 1: Start / Stop YOLO
  container.querySelector("#btnStartYolo").addEventListener("click", async () => {
    const prompt = container.querySelector("#yoloPromptInput").value.trim() || "bottle";
    const conf = parseFloat(container.querySelector("#yoloConfInput").value) || 0.4;
    const camera = slotStreams[0] || "head_left_rgb";
    try {
      const res = await api.controlPerception("start_yolo", { camera, prompt, conf });
      syncPerceptionState(res.perception);
      refreshVisibleFrames();
      notify(`步骤① YOLO 已启动 (prompt=${prompt}, gen=${res.perception?.yolo_generation})`, "success");
    } catch (err) {
      notify(`启动 YOLO 被拦截: ${err.message}`, "error");
    }
  });

  container.querySelector("#btnStopYolo").addEventListener("click", async () => {
    try {
      const res = await api.controlPerception("stop_yolo");
      syncPerceptionState(res.perception);
      refreshVisibleFrames();
      notify("YOLO 检测已停止", "info");
    } catch (err) {
      notify(`停止 YOLO 被门禁拦截: ${err.message}`, "error");
    }
  });

  // Step 2: Latch Best Segment
  container.querySelector("#btnLatchSegment").addEventListener("click", async () => {
    try {
      const res = await api.controlPerception("latch_yolo", {});
      syncPerceptionState(res.perception);
      refreshVisibleFrames();
      notify(`步骤② 已锁存最佳分割 (gen=${res.perception?.latched_mask?.generation})`, "success");
    } catch (err) {
      notify(`三步门禁拦截: ${err.message}`, "error");
    }
  });

  // Step 3: Start / Stop FoundationPose 6D
  container.querySelector("#btnStart6D").addEventListener("click", async () => {
    const camera = latchedMask?.camera || slotStreams[0] || "head_left_rgb";
    const classes = latchedMask?.class_name || container.querySelector("#yoloPromptInput").value.trim() || "bottle";
    try {
      const res = await api.controlPerception("start_foundation_pose", { camera, classes });
      syncPerceptionState(res.perception);
      refreshVisibleFrames();
      notify("步骤③ FoundationPose 6D 已启动（注册完成前锁定停止 YOLO）", "success");
    } catch (err) {
      notify(`三步门禁拦截: ${err.message}`, "error");
    }
  });

  container.querySelector("#btnStop6D").addEventListener("click", async () => {
    try {
      const res = await api.controlPerception("stop_foundation_pose");
      syncPerceptionState(res.perception);
      refreshVisibleFrames();
      notify("FoundationPose 6D 已停止", "info");
    } catch (err) {
      notify(`停止 6D 解算失败: ${err.message}`, "error");
    }
  });

  // Reset Pipeline
  container.querySelector("#percepResetPipelineBtn").addEventListener("click", async () => {
    try {
      await api.controlPerception("stop_foundation_pose").catch(() => {});
      await api.controlPerception("stop_yolo").catch(() => {});
      await api.controlPerception("aruco_stop_pose").catch(() => {});
      const state = await api.getState();
      syncPerceptionState(state.observation?.perception);
      refreshVisibleFrames();
      notify("已重置感知流水线状态", "info");
    } catch (err) {
      notify(`重置感知流水线失败: ${err.message}`, "error");
    }
  });

  // ArUco Controls
  container.querySelector("#btnStartAruco").addEventListener("click", async () => {
    try {
      const res = await api.controlPerception("aruco_start_pose", {});
      syncPerceptionState(res.perception);
      refreshVisibleFrames();
      notify("ArUco 位姿检测已启动", "success");
    } catch (err) {
      notify(`启动 ArUco 失败: ${err.message}`, "error");
    }
  });

  container.querySelector("#btnStopAruco").addEventListener("click", async () => {
    try {
      const res = await api.controlPerception("aruco_stop_pose", {});
      syncPerceptionState(res.perception);
      refreshVisibleFrames();
      notify("ArUco 位姿检测已停止", "info");
    } catch (err) {
      notify(`停止 ArUco 失败: ${err.message}`, "error");
    }
  });

  container.querySelector("#btnArucoFineAdjust").addEventListener("click", async () => {
    try {
      const next = !arucoFineEnabled;
      const res = await api.controlPerception("aruco_set_fine", { enabled: next });
      syncPerceptionState(res.perception);
      refreshVisibleFrames();
      notify(next ? "已开启 ArUco 底盘离散居中微调 (±5mm)" : "已关闭 ArUco 离散微调", "success");
    } catch (err) {
      notify(`微调对齐失败: ${err.message}`, "error");
    }
  });

  function syncPerceptionState(p) {
    if (!p) return;
    depthEnabled = !!p.head_depth_enabled;
    depthReady = !!p.head_depth_ready;
    yoloRunning = !!p.yolo_running;
    latchedMask = p.latched_mask || null;
    sixDRunning = !!p.foundation_pose_running;
    sixDPhase = p.foundation_pose_phase || "idle";
    arucoRunning = !!p.aruco_pose_running;
    arucoFineEnabled = !!p.aruco_fine_enabled;

    // Head Depth Badge & Toggle
    const depthBadge = container.querySelector("#percepDepthGateBadge");
    const depthBtn = container.querySelector("#percepToggleDepthBtn");
    if (depthBadge) {
      depthBadge.textContent = depthReady
        ? `头部深度: READY (${Number(p.head_depth_fps || 0).toFixed(1)} FPS)`
        : depthEnabled ? "头部深度: 启动中" : "头部深度: OFFLINE";
      depthBadge.className = `badge ${depthReady ? "badge-passed" : "badge-no_result"}`;
    }
    if (depthBtn) {
      depthBtn.textContent = depthEnabled ? "关闭头部深度相机" : "启动头部深度相机";
    }

    // Step 1: YOLO
    const s1Badge = container.querySelector("#step1StatusBadge");
    const yoloBox = container.querySelector("#yoloResultBox");
    const stopYoloBtn = container.querySelector("#btnStopYolo");
    const yoloLocked = sixDRunning && (sixDPhase === "waiting" || sixDPhase === "registering");
    if (stopYoloBtn) {
      stopYoloBtn.disabled = yoloLocked;
      stopYoloBtn.textContent = yoloLocked ? "停止 YOLO (注册锁定中)" : "停止 YOLO";
      stopYoloBtn.title = yoloLocked ? "FoundationPose 注册完成前禁止停止 YOLO" : "停止 YOLO 检测";
    }
    if (yoloRunning) {
      s1Badge.textContent = `运行中 · gen ${p.yolo_generation || 0}`;
      s1Badge.className = "badge badge-running";
      const dets = p.yolo_detections || [];
      yoloBox.style.display = "block";
      yoloBox.textContent = dets.length
        ? dets.map(d => `${d.class_name} (conf=${d.confidence}, area=${d.area_px}px, gen=${d.generation})`).join(" | ")
        : "运行中，等待检测结果...";
    } else {
      s1Badge.textContent = "未启动";
      s1Badge.className = "badge badge-no_result";
      yoloBox.style.display = "none";
    }

    // Step 2: Latch
    const s2Badge = container.querySelector("#step2StatusBadge");
    const latchBox = container.querySelector("#latchResultBox");
    if (latchedMask) {
      s2Badge.textContent = `已锁存 · gen ${latchedMask.generation}`;
      s2Badge.className = "badge badge-passed";
      latchBox.style.display = "block";
      container.querySelector("#latchedGenTag").textContent =
        `gen_${latchedMask.generation} (${latchedMask.class_name}, ${latchedMask.camera}, area=${latchedMask.area_px}px)`;
    } else {
      s2Badge.textContent = "未锁存";
      s2Badge.className = "badge badge-no_result";
      latchBox.style.display = "none";
    }

    // Step 3: FoundationPose 6D
    const s3Badge = container.querySelector("#step3StatusBadge");
    const sixDBox = container.querySelector("#sixDResultBox");
    const pBadge = container.querySelector("#percepPipelineBadge");
    if (sixDRunning) {
      s3Badge.textContent = `6D: ${sixDPhase.toUpperCase()}`;
      s3Badge.className = sixDPhase === "tracking" ? "badge badge-passed" : "badge badge-running";
      sixDBox.style.display = "block";
      const res = p.foundation_pose_result;
      if (res) {
        const [x, y, z] = Array.isArray(res.xyz_m) ? res.xyz_m : [res.x_m, res.y_m, res.z_m];
        const [rDeg, pDeg, yDeg] = Array.isArray(res.rpy_deg) ? res.rpy_deg : [res.roll_deg, res.pitch_deg, res.yaw_deg];
        const objMeta = res.object ? ` · ${escapeHtml(res.object)}${res.confidence ? ` (${(Number(res.confidence) * 100).toFixed(0)}%)` : ""}` : "";
        sixDBox.innerHTML = `
          <strong>6D Pose (${escapeHtml(res.frame || "base_link")} · ${escapeHtml(sixDPhase)}${objMeta}):</strong><br>
          X: ${Number(x).toFixed(3)}m | Y: ${Number(y).toFixed(3)}m | Z: ${Number(z).toFixed(3)}m<br>
          Roll: ${Number(rDeg).toFixed(1)}° | Pitch: ${Number(pDeg).toFixed(1)}° | Yaw: ${Number(yDeg).toFixed(1)}°
        `;
      } else {
        sixDBox.textContent = `阶段: ${sixDPhase} — 正在配准网格模型 (${p.foundation_pose_mesh || "--"})...`;
      }
      pBadge.textContent = `流水线: 6D ${sixDPhase.toUpperCase()}`;
      pBadge.className = "badge badge-running";
    } else if (latchedMask) {
      s3Badge.textContent = "就绪 (已锁存)";
      s3Badge.className = "badge badge-info";
      sixDBox.style.display = "none";
      pBadge.textContent = "流水线: 目标已锁存";
      pBadge.className = "badge badge-info";
    } else if (yoloRunning) {
      s3Badge.textContent = "等待锁存";
      s3Badge.className = "badge badge-no_result";
      sixDBox.style.display = "none";
      pBadge.textContent = "流水线: YOLO 运行中";
      pBadge.className = "badge badge-running";
    } else {
      s3Badge.textContent = "未就绪";
      s3Badge.className = "badge badge-no_result";
      sixDBox.style.display = "none";
      pBadge.textContent = "流水线: IDLE";
      pBadge.className = "badge badge-no_result";
    }

    // ArUco state
    const arucoTopBadge = container.querySelector("#percepArucoBadge");
    const arucoCardBadge = container.querySelector("#arucoDetectedBadge");
    const arucoPoseEl = container.querySelector("#arucoPoseDisplay");
    const arucoFineBtn = container.querySelector("#btnArucoFineAdjust");
    const markers = p.aruco_markers || [];
    if (arucoRunning) {
      const m = markers[0];
      arucoTopBadge.textContent = arucoFineEnabled ? "ArUco: 居中微调已对齐" : `ArUco: 追踪中 (${markers.length})`;
      arucoTopBadge.className = "badge badge-passed";
      arucoCardBadge.textContent = m ? `Marker ID: ${m.id}` : "搜索中...";
      arucoCardBadge.className = m ? "badge badge-passed" : "badge badge-info";
      arucoPoseEl.textContent = m
        ? `Tag #${m.id} (${m.frame}): X=${Number(m.x_m).toFixed(3)}m, Y=${Number(m.y_m).toFixed(3)}m, Z=${Number(m.z_m).toFixed(3)}m (Yaw=${Number(m.yaw_deg).toFixed(1)}°)`
        : "已启动 ArUco，等待画面中出现标记...";
      if (arucoFineBtn) arucoFineBtn.textContent = arucoFineEnabled ? "关闭居中微调" : "开启 5mm 居中微调";
    } else {
      arucoTopBadge.textContent = "ArUco: 未启动";
      arucoTopBadge.className = "badge badge-no_result";
      arucoCardBadge.textContent = "未启动";
      arucoCardBadge.className = "badge badge-no_result";
      arucoPoseEl.textContent = "尚未启动 ArUco 位姿检测。";
      if (arucoFineBtn) arucoFineBtn.textContent = "开启 5mm 居中微调";
    }

    // Real perception logs
    const logsPre = container.querySelector("#percepLogsPre");
    if (logsPre) {
      const logs = p.logs || [];
      logsPre.textContent = logs.length ? logs.join("\n") : "等待感知管线事件...";
      logsPre.scrollTop = logsPre.scrollHeight;
    }
  }

  // Only poll mock camera frames while Perception tab is active & visible (prevents background polling leak!)
  refreshTimer = setInterval(refreshVisibleFrames, 1200);

  return {
    destroy() {
      if (refreshTimer) clearInterval(refreshTimer);
    },
    update(state) {
      if (!state) return;
      const desc = state.descriptor || {};
      const obs = state.observation || {};
      const caps = desc.capabilities || {};
      const cams = desc.cameras || [];

      if (currentRobotId !== desc.id) {
        currentRobotId = desc.id;
        clicked3DPoint = null;
        container.querySelectorAll(".cam-slot-crosshair").forEach(el => { el.style.display = "none"; });
        configureCameraSlots(cams);
      } else if (container.classList.contains("active") && imgTick === 0 && cams.length) {
        refreshVisibleFrames();
      }

      if (!cams || cams.length === 0) {
        camGrid.style.opacity = "0.3";
      } else {
        camGrid.style.opacity = "1.0";
      }

      if (obs.perception) {
        syncPerceptionState(obs.perception);
      }

      // Camera viewing does not imply unsupported depth/YOLO/6D/ArUco operations.
      const capabilityButtons = {
        percepToggleDepthBtn: caps.head_depth,
        btnStartYolo: caps.yolo_foundation_pose,
        btnLatchSegment: caps.yolo_foundation_pose,
        btnStart6D: caps.yolo_foundation_pose,
        btnStartAruco: caps.aruco,
        btnArucoFineAdjust: caps.aruco,
      };
      for (const [id, enabled] of Object.entries(capabilityButtons)) {
        const button = container.querySelector("#" + id);
        if (!button) continue;
        button.disabled = !enabled;
        button.title = enabled ? "需持有控制 Lease 后下发感知流水线指令" : "当前机型未提供此项能力；相机查看仍可用";
      }
    },
  };
}
