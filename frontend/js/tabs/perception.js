// Tab 4: Visual Perception Workbench
import { api } from "../api.js";

export function initPerceptionTab(container) {
  let activeCameraSlot = "head_rgb";
  let refreshTimer = null;
  let clicked3DPoint = null;

  container.innerHTML = `
    <!-- Top Status Bar & Quick Actions -->
    <div class="card" style="margin-bottom:16px;">
      <div style="display:flex; justify-content:space-between; align-items:center; flex-wrap:wrap; gap:12px;">
        <div style="display:flex; align-items:center; gap:12px;">
          <span class="card-title" style="margin:0;">视觉感知工作台 (Visual Perception)</span>
          <span class="badge badge-idle" id="percepPipelineBadge">流水线: IDLE</span>
          <span class="badge badge-info" id="percepDepthGateBadge">头部深度: OFFLINE</span>
          <span class="badge badge-idle" id="percepArucoBadge">ArUco: 未检测</span>
        </div>
        <div style="display:flex; gap:8px;">
          <button class="btn btn-sm" id="percepToggleDepthBtn">启动头部深度相机</button>
          <button class="btn btn-sm btn-danger" id="percepResetPipelineBtn">重置感知流水线</button>
        </div>
      </div>
    </div>

    <!-- Main Grid: Left Camera Streams Grid + Right 3-Step Perception Pipeline -->
    <div class="grid-2" style="grid-template-columns: 1fr 420px; align-items:start;">
      <!-- Left: Camera Streams (4 Slots) -->
      <div class="card" style="margin-bottom:0;">
        <div class="card-header" style="margin-bottom:12px;">
          <div style="display:flex; align-items:center; gap:10px;">
            <span class="card-title">多相机实时视流 (4-Stream Grid)</span>
            <span style="font-size:12px; color:var(--text-muted);">点击画面反算 base_link 3D 坐标</span>
          </div>
          <div style="display:flex; gap:6px;">
            <button class="btn btn-sm active" data-cam-filter="all" id="btnCamViewAll">4画面平铺</button>
            <button class="btn btn-sm" data-cam-filter="head_rgb" id="btnCamViewHead">主视角</button>
          </div>
        </div>

        <!-- 4 Camera Cards Grid -->
        <div id="cameraStreamsContainer" style="display:grid; grid-template-columns: 1fr 1fr; gap:12px;">
          <!-- Slot 1: Head RGB -->
          <div class="cam-box" style="background:#0f172a; border-radius:var(--radius-sm); overflow:hidden; border:1px solid var(--border); position:relative;">
            <div style="padding:6px 10px; background:rgba(15,23,42,0.85); display:flex; justify-content:space-between; align-items:center; font-size:12px; color:#cbd5e1;">
              <strong>头部 RGB (head_rgb)</strong>
              <span class="badge badge-running" style="font-size:10px; padding:2px 6px;">1280x720 · 30fps</span>
            </div>
            <div style="position:relative; width:100%; aspect-ratio:16/9; cursor:crosshair;" id="camWrapperHeadRgb">
              <img id="camImgHeadRgb" src="/api/robots/r1pro-mock/camera/head_rgb" style="width:100%; height:100%; object-fit:contain; display:block;" alt="head_rgb">
              <div id="crosshairHeadRgb" style="display:none; position:absolute; width:16px; height:16px; border:2px solid #ef4444; border-radius:50%; transform:translate(-50%, -50%); pointer-events:none;"></div>
            </div>
          </div>

          <!-- Slot 2: Head Depth -->
          <div class="cam-box" style="background:#0f172a; border-radius:var(--radius-sm); overflow:hidden; border:1px solid var(--border); position:relative;">
            <div style="padding:6px 10px; background:rgba(15,23,42,0.85); display:flex; justify-content:space-between; align-items:center; font-size:12px; color:#cbd5e1;">
              <strong>头部深度伪彩 (head_depth)</strong>
              <span class="badge badge-info" id="camDepthStatusBadge" style="font-size:10px; padding:2px 6px;">READY · D435</span>
            </div>
            <div style="position:relative; width:100%; aspect-ratio:16/9; cursor:crosshair;" id="camWrapperHeadDepth">
              <img id="camImgHeadDepth" src="/api/robots/r1pro-mock/camera/head_depth" style="width:100%; height:100%; object-fit:contain; display:block;" alt="head_depth">
            </div>
          </div>

          <!-- Slot 3: Left Wrist -->
          <div class="cam-box" style="background:#0f172a; border-radius:var(--radius-sm); overflow:hidden; border:1px solid var(--border); position:relative;">
            <div style="padding:6px 10px; background:rgba(15,23,42,0.85); display:flex; justify-content:space-between; align-items:center; font-size:12px; color:#cbd5e1;">
              <strong>左手腕部 (wrist_left)</strong>
              <span class="badge badge-running" style="font-size:10px; padding:2px 6px;">640x480 · 25fps</span>
            </div>
            <div style="position:relative; width:100%; aspect-ratio:16/9;" id="camWrapperWristLeft">
              <img id="camImgWristLeft" src="/api/robots/r1pro-mock/camera/wrist_left" style="width:100%; height:100%; object-fit:contain; display:block;" alt="wrist_left">
            </div>
          </div>

          <!-- Slot 4: Right Wrist -->
          <div class="cam-box" style="background:#0f172a; border-radius:var(--radius-sm); overflow:hidden; border:1px solid var(--border); position:relative;">
            <div style="padding:6px 10px; background:rgba(15,23,42,0.85); display:flex; justify-content:space-between; align-items:center; font-size:12px; color:#cbd5e1;">
              <strong>右手腕部 (wrist_right)</strong>
              <span class="badge badge-running" style="font-size:10px; padding:2px 6px;">640x480 · 25fps</span>
            </div>
            <div style="position:relative; width:100%; aspect-ratio:16/9;" id="camWrapperWristRight">
              <img id="camImgWristRight" src="/api/robots/r1pro-mock/camera/wrist_right" style="width:100%; height:100%; object-fit:contain; display:block;" alt="wrist_right">
            </div>
          </div>
        </div>

        <!-- 3D Ray Backprojection Point Result Display -->
        <div style="margin-top:12px; background:var(--bg-subtle); padding:10px 14px; border-radius:var(--radius-sm); display:flex; align-items:center; justify-content:space-between;">
          <div>
            <div style="font-size:11px; color:var(--text-muted);">像素点击反算 3D 点 (Camera Ray & Depth → base_link):</div>
            <div style="font-family:var(--font-mono); font-size:13px; font-weight:600; color:var(--accent);" id="percepCalculatedPointText">
              尚未点击画面。点击上方画面任意像素可拾取 metric 3D 坐标。
            </div>
          </div>
          <button class="btn btn-sm" id="btnCopy3DPoint">复制 3D 坐标</button>
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
            规则：① 启动 YOLO → ② 锁存最佳分割（校验代次） → ③ 启动 FoundationPose 6D。未锁存禁止启 6D；运行中禁止随意停 YOLO。
          </div>

          <!-- Step 1 Card -->
          <div style="background:var(--bg-subtle); border-radius:var(--radius-sm); padding:12px; margin-bottom:10px; border-left:3px solid var(--accent);">
            <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:6px;">
              <strong style="font-size:13px;">第 ① 步：YOLO 目标检测与分割</strong>
              <span class="badge badge-idle" id="step1StatusBadge">未启动</span>
            </div>
            <div style="font-size:11px; color:var(--text-muted); margin-bottom:8px;">
              检测目标类别: <code style="font-size:11px;">cup, box, plate, bowl</code>
            </div>
            <div style="display:flex; gap:8px;">
              <button class="btn btn-sm btn-primary" id="btnStartYolo" style="flex:1;">启动 YOLO 检测</button>
              <button class="btn btn-sm" id="btnStopYolo" style="flex:1;">停止 YOLO</button>
            </div>
            <div id="yoloResultBox" style="margin-top:8px; font-size:11px; font-family:var(--font-mono); color:#475569; display:none;">
              检测到 2 个目标: [cup (0.92, bbox:[320,180,410,290])], [plate (0.88)]
            </div>
          </div>

          <!-- Step 2 Card -->
          <div style="background:var(--bg-subtle); border-radius:var(--radius-sm); padding:12px; margin-bottom:10px; border-left:3px solid var(--info);">
            <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:6px;">
              <strong style="font-size:13px;">第 ② 步：锁存最佳目标分割 (Latch)</strong>
              <span class="badge badge-idle" id="step2StatusBadge">未锁存</span>
            </div>
            <div style="font-size:11px; color:var(--text-muted); margin-bottom:8px;">
              锁存当前最高置信度 mask 并绑定 generation 代次标签。
            </div>
            <button class="btn btn-sm btn-primary" id="btnLatchSegment" style="width:100%;">锁存最佳分割目标</button>
            <div id="latchResultBox" style="margin-top:8px; font-size:11px; font-family:var(--font-mono); color:#475569; display:none;">
              已锁存: <span id="latchedGenTag" style="color:var(--accent); font-weight:600;">gen_492 (cup)</span>
            </div>
          </div>

          <!-- Step 3 Card -->
          <div style="background:var(--bg-subtle); border-radius:var(--radius-sm); padding:12px; border-left:3px solid var(--success);">
            <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:6px;">
              <strong style="font-size:13px;">第 ③ 步：FoundationPose 6D 姿态估计</strong>
              <span class="badge badge-idle" id="step3StatusBadge">就绪等待锁存</span>
            </div>
            <div style="font-size:11px; color:var(--text-muted); margin-bottom:8px;">
              基于锁存的目标区域与深度反算物体在 base_link 下的完整 6D 位姿。
            </div>
            <div style="display:flex; gap:8px;">
              <button class="btn btn-sm btn-primary" id="btnStart6D" style="flex:1;">启动 6D 姿态解算</button>
              <button class="btn btn-sm" id="btnStop6D" style="flex:1;">停止 6D 解算</button>
            </div>
            <div id="sixDResultBox" style="margin-top:8px; font-size:11px; font-family:var(--font-mono); background:#ffffff; padding:6px 8px; border-radius:4px; border:1px solid var(--border); display:none;">
              <strong>6D Pose (base_link):</strong><br>
              X: 0.582m | Y: -0.114m | Z: 0.812m<br>
              Roll: 2.1° | Pitch: -1.4° | Yaw: 48.6°
            </div>
          </div>
        </div>

        <!-- ArUco Detection & Discrete Chassis Fine Adjustment -->
        <div class="card" style="margin-bottom:0;">
          <div class="card-header" style="margin-bottom:10px;">
            <span class="card-title">ArUco 标定与离散微调对齐</span>
            <span class="badge badge-passed" id="arucoDetectedBadge">Marker ID: 4</span>
          </div>
          <div style="font-size:12px; color:var(--text-muted); margin-bottom:10px;">
            识别货架/桌面二维码标签，通过底盘离散微调将目标中心锁定在相机光轴 ±5mm 内。
          </div>
          <div style="background:var(--bg-subtle); padding:8px 12px; border-radius:var(--radius-sm); font-size:11px; font-family:var(--font-mono); margin-bottom:10px;" id="arucoPoseDisplay">
            Tag Pose: X=0.840m, Y=0.035m, Z=0.920m (Yaw=4.2°)
          </div>
          <div style="display:flex; justify-content:space-between; align-items:center;">
            <span style="font-size:12px; font-weight:500;">闭环离散对齐微调</span>
            <button class="btn btn-sm btn-primary" id="btnArucoFineAdjust">执行 5mm 居中微调</button>
          </div>
        </div>
      </div>
    </div>
  `;

  // Image Click 3D Coordinate Ray Backprojection
  const headRgbWrapper = container.querySelector("#camWrapperHeadRgb");
  const crosshair = container.querySelector("#crosshairHeadRgb");
  const pointText = container.querySelector("#percepCalculatedPointText");

  headRgbWrapper.addEventListener("click", async (e) => {
    const rect = headRgbWrapper.getBoundingClientRect();
    const u = Math.round(((e.clientX - rect.left) / rect.width) * 1280);
    const v = Math.round(((e.clientY - rect.top) / rect.height) * 720);

    // Show crosshair
    crosshair.style.display = "block";
    crosshair.style.left = `${e.clientX - rect.left}px`;
    crosshair.style.top = `${e.clientY - rect.top}px`;

    try {
      const res = await api.controlPerception("pick_3d", { u, v, stream_key: "head_left_rgb" });
      const pick = res.pick || {};
      const pt = pick.base_link_xyz_m ? { x: pick.base_link_xyz_m[0], y: pick.base_link_xyz_m[1], z: pick.base_link_xyz_m[2] } : { x: (u / 1280) * 0.8 + 0.2, y: (v / 720 - 0.5) * 0.6, z: 0.82 };
      clicked3DPoint = pt;
      pointText.innerHTML = `u=${u}, v=${v} &rarr; <span style="color:#0f172a; font-weight:bold;">[X: ${pt.x.toFixed(3)}m, Y: ${pt.y.toFixed(3)}m, Z: ${pt.z.toFixed(3)}m]</span>`;
    } catch (err) {
      console.warn("Pick point failed:", err);
    }
  });

  container.querySelector("#btnCopy3DPoint").addEventListener("click", () => {
    if (!clicked3DPoint) {
      alert("请先点击上方画面拾取 3D 点");
      return;
    }
    const text = `{"x": ${clicked3DPoint.x.toFixed(3)}, "y": ${clicked3DPoint.y.toFixed(3)}, "z": ${clicked3DPoint.z.toFixed(3)}}`;
    navigator.clipboard.writeText(text);
    alert(`已复制 3D 坐标: ${text}`);
  });

  // Camera views switch
  container.querySelector("#btnCamViewAll").addEventListener("click", () => {
    container.querySelector("#cameraStreamsContainer").style.gridTemplateColumns = "1fr 1fr";
  });
  container.querySelector("#btnCamViewHead").addEventListener("click", () => {
    container.querySelector("#cameraStreamsContainer").style.gridTemplateColumns = "1fr";
  });

  // Head Depth Gate Toggle
  let depthReady = false;
  container.querySelector("#percepToggleDepthBtn").addEventListener("click", async () => {
    depthReady = !depthReady;
    try {
      await api.controlPerception("set_head_depth", { enabled: depthReady });
      const badge = container.querySelector("#percepDepthGateBadge");
      badge.textContent = depthReady ? "头部深度: READY" : "头部深度: OFFLINE";
      badge.className = `badge ${depthReady ? "badge-passed" : "badge-idle"}`;
      container.querySelector("#percepToggleDepthBtn").textContent = depthReady ? "关闭深度相机" : "启动头部深度相机";
    } catch (err) {
      alert(`切换深度门禁失败: ${err.message}`);
    }
  });

  // Strict 3-Step Pipeline Buttons
  let yoloRunning = false;
  let latchedGeneration = null;
  let sixDRunning = false;

  container.querySelector("#btnStartYolo").addEventListener("click", async () => {
    try {
      const res = await api.controlPerception("start_yolo", { camera: "head_left_rgb", prompt: "cup" });
      yoloRunning = true;
      updatePipelineUI();
    } catch (err) {
      alert(`启动 YOLO 失败: ${err.message}`);
    }
  });

  container.querySelector("#btnStopYolo").addEventListener("click", async () => {
    if (sixDRunning) {
      const ok = confirm("【门禁警告】当前 FoundationPose 6D 仍在运行，停止 YOLO 将导致追踪丢失并自动中止 6D 姿态解算。确认停止？");
      if (!ok) return;
    }
    try {
      await api.controlPerception("stop_yolo");
      yoloRunning = false;
      latchedGeneration = null;
      sixDRunning = false;
      updatePipelineUI();
    } catch (err) {
      alert(`停止 YOLO 失败: ${err.message}`);
    }
  });

  container.querySelector("#btnLatchSegment").addEventListener("click", async () => {
    if (!yoloRunning) {
      alert("【三步门禁阻断】必须先启动第 ① 步 YOLO 检测并识别到候选物体，才能锁存最佳分割！");
      return;
    }
    try {
      const res = await api.controlPerception("latch_yolo", {});
      latchedGeneration = res.perception?.latched_mask?.generation || `gen_${Math.floor(Math.random() * 900 + 100)}`;
      updatePipelineUI();
    } catch (err) {
      alert(`锁存分割失败: ${err.message}`);
    }
  });

  container.querySelector("#btnStart6D").addEventListener("click", async () => {
    if (!latchedGeneration) {
      alert("【三步门禁阻断】必须先在第 ② 步成功锁存目标分割 (Latch)，才能启动 FoundationPose 6D 姿态估计！");
      return;
    }
    try {
      await api.controlPerception("start_foundation_pose", { camera: "head_left_rgb", classes: "cup" });
      sixDRunning = true;
      updatePipelineUI();
    } catch (err) {
      alert(`启动 FoundationPose 6D 失败: ${err.message}`);
    }
  });

  container.querySelector("#btnStop6D").addEventListener("click", async () => {
    try {
      await api.controlPerception("stop_foundation_pose");
      sixDRunning = false;
      updatePipelineUI();
    } catch (err) {
      alert(`停止 6D 解算失败: ${err.message}`);
    }
  });

  container.querySelector("#percepResetPipelineBtn").addEventListener("click", async () => {
    try {
      await api.controlPerception("stop_foundation_pose").catch(() => {});
      await api.controlPerception("stop_yolo").catch(() => {});
      yoloRunning = false;
      latchedGeneration = null;
      sixDRunning = false;
      updatePipelineUI();
    } catch (err) {
      alert(`重置感知流水线失败: ${err.message}`);
    }
  });

  function updatePipelineUI() {
    const s1Badge = container.querySelector("#step1StatusBadge");
    const s2Badge = container.querySelector("#step2StatusBadge");
    const s3Badge = container.querySelector("#step3StatusBadge");
    const pBadge = container.querySelector("#percepPipelineBadge");

    const yoloBox = container.querySelector("#yoloResultBox");
    const latchBox = container.querySelector("#latchResultBox");
    const sixDBox = container.querySelector("#sixDResultBox");

    if (yoloRunning) {
      s1Badge.textContent = "运行中 (30 FPS)";
      s1Badge.className = "badge badge-running";
      yoloBox.style.display = "block";
    } else {
      s1Badge.textContent = "未启动";
      s1Badge.className = "badge badge-idle";
      yoloBox.style.display = "none";
    }

    if (latchedGeneration) {
      s2Badge.textContent = "已锁存";
      s2Badge.className = "badge badge-passed";
      latchBox.style.display = "block";
      container.querySelector("#latchedGenTag").textContent = `${latchedGeneration} (cup, score 0.94)`;
    } else {
      s2Badge.textContent = "未锁存";
      s2Badge.className = "badge badge-idle";
      latchBox.style.display = "none";
    }

    if (sixDRunning) {
      s3Badge.textContent = "6D 解算中 (15 Hz)";
      s3Badge.className = "badge badge-running";
      sixDBox.style.display = "block";
      pBadge.textContent = "流水线: 6D 追踪中";
      pBadge.className = "badge badge-running";
    } else if (latchedGeneration) {
      s3Badge.textContent = "就绪 (已锁存)";
      s3Badge.className = "badge badge-info";
      sixDBox.style.display = "none";
      pBadge.textContent = "流水线: 目标已锁存";
      pBadge.className = "badge badge-info";
    } else if (yoloRunning) {
      s3Badge.textContent = "等待锁存";
      s3Badge.className = "badge badge-idle";
      sixDBox.style.display = "none";
      pBadge.textContent = "流水线: YOLO 运行中";
      pBadge.className = "badge badge-running";
    } else {
      s3Badge.textContent = "未就绪";
      s3Badge.className = "badge badge-idle";
      sixDBox.style.display = "none";
      pBadge.textContent = "流水线: IDLE";
      pBadge.className = "badge badge-idle";
    }
  }

  // ArUco Fine Alignment Button
  container.querySelector("#btnArucoFineAdjust").addEventListener("click", async () => {
    try {
      await api.controlPerception("aruco_set_fine", { enabled: true, offset_x: 0.005 });
      alert("已下发 ArUco 离散微调指令: 底盘向左对齐 5mm");
    } catch (err) {
      alert(`微调对齐失败 (请确认是否已获取控制 Lease): ${err.message}`);
    }
  });

  // Auto-refresh camera stream images at ~2Hz (mock frames)
  let imgTick = 0;
  refreshTimer = setInterval(() => {
    imgTick++;
    const robotId = api.currentRobotId;
    const imgHead = container.querySelector("#camImgHeadRgb");
    const imgDepth = container.querySelector("#camImgHeadDepth");
    const imgLeft = container.querySelector("#camImgWristLeft");
    const imgRight = container.querySelector("#camImgWristRight");

    if (imgHead) imgHead.src = `/api/robots/${robotId}/camera/head_rgb?_t=${imgTick}`;
    if (imgDepth) imgDepth.src = `/api/robots/${robotId}/camera/head_depth?_t=${imgTick}`;
    if (imgLeft) imgLeft.src = `/api/robots/${robotId}/camera/wrist_left?_t=${imgTick}`;
    if (imgRight) imgRight.src = `/api/robots/${robotId}/camera/wrist_right?_t=${imgTick}`;
  }, 1200);

  return {
    destroy() {
      if (refreshTimer) clearInterval(refreshTimer);
    },
    update(state) {
      if (!state) return;
      const desc = state.descriptor || {};
      const obs = state.observation || {};
      const cams = (desc.capabilities && desc.capabilities.cameras) || [];

      // Check if robot has cameras
      if (!cams || cams.length === 0) {
        container.querySelector("#cameraStreamsContainer").style.opacity = "0.3";
      } else {
        container.querySelector("#cameraStreamsContainer").style.opacity = "1.0";
      }

      // Check perception state from backend observation
      if (obs.perception) {
        if (obs.perception.yolo_active !== undefined) yoloRunning = obs.perception.yolo_active;
        if (obs.perception.latched_generation !== undefined) latchedGeneration = obs.perception.latched_generation;
        if (obs.perception.foundation_pose_active !== undefined) sixDRunning = obs.perception.foundation_pose_active;
        updatePipelineUI();
      }
    },
  };
}
