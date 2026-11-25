// Tab 2: Joints, 3D Visualization (R1 Pro / Piper Dual / Dexforce W1), 6D EE Step & Motrix Read-Only Mirror
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { api } from "../api.js";
import { drawer } from "../drawer.js";
import { createRobotModelViewer } from "../robot-model-viewer.js";
import { modelForDescriptor } from "../model-registry.js";
import { PosePreview, bindLocalPreviewInput } from "../pose-preview.js";
import { escapeHtml } from "../ui-format.js";
import { notify, enableDialogLightDismiss } from "../notify.js";

export function initJointsTab(container) {
  container.innerHTML = `
    <div class="card pose-preview-toolbar">
      <div class="robot-quick-switch" role="group" aria-label="快捷切换 3D 机器人模型">
        <button type="button" class="quick-robot-btn" data-quick-robot="r1pro-mock">R1 Pro (20 DOF)</button>
        <button type="button" class="quick-robot-btn" data-quick-robot="piper-dual-mock">Piper Dual (14 DOF)</button>
        <button type="button" class="quick-robot-btn" data-quick-robot="dex-w1-mock">Dexforce W1 (20 DOF)</button>
      </div>
      <div class="brand-divider" style="height:18px;"></div>
      <label for="poseViewMode" style="font-weight:600;">3D 姿态来源</label>
      <select id="poseViewMode" class="select-clean"><option value="target">本地目标预览 · 拖动立即显示</option><option value="feedback">当前反馈姿态 · Mock 反馈来源</option></select>
      <span id="poseModeNote">拖动仅改本地模型，不申 Lease、不发动作。</span>
      <span class="badge badge-no_result" id="pendingDiffBadge" style="margin-left:auto;">与反馈一致</span>
    </div>
    <div class="grid-split-view">
      <!-- Left Column: 3D Viewport & Motrix Mirror Status -->
      <div>
        <div class="viewport-3d-box" id="viewportBox">
          <canvas class="viewport-3d-canvas" id="viewportCanvas"></canvas>
          <div class="viewport-loading-overlay" id="viewportLoadingOverlay" hidden>
            <div class="viewport-loading-label" id="viewportLoadingLabel">加载模型中...</div>
            <div class="viewport-loading-track"><div class="viewport-loading-fill" id="viewportLoadingFill" style="width:0%;"></div></div>
          </div>
          <div class="viewport-plan-badge" id="viewportPlanBadge" hidden>
            <div class="vp-plan-row">
              <span class="vp-plan-status"><span class="vp-live-dot"></span><strong id="vpPlanTitle">轨迹预演中</strong><span id="vpPlanPct" class="vp-plan-pct">0%</span></span>
              <span class="vp-legend-group">
                <span class="vp-legend-item"><i class="vp-swatch vp-swatch-ghost"></i>全息幽灵=预演</span>
                <span class="vp-legend-item"><i class="vp-swatch vp-swatch-start"></i>起点</span>
                <span class="vp-legend-item"><i class="vp-swatch vp-swatch-end"></i>终点</span>
                <span class="vp-legend-item"><i class="vp-swatch vp-swatch-trail"></i>发光轨迹+箭头</span>
              </span>
              <button type="button" class="vp-focus-btn" id="btnVpFocusTrail" title="平滑拉近镜头对准当前规划轨迹">🎯 聚焦轨迹</button>
            </div>
            <div class="vp-progress-track"><div class="vp-progress-fill" id="vpPlanProgressFill" style="width:0%;"></div></div>
          </div>
          <div class="viewport-overlay-hud" id="viewportHud">
            本地只读模型视图 · 等待机型选择（不加载硬件驱动）
          </div>
          <div class="viewport-cam-bar" role="toolbar" aria-label="3D 视角与渲染控制">
            <button type="button" class="cam-preset-btn" data-cam-preset="front">正面</button>
            <button type="button" class="cam-preset-btn" data-cam-preset="side">侧面</button>
            <button type="button" class="cam-preset-btn" data-cam-preset="top">俯视</button>
            <button type="button" class="cam-preset-btn" data-cam-preset="iso">重置</button>
            <button type="button" class="cam-preset-btn active" id="btnToggleGhostBar" aria-pressed="true" title="显示/隐藏全息运动幽灵">幽灵</button>
            <button type="button" class="cam-preset-btn" id="btnToggleAutoRotate" aria-pressed="false" title="360° 缓慢旋转检视">旋转</button>
            <button type="button" class="cam-preset-btn" id="btnToggleViewportTheme" aria-pressed="false" title="切换暗色/亮色工作室背景">亮底</button>
            <button type="button" class="cam-preset-btn active" id="btnToggleAxes" aria-pressed="true" title="显示/隐藏世界坐标轴">坐标轴</button>
          </div>
        </div>

        <!-- Dexforce W1 / R1 Pro Unified Trajectory Plan Preview & Scrub Bar -->
        <div id="planPreviewBar" class="card plan-preview-bar" hidden>
          <div style="display:flex; justify-content:space-between; align-items:center; gap:8px; flex-wrap:wrap; margin-bottom:8px;">
            <div id="planInfoLine" style="font-size:12px; color:var(--text-main); font-weight:500;">轨迹预览就绪</div>
            <div style="display:flex; gap:6px; align-items:center; flex-wrap:wrap;">
              <button type="button" class="btn btn-sm" id="btnFocusTrail" title="拉近镜头聚焦当前轨迹区域">🎯 聚焦轨迹</button>
              <button type="button" class="btn btn-sm" id="btnPlanSpeed" title="切换预演倍速 (0.5x / 1x / 2x)">1.0×</button>
              <button type="button" class="btn btn-sm active" id="btnToggleTrail">轨迹●</button>
              <button type="button" class="btn btn-sm" id="btnCancelPlan">取消规划</button>
              <button type="button" class="btn btn-sm btn-step-debug" id="btnStepExecPlan" hidden title="以单步调试模式执行完整工作流：每步闭环到位后自动暂停，确认安全后点「下一步」（需 Lease）">⏭ 单步执行（需 Lease）</button>
              <button type="button" class="btn btn-sm btn-primary" id="btnExecPlan">确认执行规划（需 Lease）</button>
            </div>
          </div>
          <div class="plan-scrub-row">
            <button type="button" class="btn btn-sm" id="btnPlanPlay" style="min-width:56px;">⏸ 暂停</button>
            <input type="range" id="planScrubSlider" min="0" max="1.2" step="0.01" value="0" style="flex:1; accent-color:#0284c7;">
            <span id="planTimeLabel" style="font-family:var(--font-mono); font-size:11px; color:var(--text-muted); min-width:86px; text-align:right;">0.00 / 1.20 s</span>
          </div>
          <div id="planStepTimeline" class="plan-step-timeline" hidden></div>
          <div id="planContextLine" style="display:none; margin-top:6px; font-family:var(--font-mono); font-size:11px; color:var(--accent); background:var(--accent-light); padding:4px 8px; border-radius:4px;"></div>
        </div>

        <!-- Motrix WASM Read-Only Mirror Status Box -->
        <div class="card" style="margin-top: 14px;">
          <div class="card-header" style="margin-bottom:6px; padding-bottom:6px;">
            <span class="card-title" style="font-size:13px;">Motrix 只读消息桥（WASM 仿真器未接入）</span>
            <span class="badge badge-passed" id="motrixIsoBadge">本地协议隔离</span>
          </div>
          <div style="font-size:12px; color:var(--text-muted); display:flex; justify-content:space-between; margin-bottom:6px; flex-wrap:wrap; gap:6px;">
            <span>WebSocket 地址: <code style="font-family:var(--font-mono); color:var(--text-main);" id="motrixWsPath">/ws/motrix</code></span>
            <span>已连接查看器: <b id="motrixClientsCount">0</b> · 拦截反向写: <b id="motrixRejectedCount">0</b></span>
          </div>
          <p style="font-size:11px; color:#64748b; background:var(--bg-subtle); padding:6px 10px; border-radius:var(--radius-sm);">
            严格只读隔离断言：本镜像桥零加载底层 CAN/ROS 写驱动，绝不通过 WebSocket 反向写控机器人，仅提供 Observation 广播协议；尚未接入 Motrix WASM 客户端。
          </p>
        </div>

        <!-- 20-Second Live Trajectory Curve (Motphys Loop HiDPI Oscilloscope) -->
        <div class="card curve-card" style="margin-top: 14px;">
          <div class="card-header curve-card-header">
            <div style="display:flex; align-items:center; gap:8px; min-width:0;">
              <span class="card-title" style="font-size:13px;">实时关节轨迹曲线</span>
              <span class="badge badge-info" id="curveWindowBadge">20s · 实时</span>
            </div>
            <div style="display:flex; align-items:center; gap:6px; flex-wrap:wrap;">
              <select class="select-clean" id="curveJointSelect" style="padding:3px 8px; font-size:12px;"></select>
              <button type="button" class="btn btn-sm" id="curvePauseBtn" title="暂停或恢复实时采样">⏸ 暂停</button>
              <button type="button" class="btn btn-sm" id="curveClearBtn" title="清空当前 20s 波形缓存">清空</button>
            </div>
          </div>
          <div class="curve-kpi-strip" id="curveKpiStrip">
            <span class="curve-kpi-pill"><i class="curve-dot curve-dot-fb"></i>反馈 <b id="curveKpiFb">--</b></span>
            <span class="curve-kpi-pill"><i class="curve-dot curve-dot-tgt"></i>目标 <b id="curveKpiTgt">--</b></span>
            <span class="curve-kpi-pill" id="curveKpiErrPill"><i class="curve-dot curve-dot-err"></i>偏差 Δe <b id="curveKpiErr">--</b></span>
            <span class="curve-kpi-pill curve-kpi-plan" id="curveKpiPlanPill" hidden><i class="curve-dot curve-dot-plan"></i>规划预演曲线</span>
            <span class="curve-kpi-limit" id="curveKpiLimit">软限位 [--, --]</span>
          </div>
          <div class="curve-stage" id="curveStage">
            <canvas id="curveCanvas" class="curve-canvas"></canvas>
            <div class="curve-tooltip" id="curveTooltip" hidden></div>
          </div>
        </div>
      </div>

      <!-- Right Column: Grouped Joint Sliders & 6D EE Controls -->
      <div>
        <!-- Action Toolbar -->
        <div class="card" style="padding:12px 16px; margin-bottom:12px;">
          <div style="display:flex; flex-wrap:wrap; gap:8px; align-items:center; justify-content:space-between;">
            <div style="display:flex; gap:6px; flex-wrap:wrap; align-items:center;">
              <button class="btn btn-sm" id="readCurrentBtn">↺ 读当前</button>
              <button class="btn btn-sm" id="homePresetBtn" title="将本地预览目标设为机型默认安全位（不自动下发）">安全初始位</button>
              <label style="display:inline-flex; align-items:center; gap:4px; font-size:11px; color:var(--text-muted);">
                时长
                <input type="number" id="execDurationInput" value="1.2" min="0.2" max="15" step="0.2" style="width:52px; padding:3px 5px; border:1px solid var(--border); border-radius:4px; font-family:var(--font-mono); font-size:11px;">s
              </label>
              <button class="btn btn-sm" id="planPreviewBtn" style="border-color:#8b5cf6; color:#6d28d9; background:#f5f3ff;" title="先计算五次多项式轨迹并在 3D 视口以紫色幽灵+霓虹轨迹预演（免 Lease、不运动）">规划轨迹预览</button>
              <button class="btn btn-sm btn-primary" id="execTargetBtn">执行到 Mock（需 Lease）</button>
            </div>
            <div style="display:flex; gap:6px; flex-wrap:wrap;">
              <button class="btn btn-sm" id="saveActionBtn">保存身体原子动作</button>
              <button class="btn btn-sm" id="teachCheckBtn">示教仅释放双臂</button>
            </div>
          </div>
          <div class="wf-direct-preview-bar">
            <span style="font-size:11px; font-weight:600; color:var(--text-muted); white-space:nowrap;">多步工作流预演:</span>
            <select id="jointsWorkflowSelect" class="select-clean" aria-label="选择要预演的完整工作流" style="flex:1; min-width:170px; padding:3px 8px; font-size:12px;">
              <option value="">加载当前机型工作流...</option>
            </select>
            <button type="button" class="btn btn-sm btn-wf-preview-direct" id="btnPreviewSelectedWorkflow" title="在 3D 视口分色预演该工作流全部原子动作串联成的完整轨迹（免 Lease）">▶ 预演完整工作流轨迹</button>
          </div>
        </div>

        <!-- 6D End-Effector IK Preview & Step Panel -->
        <div class="card" id="eeCard" style="margin-bottom:12px;">
          <div class="card-header" style="margin-bottom:8px; padding-bottom:6px; flex-wrap:wrap; gap:6px;">
            <span class="card-title" style="font-size:13px;">末端 6D IK 轨迹预览与点动</span>
            <div style="display:flex; gap:6px; align-items:center; flex-wrap:wrap;">
              <select id="eeModeSelect" class="select-clean" aria-label="末端控制模式" style="padding:2px 6px; font-size:11px;">
                <option value="ik_preview" selected>IK 轨迹预览（免 Lease）</option>
                <option value="direct_step">直接点动执行（需 Lease）</option>
              </select>
              <select id="eeStepSize" class="select-clean" aria-label="末端平移步长" style="padding:2px 6px; font-size:11px;">
                <option value="0.02">2 cm</option>
                <option value="0.05" selected>5 cm</option>
                <option value="0.10">10 cm</option>
              </select>
              <button class="btn btn-sm" id="eeArmLeftBtn" style="background:var(--accent-light); color:var(--accent);">左臂 Left</button>
              <button class="btn btn-sm" id="eeArmRightBtn">右臂 Right</button>
            </div>
          </div>
          <div style="display:grid; grid-template-columns: repeat(3, 1fr); gap:6px; font-size:12px; margin-bottom:6px;">
            <button class="btn btn-sm" data-ee-lin="dx,1">+X (前)</button>
            <button class="btn btn-sm" data-ee-lin="dx,-1">-X (后)</button>
            <button class="btn btn-sm" data-ee-lin="dz,1">+Z (上)</button>
            <button class="btn btn-sm" data-ee-lin="dy,1">+Y (左)</button>
            <button class="btn btn-sm" data-ee-lin="dy,-1">-Y (右)</button>
            <button class="btn btn-sm" data-ee-lin="dz,-1">-Z (下)</button>
          </div>
          <div style="display:grid; grid-template-columns: repeat(6, 1fr); gap:4px; font-size:11px; margin-bottom:8px;">
            <button class="btn btn-sm" data-ee-rot="droll,10" style="padding:3px 4px;">+R 10°</button>
            <button class="btn btn-sm" data-ee-rot="droll,-10" style="padding:3px 4px;">-R 10°</button>
            <button class="btn btn-sm" data-ee-rot="dpitch,10" style="padding:3px 4px;">+P 10°</button>
            <button class="btn btn-sm" data-ee-rot="dpitch,-10" style="padding:3px 4px;">-P 10°</button>
            <button class="btn btn-sm" data-ee-rot="dyaw,10" style="padding:3px 4px;">+Y 10°</button>
            <button class="btn btn-sm" data-ee-rot="dyaw,-10" style="padding:3px 4px;">-Y 10°</button>
          </div>
          <div class="ik-coord-grid">
            <label><span>X</span><input type="number" id="ikX" step="0.01" value="0.35"></label>
            <label><span>Y</span><input type="number" id="ikY" step="0.01" value="0.35"></label>
            <label><span>Z</span><input type="number" id="ikZ" step="0.01" value="0.90"></label>
            <label><span>R°</span><input type="number" id="ikRoll" step="5" value="0"></label>
            <label><span>P°</span><input type="number" id="ikPitch" step="5" value="0"></label>
            <label><span>Y°</span><input type="number" id="ikYaw" step="5" value="0"></label>
          </div>
          <div style="display:flex; gap:6px; align-items:center; margin-top:6px; flex-wrap:wrap;">
            <button type="button" class="btn btn-sm" id="syncFkBtn">↺ 同步当前 FK</button>
            <button type="button" class="btn btn-sm btn-primary" id="solveIkBtn" style="flex:1;">求解 URDF 6D IK 并预览轨迹</button>
          </div>
          <div style="margin-top:8px; font-family:var(--font-mono); font-size:11px; color:var(--text-muted); background:var(--bg-subtle); padding:6px 8px; border-radius:var(--radius-sm);" id="eePoseDisplay">
            Pos: x=0.000 y=0.000 z=0.000 | RPY: 0.0° 0.0° 0.0°
          </div>
        </div>

        <!-- Joint Sliders Container (Grouped) -->
        <div class="card">
          <div class="card-header" style="margin-bottom:8px; padding-bottom:6px;">
            <span class="card-title" style="font-size:13px;">关节控制与限位表 (Joints)</span>
            <span style="font-size:11px; color:var(--text-muted);">单位: rad / norm · 橙框提示临近软限位</span>
          </div>
          <p class="preview-field-note">滑块 / 数值：本地目标预览；中间读数：当前 Mock 反馈。预览≠执行；青色高亮表示目标与当前反馈存在差值。</p>
          <div id="jointsSlidersContainer" style="max-height: 480px; overflow-y: auto; padding-right: 4px;">
            <div style="text-align:center; color:#94a3b8; padding:20px;">加载关节定义...</div>
          </div>
        </div>
        <div id="handPreviewPanel" class="card" style="margin-top:14px;"></div>
      </div>
    </div>

    <!-- Native Dialog: Save Current Body Targets as Atomic Action -->
    <dialog id="savePoseDialog" closedby="any" aria-labelledby="savePoseTitle">
      <div class="dialog-content">
        <div class="dialog-header">
          <h1 class="dialog-title" id="savePoseTitle">保存当前身体目标为原子动作</h1>
          <button type="button" class="drawer-close-btn" id="closeSavePoseBtn" aria-label="关闭对话框">✕</button>
        </div>
        <div class="form-group">
          <label class="form-label" for="savePoseNameInput">动作名称 (Name)</label>
          <input type="text" class="form-input" id="savePoseNameInput" autofocus>
        </div>
        <div class="grid-2" style="gap:10px;">
          <div class="form-group">
            <label class="form-label" for="savePoseDurationInput">目标执行时长 (秒)</label>
            <input type="number" class="form-input" id="savePoseDurationInput" value="1.4" min="0.2" max="20" step="0.1">
          </div>
          <div class="form-group">
            <label class="form-label" for="savePoseTagsInput">标签 (逗号分隔)</label>
            <input type="text" class="form-input" id="savePoseTagsInput" value="pose, joints">
          </div>
        </div>
        <div class="form-group">
          <label class="form-label" for="savePoseDescInput">说明描述</label>
          <input type="text" class="form-input" id="savePoseDescInput" value="从关节控制面板保存的目标姿态">
        </div>
        <div style="font-size:11px; color:var(--text-muted); background:var(--bg-subtle); padding:8px 10px; border-radius:var(--radius-sm); font-family:var(--font-mono);" id="savePoseSummary">
          将保存身体关节目标（不含手部本地预览关节）。
        </div>
        <div class="dialog-actions">
          <form method="dialog" style="display:inline;"><button type="submit" value="cancel" class="btn btn-sm">取消</button></form>
          <button type="button" class="btn btn-sm btn-primary" id="confirmSavePoseBtn">确认保存动作</button>
        </div>
      </div>
    </dialog>
  `;

  // 1. Initialize Three.js Studio 3D Viewport
  const viewportBox = container.querySelector("#viewportBox");
  const canvas = container.querySelector("#viewportCanvas");
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false });
  renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
  renderer.setClearColor(0x0f172a, 1);
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.1;
  renderer.setSize(canvas.clientWidth || 500, canvas.clientHeight || 380, false);

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(34, (canvas.clientWidth || 500) / (canvas.clientHeight || 380), 0.05, 50);
  camera.position.set(1.4, 1.8, 1.9);

  const controls = new OrbitControls(camera, canvas);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.target.set(0, 0.55, 0);

  // Studio 3-point lighting + subtle floor disc & grid
  const hemi = new THREE.HemisphereLight(0xf1f5f9, 0x1e293b, 1.25);
  scene.add(hemi);
  const keyLight = new THREE.DirectionalLight(0xfffbeb, 1.95);
  keyLight.position.set(2.4, 4.2, 3.2);
  scene.add(keyLight);
  const fillLight = new THREE.DirectionalLight(0xdbeafe, 0.95);
  fillLight.position.set(-2.8, 2.4, 1.5);
  scene.add(fillLight);
  const rimLight = new THREE.DirectionalLight(0x99f6e4, 0.65);
  rimLight.position.set(-1.5, 2.2, -3.2);
  scene.add(rimLight);

  let grid = new THREE.GridHelper(2.8, 28, 0x167e83, 0x334155);
  scene.add(grid);

  const loadingOverlay = container.querySelector("#viewportLoadingOverlay");
  const loadingLabel = container.querySelector("#viewportLoadingLabel");
  const loadingFill = container.querySelector("#viewportLoadingFill");

  const modelViewer = createRobotModelViewer({
    scene,
    camera,
    controls,
    hud: container.querySelector("#viewportHud"),
    onLoadingState({ loading, progress, label }) {
      if (!loadingOverlay) return;
      loadingOverlay.hidden = !loading;
      if (loadingLabel && label) loadingLabel.textContent = label;
      if (loadingFill) loadingFill.style.width = `${progress || 0}%`;
    },
  });

  // Dynamic quick robot switcher pills -> delegates through header #robotSelector (preserves safe Lease release!)
  const quickSwitchWrap = container.querySelector(".robot-quick-switch");
  function renderQuickRobotButtons(robots) {
    if (!quickSwitchWrap || !Array.isArray(robots) || !robots.length) return;
    quickSwitchWrap.innerHTML = robots.map((r) => {
      const shortName = r.id === "r1pro-mock" ? "R1 Pro" : r.id === "piper-dual-mock" ? "Piper Dual" : r.id === "dex-w1-mock" ? "Dexforce W1" : r.name;
      return `<button type="button" class="quick-robot-btn ${r.id === currentDescriptor?.id ? "active" : ""}" data-quick-robot="${escapeHtml(r.id)}">${escapeHtml(shortName)} (${r.dof} DOF)</button>`;
    }).join("");
    quickSwitchWrap.querySelectorAll("[data-quick-robot]").forEach((btn) => {
      btn.addEventListener("click", () => {
        const targetId = btn.dataset.quickRobot;
        const selector = document.querySelector("#robotSelector");
        if (selector && selector.value !== targetId && !selector.disabled) {
          selector.value = targetId;
          selector.dispatchEvent(new Event("change"));
        }
      });
    });
  }
  const onRobotsUpdated = (e) => renderQuickRobotButtons(e.detail?.robots || []);
  window.addEventListener("motphys:robots-updated", onRobotsUpdated);
  container.querySelectorAll("[data-quick-robot]").forEach(btn => {
    btn.addEventListener("click", () => {
      const targetId = btn.dataset.quickRobot;
      const selector = document.querySelector("#robotSelector");
      if (selector && selector.value !== targetId && !selector.disabled) {
        selector.value = targetId;
        selector.dispatchEvent(new Event("change"));
      }
    });
  });

  // Camera presets & studio display toggles
  container.querySelectorAll("[data-cam-preset]").forEach(btn => {
    btn.addEventListener("click", () => modelViewer.setCameraPreset(btn.dataset.camPreset));
  });

  let autoRotate = false;
  const autoRotBtn = container.querySelector("#btnToggleAutoRotate");
  autoRotBtn.addEventListener("click", () => {
    autoRotate = !autoRotate;
    modelViewer.setAutoRotate(autoRotate);
    autoRotBtn.setAttribute("aria-pressed", String(autoRotate));
    autoRotBtn.classList.toggle("active", autoRotate);
  });

  let lightTheme = false;
  const themeBtn = container.querySelector("#btnToggleViewportTheme");
  themeBtn.addEventListener("click", () => {
    lightTheme = !lightTheme;
    themeBtn.setAttribute("aria-pressed", String(lightTheme));
    themeBtn.classList.toggle("active", lightTheme);
    themeBtn.textContent = lightTheme ? "暗底" : "亮底";
    viewportBox.classList.toggle("viewport-light-studio", lightTheme);
    renderer.setClearColor(lightTheme ? 0xeef2f6 : 0x0f172a, 1);
    modelViewer.setLightTheme(lightTheme);
    hemi.intensity = lightTheme ? 1.55 : 1.25;
    hemi.groundColor.setHex(lightTheme ? 0xcbd5e1 : 0x1e293b);
    scene.remove(grid);
    grid.geometry.dispose();
    grid.material.dispose();
    grid = new THREE.GridHelper(2.8, 28, 0x167e83, lightTheme ? 0xcbd5e1 : 0x334155);
    scene.add(grid);
  });

  let axesVisible = true;
  const axesBtn = container.querySelector("#btnToggleAxes");
  axesBtn.addEventListener("click", () => {
    axesVisible = !axesVisible;
    modelViewer.setAxesVisible(axesVisible);
    axesBtn.setAttribute("aria-pressed", String(axesVisible));
    axesBtn.classList.toggle("active", axesVisible);
  });

  let ghostVisible = true;
  const ghostBarBtn = container.querySelector("#btnToggleGhostBar");
  ghostBarBtn.addEventListener("click", () => {
    ghostVisible = !ghostVisible;
    modelViewer.setGhostEnabled(ghostVisible);
    ghostBarBtn.setAttribute("aria-pressed", String(ghostVisible));
    ghostBarBtn.classList.toggle("active", ghostVisible);
  });

  // Trajectory Plan Preview Playback State (High-Visibility Holographic Ghost + X-Ray Trail)
  let activePlan = null;
  let planScrub = 0;
  let planPlaying = true;
  let planEndHold = 0;
  let planSpeed = 1.0;
  let trailVisible = true;
  let lastFrameTs = performance.now();

  const planBar = container.querySelector("#planPreviewBar");
  const planInfoLine = container.querySelector("#planInfoLine");
  const planContextLine = container.querySelector("#planContextLine");
  const planStepTimeline = container.querySelector("#planStepTimeline");
  const planScrubSlider = container.querySelector("#planScrubSlider");
  const planTimeLabel = container.querySelector("#planTimeLabel");
  const btnPlanPlay = container.querySelector("#btnPlanPlay");
  const btnPlanSpeed = container.querySelector("#btnPlanSpeed");
  const btnToggleTrail = container.querySelector("#btnToggleTrail");
  const btnExecPlan = container.querySelector("#btnExecPlan");
  const viewportPlanBadge = container.querySelector("#viewportPlanBadge");
  const vpPlanTitle = container.querySelector("#vpPlanTitle");
  const vpPlanPct = container.querySelector("#vpPlanPct");
  const vpPlanProgressFill = container.querySelector("#vpPlanProgressFill");
  const wfSelect = container.querySelector("#jointsWorkflowSelect");
  const btnPreviewWfDirect = container.querySelector("#btnPreviewSelectedWorkflow");
  let baseContextSummary = "";

  async function refreshWorkflowPicker() {
    if (!wfSelect) return;
    try {
      const res = await api.listWorkflows();
      const wfs = res.workflows || [];
      if (!wfs.length) {
        wfSelect.innerHTML = `<option value="">当前机型暂无工作流（可在「动作与工作流」新建）</option>`;
        if (btnPreviewWfDirect) btnPreviewWfDirect.disabled = true;
        return;
      }
      wfSelect.innerHTML = wfs.map(w =>
        `<option value="${escapeHtml(w.id)}">${escapeHtml(w.name)} (${(w.steps || []).length} 步)</option>`
      ).join("");
      if (btnPreviewWfDirect) btnPreviewWfDirect.disabled = false;
    } catch {
      wfSelect.innerHTML = `<option value="">工作流列表加载失败</option>`;
      if (btnPreviewWfDirect) btnPreviewWfDirect.disabled = true;
    }
  }

  btnPreviewWfDirect?.addEventListener("click", async () => {
    const wfid = wfSelect?.value;
    if (!wfid) return;
    try {
      const res = await api.previewWorkflow(wfid);
      const ctxKeys = Object.keys(res.workflow_context || {});
      const ctxSummary = ctxKeys.length
        ? `参数传递上下文: ${ctxKeys.map(k => `${k}=${JSON.stringify(res.workflow_context[k]?.xyz_m || res.workflow_context[k])}`).join(" · ")}`
        : "";
      setActivePlan(res.plan, ctxSummary, { autoFocus: true });
      notify(`已在 3D 视口分色预演完整工作流（共 ${(res.plan?.step_segments || res.steps || []).length} 步 · 未下发执行）`, "success");
    } catch (err) {
      notify("工作流完整轨迹预演失败: " + err.message, "error");
    }
  });

  function updatePlanScrubUI() {
    if (!activePlan) return;
    const dur = Math.max(0.01, activePlan.duration || 1.0);
    const cur = Math.min(dur, planScrub);
    const pct = Math.round((cur / dur) * 100);
    planScrubSlider.max = String(dur);
    planScrubSlider.value = String(cur);
    planTimeLabel.textContent = `${cur.toFixed(2)} / ${dur.toFixed(2)} s`;
    if (vpPlanPct) vpPlanPct.textContent = `${pct}%`;
    if (vpPlanProgressFill) vpPlanProgressFill.style.width = `${pct}%`;

    const segs = activePlan.step_segments;
    if (Array.isArray(segs) && segs.length > 1) {
      const activeSeg = segs.find(s => cur >= s.t_start - 1e-4 && cur <= s.t_end + 1e-4) || segs[segs.length - 1];
      if (activeSeg && vpPlanTitle) {
        const label = `步 ${activeSeg.step_index}/${segs.length} · ${activeSeg.action_name}`;
        vpPlanTitle.textContent = label;
        vpPlanTitle.title = `${label} (${activeSeg.note || ""})`;
      }
      if (planStepTimeline) {
        planStepTimeline.querySelectorAll(".plan-step-chip").forEach(chip => {
          const isCur = Number(chip.dataset.stepIndex) === activeSeg?.step_index;
          chip.classList.toggle("active", isCur);
        });
      }
    }
  }

  function renderPlanStepTimeline(plan) {
    if (!planStepTimeline) return;
    const segs = plan?.step_segments;
    if (!Array.isArray(segs) || segs.length <= 1) {
      planStepTimeline.hidden = true;
      planStepTimeline.innerHTML = "";
      return;
    }
    planStepTimeline.hidden = false;
    planStepTimeline.innerHTML = segs.map((s) => `
      <button type="button" class="plan-step-chip" data-step-index="${s.step_index}" data-t-start="${s.t_start}" data-t-end="${s.t_end}" title="${escapeHtml(s.note || s.action_name)}">
        <span class="psc-dot" style="background:${escapeHtml(s.color || "#00e5ff")};"></span>
        <span class="psc-idx">步${s.step_index}</span>
        <span class="psc-name">${escapeHtml(s.action_name)}</span>
        <span class="psc-dur">${Number(s.duration_s || 0).toFixed(1)}s</span>
      </button>
    `).join('<span class="psc-arrow" aria-hidden="true">→</span>');

    planStepTimeline.querySelectorAll(".plan-step-chip").forEach(btn => {
      btn.addEventListener("click", () => {
        if (!activePlan) return;
        const tStart = parseFloat(btn.dataset.tStart) || 0;
        const tEnd = parseFloat(btn.dataset.tEnd) || tStart;
        const idx = Number(btn.dataset.stepIndex);
        const seg = (activePlan.step_segments || []).find(x => x.step_index === idx);
        planPlaying = false;
        btnPlanPlay.textContent = "▶ 播放";
        // Jump to midpoint of clicked step so user immediately sees that step's motion pose
        planScrub = Math.min(activePlan.duration || tEnd, tStart + (tEnd - tStart) * 0.6);
        modelViewer.setPlanTime(planScrub);
        updatePlanScrubUI();
        if (seg?.note && planContextLine) {
          planContextLine.style.display = "block";
          planContextLine.textContent = `步 ${seg.step_index}/${activePlan.step_segments.length} [${seg.action_name}]: ${seg.note}` + (baseContextSummary ? `  |  ${baseContextSummary}` : "");
        }
      });
    });
  }

  function setActivePlan(plan, contextSummary = "", { autoFocus = false } = {}) {
    activePlan = plan || null;
    baseContextSummary = contextSummary || "";
    planScrub = 0;
    planEndHold = 0;
    planPlaying = true;
    btnPlanPlay.textContent = "⏸ 暂停";
    if (!activePlan) {
      planBar.hidden = true;
      if (planStepTimeline) planStepTimeline.hidden = true;
      if (viewportPlanBadge) viewportPlanBadge.hidden = true;
      viewportBox.classList.remove("viewport-planning-active");
      modelViewer.clearPlan();
      return;
    }
    planBar.hidden = false;
    if (viewportPlanBadge) viewportPlanBadge.hidden = false;
    viewportBox.classList.add("viewport-planning-active");
    const isWf = activePlan.kind === "workflow_preview" && Boolean(activePlan.workflow_id);
    if (btnExecPlan) {
      btnExecPlan.textContent = isWf ? "确认执行完整工作流（需 Lease）" : "确认执行规划（需 Lease）";
    }
    const btnStepExec = container.querySelector("#btnStepExecPlan");
    if (btnStepExec) btnStepExec.hidden = !isWf;
    const ik = activePlan.ik;
    const stepCount = (activePlan.step_segments || []).length;
    const kindLabel = activePlan.kind === "ik_cartesian"
      ? `IK 笛卡尔轨迹 (${ik?.arm || ""})`
      : activePlan.kind === "workflow_preview"
        ? `多步工作流 (${activePlan.workflow_name || ""} · ${stepCount}步)`
        : "关节五次多项式规划";
    if (vpPlanTitle) {
      vpPlanTitle.textContent = `${kindLabel} · 预演中`;
      vpPlanTitle.title = `${kindLabel} · 预演中`;
    }
    const ikTag = ik
      ? ` · 末段 IK 误差 <b>${(ik.pos_error_m * 1000).toFixed(2)}mm / ${ik.rot_error_deg.toFixed(1)}°</b> (${ik.ok ? "收敛" : "近似"})`
      : "";
    const clampCnt = Object.keys(activePlan.clamped || {}).length;
    planInfoLine.innerHTML =
      `<span class="badge badge-info" style="margin-right:6px;">${escapeHtml(kindLabel)}</span>` +
      `总计运动 <b>${(activePlan.moving || []).length}</b> 轴 · 总时长 <b>${(activePlan.duration || 0).toFixed(2)}s</b>` +
      ` · 峰值 <b>${(activePlan.max_peak_vel || 0).toFixed(2)}</b> rad/s · Δ <b>${(activePlan.max_delta || 0).toFixed(3)}</b>` +
      ikTag +
      (clampCnt ? ` · <span style="color:var(--warning);">${clampCnt} 限位截断</span>` : "");
    if (contextSummary) {
      planContextLine.style.display = "block";
      planContextLine.textContent = contextSummary;
    } else {
      planContextLine.style.display = "none";
      planContextLine.textContent = "";
    }
    renderPlanStepTimeline(activePlan);
    modelViewer.setTrailEnabled(trailVisible);
    modelViewer.setPlan(activePlan, { autoFocus });
    updatePlanScrubUI();
  }

  btnPlanPlay.addEventListener("click", () => {
    planPlaying = !planPlaying;
    if (planPlaying && activePlan && planScrub >= (activePlan.duration || 0)) {
      planScrub = 0;
      planEndHold = 0;
    }
    btnPlanPlay.textContent = planPlaying ? "⏸ 暂停" : "▶ 播放";
  });

  const PLAN_SPEEDS = [1.0, 0.5, 2.0];
  btnPlanSpeed.addEventListener("click", () => {
    const idx = PLAN_SPEEDS.indexOf(planSpeed);
    planSpeed = PLAN_SPEEDS[(idx + 1) % PLAN_SPEEDS.length];
    btnPlanSpeed.textContent = `${planSpeed.toFixed(1)}×`;
  });

  container.querySelector("#btnFocusTrail").addEventListener("click", () => {
    modelViewer.focusTrajectory();
  });
  container.querySelector("#btnVpFocusTrail").addEventListener("click", () => {
    modelViewer.focusTrajectory();
  });

  planScrubSlider.addEventListener("input", () => {
    if (!activePlan) return;
    planPlaying = false;
    btnPlanPlay.textContent = "▶ 播放";
    planScrub = parseFloat(planScrubSlider.value) || 0;
    modelViewer.setPlanTime(planScrub);
    updatePlanScrubUI();
  });

  btnToggleTrail.addEventListener("click", () => {
    trailVisible = !trailVisible;
    btnToggleTrail.textContent = trailVisible ? "轨迹●" : "轨迹○";
    btnToggleTrail.classList.toggle("active", trailVisible);
    modelViewer.setTrailEnabled(trailVisible);
  });

  container.querySelector("#btnCancelPlan").addEventListener("click", () => {
    setActivePlan(null);
    notify("已清除 3D 轨迹预览", "info");
  });

  container.querySelector("#btnStepExecPlan")?.addEventListener("click", async () => {
    if (!activePlan || activePlan.kind !== "workflow_preview" || !activePlan.workflow_id) return;
    try {
      const res = await api.runWorkflow(activePlan.workflow_id, false, { stepMode: true });
      notify(`已启动单步调试 '${activePlan.workflow_name}'（每步闭环到位后自动暂停）`, "success");
      setActivePlan(null);
      if (res?.job?.job_id) drawer.open(res.job.job_id);
    } catch (err) {
      notify("启动单步调试失败（需持有 Lease）: " + err.message, "error");
    }
  });

  container.querySelector("#btnExecPlan").addEventListener("click", async () => {
    if (!activePlan) return;
    try {
      if (activePlan.kind === "workflow_preview" && activePlan.workflow_id) {
        const res = await api.runWorkflow(activePlan.workflow_id, false);
        notify(`已下发完整多步工作流 '${activePlan.workflow_name}'，正在执行`, "success");
        setActivePlan(null);
        if (res?.job?.job_id) drawer.open(res.job.job_id);
        return;
      }
      const dur = Math.max(0.4, Math.min(20, activePlan.duration || 1.2));
      await api.sendJointTargets(activePlan.goal, dur);
      notify(`已下发规划终点目标 (${(activePlan.moving || []).length} 运动轴 · ${dur.toFixed(2)}s)`, "success");
      setActivePlan(null);
    } catch (err) {
      notify("执行规划失败（需持有 Lease）: " + err.message, "error");
    }
  });

  // Resize handler for 3D canvas
  function onResize() {
    if (!viewportBox) return;
    const w = viewportBox.clientWidth;
    const h = viewportBox.clientHeight;
    if (w > 0 && h > 0) {
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      renderer.setSize(w, h, false);
    }
  }
  window.addEventListener("resize", onResize);
  const resizeObserver = new ResizeObserver(onResize);
  resizeObserver.observe(viewportBox);
  setTimeout(onResize, 100);

  // 3D Render Loop (smoothly advances trajectory scrub playback when active)
  let animId = null;
  function animate(nowTs = performance.now()) {
    animId = requestAnimationFrame(animate);
    const dt = Math.min(0.1, Math.max(0, (nowTs - lastFrameTs) / 1000));
    lastFrameTs = nowTs;
    if (activePlan && planPlaying) {
      const dur = activePlan.duration || 1.0;
      if (planScrub < dur) {
        planScrub = Math.min(dur, planScrub + dt * planSpeed);
        modelViewer.setPlanTime(planScrub);
        updatePlanScrubUI();
        if (typeof drawCurve === "function") drawCurve();
      } else {
        planEndHold += dt;
        if (planEndHold >= 0.75) {
          planScrub = 0;
          planEndHold = 0;
          modelViewer.setPlanTime(0);
          updatePlanScrubUI();
          if (typeof drawCurve === "function") drawCurve();
        }
      }
    }
    modelViewer.stepFrame();
    controls.update();
    renderer.render(scene, camera);
  }
  animate();

  // 2. Trajectory 20s Live Curve — Retina HiDPI Motphys Loop Engineering Oscilloscope
  const CURVE_WINDOW_MS = 20000;
  const curveBuffer = [];
  let selectedJointName = "left_arm_1";
  let curvePaused = false;
  let curveHoverX = null;
  const curveStage = container.querySelector("#curveStage");
  const curveCanvas = container.querySelector("#curveCanvas");
  const curveCtx = curveCanvas.getContext("2d");
  const curveTooltip = container.querySelector("#curveTooltip");
  const curvePauseBtn = container.querySelector("#curvePauseBtn");
  const curveClearBtn = container.querySelector("#curveClearBtn");
  const curveWindowBadge = container.querySelector("#curveWindowBadge");
  const curveKpiFb = container.querySelector("#curveKpiFb");
  const curveKpiTgt = container.querySelector("#curveKpiTgt");
  const curveKpiErr = container.querySelector("#curveKpiErr");
  const curveKpiErrPill = container.querySelector("#curveKpiErrPill");
  const curveKpiPlanPill = container.querySelector("#curveKpiPlanPill");
  const curveKpiLimit = container.querySelector("#curveKpiLimit");

  let curveCssW = 560;
  let curveCssH = 192;
  function syncCurveCanvasSize() {
    if (!curveStage) return;
    const rect = curveStage.getBoundingClientRect();
    const w = Math.max(260, Math.round(rect.width || 560));
    const h = 192;
    const dpr = Math.min(2, window.devicePixelRatio || 1);
    if (curveCssW !== w || curveCanvas.width !== Math.round(w * dpr) || curveCanvas.height !== Math.round(h * dpr)) {
      curveCssW = w;
      curveCssH = h;
      curveCanvas.width = Math.round(w * dpr);
      curveCanvas.height = Math.round(h * dpr);
      curveCtx.setTransform(dpr, 0, 0, dpr, 0, 0);
    }
  }
  const curveResizeObserver = new ResizeObserver(() => {
    syncCurveCanvasSize();
    drawCurve();
  });
  curveResizeObserver.observe(curveStage);
  syncCurveCanvasSize();

  curvePauseBtn.addEventListener("click", () => {
    curvePaused = !curvePaused;
    curvePauseBtn.textContent = curvePaused ? "▶ 继续" : "⏸ 暂停";
    curvePauseBtn.classList.toggle("active", curvePaused);
    if (curveWindowBadge) {
      curveWindowBadge.textContent = curvePaused ? "20s · 已暂停" : "20s · 实时";
      curveWindowBadge.className = `badge ${curvePaused ? "badge-warning" : "badge-info"}`;
    }
  });

  curveClearBtn.addEventListener("click", () => {
    curveBuffer.length = 0;
    curveHoverX = null;
    if (curveTooltip) curveTooltip.hidden = true;
    drawCurve();
  });

  curveCanvas.addEventListener("mousemove", (e) => {
    const rect = curveCanvas.getBoundingClientRect();
    curveHoverX = e.clientX - rect.left;
    drawCurve();
  });
  curveCanvas.addEventListener("mouseleave", () => {
    curveHoverX = null;
    if (curveTooltip) curveTooltip.hidden = true;
    drawCurve();
  });

  function drawCurve() {
    const w = curveCssW;
    const h = curveCssH;
    const ctx = curveCtx;

    const spec = (currentDescriptor?.joints || []).find(j => j.name === selectedJointName) || null;
    const unit = spec?.unit === "norm" ? "norm" : "rad";
    const planSamples = (activePlan?.samples || []).filter(sp => sp?.q && Number.isFinite(sp.q[selectedJointName]));
    const hasPlanCurve = planSamples.length >= 2 && Boolean(activePlan?.moving?.includes(selectedJointName));

    // Update KPI header strip
    if (curveBuffer.length > 0) {
      const last = curveBuffer[curveBuffer.length - 1];
      const err = Math.abs((last.target ?? 0) - (last.pos ?? 0));
      if (curveKpiFb) curveKpiFb.textContent = `${(last.pos ?? 0).toFixed(3)} ${unit}`;
      if (curveKpiTgt) curveKpiTgt.textContent = `${(last.target ?? 0).toFixed(3)} ${unit}`;
      if (curveKpiErr) curveKpiErr.textContent = `${err.toFixed(3)} ${unit}`;
      if (curveKpiErrPill) {
        curveKpiErrPill.classList.toggle("curve-kpi-warn", err >= 0.04 && err < 0.25);
        curveKpiErrPill.classList.toggle("curve-kpi-alert", err >= 0.25);
      }
    } else {
      if (curveKpiFb) curveKpiFb.textContent = "--";
      if (curveKpiTgt) curveKpiTgt.textContent = "--";
      if (curveKpiErr) curveKpiErr.textContent = "--";
      if (curveKpiErrPill) curveKpiErrPill.classList.remove("curve-kpi-warn", "curve-kpi-alert");
    }
    if (curveKpiLimit) {
      curveKpiLimit.textContent = spec ? `软限位 [${spec.lower.toFixed(2)}, ${spec.upper.toFixed(2)}]` : "软限位 [--, --]";
    }
    if (curveKpiPlanPill) curveKpiPlanPill.hidden = !hasPlanCurve;

    // Background (Clean Motphys Loop light slate frame + crisp white plot surface)
    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = "#f8fafc";
    ctx.fillRect(0, 0, w, h);

    const padLeft = 48;
    const padRight = 14;
    const padTop = 14;
    const padBottom = 24;
    const plotW = Math.max(60, w - padLeft - padRight);
    const plotH = Math.max(40, h - padTop - padBottom);
    const liveW = hasPlanCurve ? Math.round(plotW * 0.72) : plotW;
    const planW = plotW - liveW;

    ctx.fillStyle = "#ffffff";
    ctx.fillRect(padLeft, padTop, plotW, plotH);
    ctx.strokeStyle = "#e2e8f0";
    ctx.lineWidth = 1;
    ctx.strokeRect(padLeft + 0.5, padTop + 0.5, plotW - 1, plotH - 1);

    // Dynamic smart Y range (includes live samples + active plan curve)
    const vals = [];
    for (const pt of curveBuffer) {
      if (Number.isFinite(pt.pos)) vals.push(pt.pos);
      if (Number.isFinite(pt.target)) vals.push(pt.target);
    }
    if (hasPlanCurve) {
      for (const sp of planSamples) vals.push(sp.q[selectedJointName]);
    }
    let minV = -1.0;
    let maxV = 1.0;
    if (vals.length > 0) {
      const lo = Math.min(...vals);
      const hi = Math.max(...vals);
      const span = Math.max(0.45, (hi - lo) * 1.45);
      const mid = (lo + hi) * 0.5;
      minV = mid - span * 0.5;
      maxV = mid + span * 0.5;
    }
    const toY = (v) => padTop + ((maxV - v) / Math.max(1e-4, maxV - minV)) * plotH;

    // Soft-limit guard zones (tinted light-rose bands if joint limits appear inside Y viewport)
    ctx.save();
    ctx.beginPath();
    ctx.rect(padLeft, padTop, plotW, plotH);
    ctx.clip();
    if (spec) {
      if (spec.upper < maxV) {
        const yHi = Math.max(padTop, Math.min(padTop + plotH, toY(spec.upper)));
        ctx.fillStyle = "rgba(239, 68, 68, 0.07)";
        ctx.fillRect(padLeft, padTop, plotW, yHi - padTop);
        ctx.strokeStyle = "rgba(239, 68, 68, 0.55)";
        ctx.setLineDash([3, 3]);
        ctx.beginPath();
        ctx.moveTo(padLeft, yHi);
        ctx.lineTo(padLeft + plotW, yHi);
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.fillStyle = "#dc2626";
        ctx.font = "600 9px ui-monospace, SFMono-Regular, Menlo, monospace";
        ctx.fillText(`MAX ${spec.upper.toFixed(2)}`, padLeft + 6, Math.min(padTop + plotH - 4, yHi + 10));
      }
      if (spec.lower > minV) {
        const yLo = Math.max(padTop, Math.min(padTop + plotH, toY(spec.lower)));
        ctx.fillStyle = "rgba(239, 68, 68, 0.07)";
        ctx.fillRect(padLeft, yLo, plotW, padTop + plotH - yLo);
        ctx.strokeStyle = "rgba(239, 68, 68, 0.55)";
        ctx.setLineDash([3, 3]);
        ctx.beginPath();
        ctx.moveTo(padLeft, yLo);
        ctx.lineTo(padLeft + plotW, yLo);
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.fillStyle = "#dc2626";
        ctx.font = "600 9px ui-monospace, SFMono-Regular, Menlo, monospace";
        ctx.fillText(`MIN ${spec.lower.toFixed(2)}`, padLeft + 6, Math.max(padTop + 10, yLo - 4));
      }
    }

    // Horizontal gridlines & zero line
    const yTicks = 4;
    for (let i = 0; i <= yTicks; i++) {
      const frac = i / yTicks;
      const y = padTop + frac * plotH;
      if (i > 0 && i < yTicks) {
        ctx.strokeStyle = "#eef2f6";
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(padLeft, Math.round(y) + 0.5);
        ctx.lineTo(padLeft + plotW, Math.round(y) + 0.5);
        ctx.stroke();
      }
    }
    if (minV < 0 && maxV > 0) {
      const y0 = Math.round(toY(0)) + 0.5;
      ctx.strokeStyle = "#cbd5e1";
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(padLeft, y0);
      ctx.lineTo(padLeft + plotW, y0);
      ctx.stroke();
    }

    // Vertical time gridlines (-20s .. 0s)
    const timeMarks = [20, 15, 10, 5, 0];
    for (const sec of timeMarks) {
      const x = padLeft + ((20 - sec) / 20) * liveW;
      if (sec !== 20 && sec !== 0) {
        ctx.strokeStyle = "#f1f5f9";
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(Math.round(x) + 0.5, padTop);
        ctx.lineTo(Math.round(x) + 0.5, padTop + plotH);
        ctx.stroke();
      }
    }

    // Optional right-hand active trajectory plan S-curve zone
    if (hasPlanCurve) {
      const splitX = padLeft + liveW;
      ctx.fillStyle = "rgba(99, 102, 241, 0.045)";
      ctx.fillRect(splitX, padTop, planW, plotH);
      ctx.strokeStyle = "rgba(99, 102, 241, 0.35)";
      ctx.setLineDash([4, 3]);
      ctx.beginPath();
      ctx.moveTo(splitX + 0.5, padTop);
      ctx.lineTo(splitX + 0.5, padTop + plotH);
      ctx.stroke();
      ctx.setLineDash([]);

      const dur = Math.max(0.01, activePlan.duration || 1.0);
      ctx.beginPath();
      ctx.strokeStyle = "#6366f1";
      ctx.lineWidth = 2.1;
      ctx.lineJoin = "round";
      planSamples.forEach((sp, i) => {
        const px = splitX + (Math.min(dur, sp.t) / dur) * (planW - 6) + 3;
        const py = toY(sp.q[selectedJointName]);
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      });
      ctx.stroke();

      // Current playback scrub marker on S-curve
      const curT = Math.min(dur, planScrub || 0);
      const sx = splitX + (curT / dur) * (planW - 6) + 3;
      const curSample = planSamples.reduce((best, sp) => (Math.abs(sp.t - curT) < Math.abs(best.t - curT) ? sp : best), planSamples[0]);
      const sy = toY(curSample.q[selectedJointName]);
      ctx.strokeStyle = "rgba(99, 102, 241, 0.45)";
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(sx, padTop);
      ctx.lineTo(sx, padTop + plotH);
      ctx.stroke();
      ctx.fillStyle = "#6366f1";
      ctx.beginPath();
      ctx.arc(sx, sy, 3.8, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 1.5;
      ctx.stroke();

      ctx.fillStyle = "#4f46e5";
      ctx.font = "600 9px ui-sans-serif, system-ui, sans-serif";
      ctx.fillText(`预演 +${dur.toFixed(1)}s`, splitX + 6, padTop + 11);
    }

    // Live curves (Target + Tracking Error Fill + Feedback Area Gradient)
    const nowTs = curveBuffer.length ? curveBuffer[curveBuffer.length - 1].ts : performance.now();
    const toX = (ts) => padLeft + Math.max(0, Math.min(1, 1 - (nowTs - ts) / CURVE_WINDOW_MS)) * liveW;

    if (curveBuffer.length >= 2) {
      const pts = curveBuffer.map(pt => ({
        x: toX(pt.ts),
        yPos: toY(pt.pos ?? 0),
        yTgt: toY(pt.target ?? 0),
        raw: pt,
      }));

      // 1. Subtle tracking-error ribbon between Target and Feedback
      ctx.beginPath();
      pts.forEach((p, i) => (i === 0 ? ctx.moveTo(p.x, p.yTgt) : ctx.lineTo(p.x, p.yTgt)));
      for (let i = pts.length - 1; i >= 0; i--) ctx.lineTo(pts[i].x, pts[i].yPos);
      ctx.closePath();
      ctx.fillStyle = "rgba(245, 158, 11, 0.10)";
      ctx.fill();

      // 2. Soft teal area gradient beneath Feedback curve
      const grad = ctx.createLinearGradient(0, padTop, 0, padTop + plotH);
      grad.addColorStop(0, "rgba(13, 148, 136, 0.18)");
      grad.addColorStop(1, "rgba(13, 148, 136, 0.01)");
      ctx.beginPath();
      ctx.moveTo(pts[0].x, padTop + plotH);
      pts.forEach(p => ctx.lineTo(p.x, p.yPos));
      ctx.lineTo(pts[pts.length - 1].x, padTop + plotH);
      ctx.closePath();
      ctx.fillStyle = grad;
      ctx.fill();

      // 3. Target curve (Crisp dashed warm amber)
      ctx.beginPath();
      ctx.strokeStyle = "#f59e0b";
      ctx.lineWidth = 1.8;
      ctx.lineJoin = "round";
      ctx.lineCap = "round";
      ctx.setLineDash([5, 4]);
      pts.forEach((p, i) => (i === 0 ? ctx.moveTo(p.x, p.yTgt) : ctx.lineTo(p.x, p.yTgt)));
      ctx.stroke();
      ctx.setLineDash([]);

      // 4. Feedback curve (Crisp solid Motphys teal)
      ctx.beginPath();
      ctx.strokeStyle = "#0d9488";
      ctx.lineWidth = 2.2;
      ctx.lineJoin = "round";
      ctx.lineCap = "round";
      pts.forEach((p, i) => (i === 0 ? ctx.moveTo(p.x, p.yPos) : ctx.lineTo(p.x, p.yPos)));
      ctx.stroke();

      // 5. Crisp live endpoint markers
      const lastP = pts[pts.length - 1];
      ctx.fillStyle = "#ffffff";
      ctx.strokeStyle = "#f59e0b";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(lastP.x, lastP.yTgt, 3.5, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();

      ctx.fillStyle = "#0d9488";
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 1.8;
      ctx.beginPath();
      ctx.arc(lastP.x, lastP.yPos, 4, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();

      // 6. Interactive hover crosshair + nearest-sample readout
      if (curveHoverX !== null && curveHoverX >= padLeft && curveHoverX <= padLeft + liveW) {
        let nearest = pts[0];
        let bestDist = Infinity;
        for (const p of pts) {
          const d = Math.abs(p.x - curveHoverX);
          if (d < bestDist) {
            bestDist = d;
            nearest = p;
          }
        }
        ctx.strokeStyle = "#94a3b8";
        ctx.lineWidth = 1;
        ctx.setLineDash([3, 3]);
        ctx.beginPath();
        ctx.moveTo(Math.round(nearest.x) + 0.5, padTop);
        ctx.lineTo(Math.round(nearest.x) + 0.5, padTop + plotH);
        ctx.stroke();
        ctx.setLineDash([]);

        ctx.fillStyle = "#0d9488";
        ctx.beginPath();
        ctx.arc(nearest.x, nearest.yPos, 4.5, 0, Math.PI * 2);
        ctx.fill();
        ctx.fillStyle = "#f59e0b";
        ctx.beginPath();
        ctx.arc(nearest.x, nearest.yTgt, 4, 0, Math.PI * 2);
        ctx.fill();

        if (curveTooltip) {
          const ageSec = Math.max(0, (nowTs - nearest.raw.ts) / 1000);
          const diff = Math.abs((nearest.raw.target ?? 0) - (nearest.raw.pos ?? 0));
          curveTooltip.hidden = false;
          curveTooltip.innerHTML =
            `<div class="ct-time">-${ageSec.toFixed(1)}s</div>` +
            `<div><i class="curve-dot curve-dot-fb"></i>反馈 <b>${(nearest.raw.pos ?? 0).toFixed(3)}</b></div>` +
            `<div><i class="curve-dot curve-dot-tgt"></i>目标 <b>${(nearest.raw.target ?? 0).toFixed(3)}</b></div>` +
            `<div><i class="curve-dot curve-dot-err"></i>Δe <b>${diff.toFixed(3)}</b></div>`;
          const flipLeft = nearest.x > padLeft + liveW - 125;
          curveTooltip.style.left = `${Math.round(flipLeft ? nearest.x - 118 : nearest.x + 10)}px`;
          curveTooltip.style.top = `${Math.max(8, Math.min(h - 86, Math.round(nearest.yPos - 26)))}px`;
        }
      } else if (curveTooltip) {
        curveTooltip.hidden = true;
      }
    } else {
      ctx.fillStyle = "#94a3b8";
      ctx.font = "500 12px ui-sans-serif, system-ui, sans-serif";
      ctx.textAlign = "center";
      ctx.fillText("正在采样实时关节波形…", padLeft + liveW * 0.5, padTop + plotH * 0.5 + 4);
      ctx.textAlign = "left";
    }
    ctx.restore();

    // Crisp Y-axis numeric labels (right-aligned in left gutter)
    ctx.fillStyle = "#64748b";
    ctx.font = "500 10px ui-monospace, SFMono-Regular, Menlo, monospace";
    ctx.textAlign = "right";
    for (let i = 0; i <= yTicks; i++) {
      const frac = i / yTicks;
      const val = maxV - frac * (maxV - minV);
      const y = padTop + frac * plotH;
      ctx.fillText(val.toFixed(2), padLeft - 6, y + 3.5);
    }

    // Crisp X-axis time labels
    ctx.textAlign = "center";
    ctx.fillStyle = "#94a3b8";
    for (const sec of timeMarks) {
      const x = padLeft + ((20 - sec) / 20) * liveW;
      const label = sec === 0 ? "Now" : `-${sec}s`;
      ctx.fillText(label, sec === 20 ? x + 10 : sec === 0 ? x - 8 : x, h - 7);
    }
    if (hasPlanCurve) {
      ctx.fillStyle = "#6366f1";
      ctx.fillText("S-Curve", padLeft + liveW + planW * 0.5, h - 7);
    }
    ctx.textAlign = "left";
  }

  // 3. UI State & Event Bindings
  const preview = new PosePreview();
  let localTargets = preview.targets;
  let currentPositions = {};
  let currentDescriptor = null;
  let activeEEArm = "left";

  const jointSelect = container.querySelector("#curveJointSelect");
  jointSelect.addEventListener("change", (e) => {
    selectedJointName = e.target.value;
    curveBuffer.length = 0;
  });

  let latestEEPoses = {};
  let ikInputsSeededFor = "";

  function populateIkInputsFromEE(ee) {
    if (!ee) return;
    container.querySelector("#ikX").value = Number(ee.x).toFixed(3);
    container.querySelector("#ikY").value = Number(ee.y).toFixed(3);
    container.querySelector("#ikZ").value = Number(ee.z).toFixed(3);
    container.querySelector("#ikRoll").value = Number(ee.roll_deg).toFixed(1);
    container.querySelector("#ikPitch").value = Number(ee.pitch_deg).toFixed(1);
    container.querySelector("#ikYaw").value = Number(ee.yaw_deg).toFixed(1);
  }

  async function runIkPreview(customPayload = null) {
    const dur = Math.max(0.4, Math.min(15, parseFloat(container.querySelector("#execDurationInput").value) || 1.4));
    const payload = customPayload || {
      arm: activeEEArm,
      xyz_m: [
        parseFloat(container.querySelector("#ikX").value) || 0,
        parseFloat(container.querySelector("#ikY").value) || 0,
        parseFloat(container.querySelector("#ikZ").value) || 0,
      ],
      rpy_deg: [
        parseFloat(container.querySelector("#ikRoll").value) || 0,
        parseFloat(container.querySelector("#ikPitch").value) || 0,
        parseFloat(container.querySelector("#ikYaw").value) || 0,
      ],
      seed_joints: { ...currentPositions },
      plan_trajectory: true,
      waypoints: 12,
      duration_s: dur,
    };
    try {
      const res = await api.solveIK(payload);
      const ik = res.ik;
      if (ik?.solved_joints) {
        for (const [k, v] of Object.entries(ik.solved_joints)) {
          preview.setTarget(k, Number(v));
        }
        preview.mode = "target";
        _renderSliders();
        refreshLocalModel();
      }
      if (res.plan) {
        setActivePlan(res.plan);
      }
      const errMm = (ik.pos_error_m * 1000).toFixed(2);
      notify(
        `URDF 6D IK (${activeEEArm}) ${ik.ok ? "收敛" : "接近限位"}: 位置误差 ${errMm}mm · 姿态误差 ${ik.rot_error_deg.toFixed(1)}°（已在 3D 视口预演，未下发）`,
        ik.ok ? "success" : "info"
      );
    } catch (err) {
      notify("IK 求解失败: " + err.message, "error");
    }
  }

  // End Effector Arm Switcher
  container.querySelector("#eeArmLeftBtn").addEventListener("click", () => {
    activeEEArm = "left";
    container.querySelector("#eeArmLeftBtn").style.background = "var(--accent-light)";
    container.querySelector("#eeArmLeftBtn").style.color = "var(--accent)";
    container.querySelector("#eeArmRightBtn").style.background = "";
    container.querySelector("#eeArmRightBtn").style.color = "";
    if (latestEEPoses.left) populateIkInputsFromEE(latestEEPoses.left);
  });
  container.querySelector("#eeArmRightBtn").addEventListener("click", () => {
    activeEEArm = "right";
    container.querySelector("#eeArmRightBtn").style.background = "var(--accent-light)";
    container.querySelector("#eeArmRightBtn").style.color = "var(--accent)";
    container.querySelector("#eeArmLeftBtn").style.background = "";
    container.querySelector("#eeArmLeftBtn").style.color = "";
    if (latestEEPoses.right) populateIkInputsFromEE(latestEEPoses.right);
  });

  container.querySelector("#syncFkBtn").addEventListener("click", async () => {
    try {
      const res = await api.getKinematicsFK(preview.visiblePose(), activeEEArm);
      const fk = res.fk?.[activeEEArm];
      if (fk) {
        populateIkInputsFromEE({
          x: fk.position[0], y: fk.position[1], z: fk.position[2],
          roll_deg: fk.rpy_deg[0], pitch_deg: fk.rpy_deg[1], yaw_deg: fk.rpy_deg[2],
        });
        notify(`已同步 ${activeEEArm} 臂真实 URDF FK 坐标`, "info");
      }
    } catch (err) {
      notify("读取 FK 失败: " + err.message, "error");
    }
  });

  container.querySelector("#solveIkBtn").addEventListener("click", () => runIkPreview());

  container.querySelectorAll("[data-ee-lin]").forEach(btn => {
    btn.addEventListener("click", async () => {
      const [axis, signStr] = btn.getAttribute("data-ee-lin").split(",");
      const step = parseFloat(container.querySelector("#eeStepSize").value) || 0.05;
      const mode = container.querySelector("#eeModeSelect").value;
      const delta = step * parseFloat(signStr);
      if (mode === "ik_preview") {
        const inputId = axis === "dx" ? "#ikX" : axis === "dy" ? "#ikY" : "#ikZ";
        const el = container.querySelector(inputId);
        el.value = ((parseFloat(el.value) || 0) + delta).toFixed(3);
        await runIkPreview();
        return;
      }
      const payload = { dx: 0, dy: 0, dz: 0 };
      payload[axis] = delta;
      try {
        await api.sendEEStep(activeEEArm, payload.dx, payload.dy, payload.dz);
      } catch (err) {
        notify("6D 平移点动失败: " + err.message, "error");
      }
    });
  });

  container.querySelectorAll("[data-ee-rot]").forEach(btn => {
    btn.addEventListener("click", async () => {
      const [axis, degStr] = btn.getAttribute("data-ee-rot").split(",");
      const mode = container.querySelector("#eeModeSelect").value;
      const deg = parseFloat(degStr);
      if (mode === "ik_preview") {
        const inputId = axis === "droll" ? "#ikRoll" : axis === "dpitch" ? "#ikPitch" : "#ikYaw";
        const el = container.querySelector(inputId);
        el.value = ((parseFloat(el.value) || 0) + deg).toFixed(1);
        await runIkPreview();
        return;
      }
      const payload = { droll: 0, dpitch: 0, dyaw: 0 };
      payload[axis] = deg;
      try {
        await api.sendEEStep(activeEEArm, 0, 0, 0, payload.droll, payload.dpitch, payload.dyaw);
      } catch (err) {
        notify("6D 姿态点动失败: " + err.message, "error");
      }
    });
  });

  function updateDiffBadges() {
    let diffCount = 0;
    const specMap = new Map((currentDescriptor?.joints || []).map(j => [j.name, j]));
    container.querySelectorAll(".joint-row").forEach(row => {
      const name = row.getAttribute("data-jname");
      const fb = currentPositions[name];
      const tgt = localTargets[name];
      const modified = Number.isFinite(fb) && Number.isFinite(tgt) && Math.abs(tgt - fb) > 0.005;
      row.classList.toggle("joint-row-modified", modified);
      if (modified) diffCount++;

      // Soft-limit proximity highlight (within 5% of range)
      const sp = specMap.get(name);
      const numEl = row.querySelector(".joint-input");
      if (sp && numEl && Number.isFinite(tgt)) {
        const margin = (sp.upper - sp.lower) * 0.05;
        const nearLimit = (tgt - sp.lower <= margin) || (sp.upper - tgt <= margin);
        numEl.style.borderColor = nearLimit ? "var(--warning)" : "";
      }
    });
    const badge = container.querySelector("#pendingDiffBadge");
    if (badge) {
      badge.textContent = diffCount > 0 ? `${diffCount} 项待下发` : "与反馈一致";
      badge.className = `badge ${diffCount > 0 ? "badge-info" : "badge-no_result"}`;
    }
  }

  function refreshLocalModel() {
    localTargets = preview.targets;
    container.querySelector("#poseViewMode").value = preview.mode;
    modelViewer.updatePose(preview.visiblePose(), preview.feedback, preview.mode);
    modelViewer.updateHandTargets(preview.handTargets);
    updateDiffBadges();
  }
  container.querySelector("#poseViewMode").addEventListener("change", event => {
    preview.mode = event.target.value === "feedback" ? "feedback" : "target";
    refreshLocalModel();
  });

  function renderHands() {
    const panel = container.querySelector("#handPreviewPanel");
    if (!preview.handSpecs.length) {
      panel.textContent = "该机型采用平行二指夹爪（已包含在上方 GRIPPER 关节组中，拖动即可实时预览双指对称开合）；不使用 W1 灵巧手替代。";
      return;
    }
    panel.innerHTML = `<div class="card-title">BrainCo Revo1 双灵巧手 · 仅本地预览</div><p class="preview-field-note">原始左右手 URDF / 24 STL、每手6个驱动关节及原始 mimic 联动。手指反馈未接入，不显示假反馈；不加入执行或保存的身体目标。</p>` + preview.handSpecs.map(hand => `
      <section class="hand-preview-group"><h4>${hand.side === "left" ? "左手" : "右手"} · ${hand.joints.length} DOF</h4>
      ${hand.joints.map(joint => `<label class="hand-preview-row"><span>${escapeHtml(joint.name)}</span><input type="range" min="${joint.lower}" max="${joint.upper}" step="0.01" value="${preview.handTargets[hand.side][joint.name]}" data-hand="${hand.side}" data-finger="${escapeHtml(joint.name)}" aria-label="${hand.side} ${escapeHtml(joint.name)} 本地预览"><output>${preview.handTargets[hand.side][joint.name].toFixed(2)} rad</output></label>`).join("")}</section>`).join("");
    panel.querySelectorAll("input[data-hand]").forEach(input => {
      bindLocalPreviewInput(input, value => preview.setHandTarget(input.dataset.hand, input.dataset.finger, value), () => {
        input.parentElement.querySelector("output").textContent = preview.handTargets[input.dataset.hand][input.dataset.finger].toFixed(2) + " rad";
        refreshLocalModel();
      });
    });
  }

  // Action Bar Buttons
  container.querySelector("#readCurrentBtn").addEventListener("click", () => {
    preview.copyFeedback();
    refreshLocalModel();
    _renderSliders();
    notify("已将当前 Mock 反馈关节角同步至目标滑块", "info");
  });

  container.querySelector("#homePresetBtn").addEventListener("click", () => {
    if (!currentDescriptor?.joints) return;
    for (const j of currentDescriptor.joints) {
      preview.setTarget(j.name, j.default ?? 0);
    }
    refreshLocalModel();
    _renderSliders();
    notify("已加载默认安全初始位到本地 3D 预览（尚未下发执行）", "info");
  });

  container.querySelector("#planPreviewBtn").addEventListener("click", async () => {
    const duration = Math.max(0.4, Math.min(15, parseFloat(container.querySelector("#execDurationInput").value) || 1.2));
    try {
      const res = await api.planJointTrajectory(preview.commandTargets(), currentPositions, 1.2, duration);
      if (!res.plan?.moving?.length) {
        notify("当前目标与反馈一致，无关节位移：请先拖动关节滑块或点击下方 6D IK 按钮设定目标后再预演。", "info");
        return;
      }
      setActivePlan(res.plan);
      notify(`已生成 ${res.plan.moving.length} 轴五次多项式轨迹预演（免 Lease、未运动，可点击「聚焦轨迹」拉近观察）`, "success");
    } catch (err) {
      notify("生成轨迹规划失败: " + err.message, "error");
    }
  });

  container.querySelector("#execTargetBtn").addEventListener("click", async () => {
    const duration = Math.max(0.2, Math.min(15, parseFloat(container.querySelector("#execDurationInput").value) || 1.2));
    try {
      const targets = preview.commandTargets();
      await api.sendJointTargets(targets, duration);
      setActivePlan(null);
      notify(`已下发 ${Object.keys(targets).length} 轴目标位姿 (时长 ${duration.toFixed(1)}s)`, "success");
    } catch (err) {
      notify("执行目标位姿失败: " + err.message, "error");
    }
  });

  // Save Atomic Action Native Dialog
  const savePoseDialog = container.querySelector("#savePoseDialog");
  enableDialogLightDismiss(savePoseDialog);
  container.querySelector("#closeSavePoseBtn").addEventListener("click", () => savePoseDialog.close("cancel"));

  container.querySelector("#saveActionBtn").addEventListener("click", () => {
    const targets = preview.commandTargets();
    const defaultName = `${(currentDescriptor?.robot_type || "pose")}_${new Date().toISOString().slice(11, 19).replace(/:/g, "")}`;
    container.querySelector("#savePoseNameInput").value = defaultName;
    container.querySelector("#savePoseSummary").textContent =
      `目标机型: ${currentDescriptor?.name || api.currentRobotId} · 身体关节数: ${Object.keys(targets).length} DOF（不含灵巧手本地预览字段）`;
    savePoseDialog.showModal();
  });

  container.querySelector("#confirmSavePoseBtn").addEventListener("click", async () => {
    const name = container.querySelector("#savePoseNameInput").value.trim();
    const duration_s = Math.max(0.2, parseFloat(container.querySelector("#savePoseDurationInput").value) || 1.4);
    const description = container.querySelector("#savePoseDescInput").value.trim() || "从关节控制面板保存的目标姿态";
    const tags = container.querySelector("#savePoseTagsInput").value.split(",").map(s => s.trim()).filter(Boolean);
    if (!name) {
      notify("请输入原子动作名称", "error");
      return;
    }
    try {
      await api.saveAction({
        name,
        robot_id: api.currentRobotId,
        kind: "joint_targets",
        description,
        tags,
        params: { targets: preview.commandTargets(), duration_s },
      });
      savePoseDialog.close("save");
      notify(`已保存身体原子动作 '${name}'，可在「动作与工作流」中选用`, "success");
    } catch (err) {
      notify("保存原子动作失败: " + err.message, "error");
    }
  });

  container.querySelector("#teachCheckBtn").addEventListener("click", async () => {
    try {
      const res = await api.post(`/api/robots/${encodeURIComponent(api.currentRobotId)}/teach/check`, {
        groups: ["left_arm", "right_arm"],
      });
      notify(`示教门禁检查通过: ${res.policy} (仅释放 ${res.groups.join(", ")})`, "success");
    } catch (err) {
      notify("示教限制检查失败: " + err.message, "error");
    }
  });

  // Support external "Preview in 3D" trigger from Atomic Actions tab (pure local preview, zero API write!)
  const onPreviewAction = (e) => {
    const { action } = e.detail || {};
    if (!action || action.kind !== "joint_targets" || !action.params?.targets) return;
    let applied = 0;
    for (const [k, v] of Object.entries(action.params.targets)) {
      if (preview.setTarget(k, Number(v))) applied++;
    }
    if (applied > 0) {
      preview.mode = "target";
      setActivePlan(null);
      refreshLocalModel();
      _renderSliders();
      notify(`已在 3D 视图本地预览动作 '${action.name}' (${applied} 关节 · 未下发执行)`, "info");
    }
  };
  window.addEventListener("motphys:preview-action", onPreviewAction);

  // Support external "3D Trajectory Preview" trigger from Workflows tab (ghost + neon trail + parameter context!)
  const onPreviewPlan = (e) => {
    const { plan, workflow_context } = e.detail || {};
    if (!plan) return;
    if (wfSelect && plan.workflow_id) wfSelect.value = plan.workflow_id;
    const ctxKeys = Object.keys(workflow_context || {});
    const ctxSummary = ctxKeys.length
      ? `参数传递上下文: ${ctxKeys.map(k => `${k}=${JSON.stringify(workflow_context[k]?.xyz_m || workflow_context[k])}`).join(" · ")}`
      : "";
    setActivePlan(plan, ctxSummary, { autoFocus: true });
  };
  window.addEventListener("motphys:preview-plan", onPreviewPlan);

  function _renderSliders() {
    if (!currentDescriptor) return;
    const slidersWrap = container.querySelector("#jointsSlidersContainer");
    const joints = currentDescriptor.joints || [];

    const groups = {};
    for (const j of joints) {
      groups[j.group] = groups[j.group] || [];
      groups[j.group].push(j);
    }

    slidersWrap.innerHTML = Object.entries(groups).map(([grp, jList]) => `
      <div style="margin-bottom:12px;">
        <div style="font-size:11px; font-weight:700; color:var(--text-muted); text-transform:uppercase; letter-spacing:0.04em; margin-bottom:4px; padding-bottom:2px; border-bottom:1px solid var(--border);">
          ${escapeHtml(grp.toUpperCase())} (${jList.length})
        </div>
        ${jList.map(j => {
          const curVal = (currentPositions[j.name] !== undefined ? currentPositions[j.name] : j.default) || 0;
          const tgtVal = (localTargets[j.name] !== undefined ? localTargets[j.name] : curVal) || 0;
          const step = j.is_gripper ? 0.05 : 0.02;
          return `
            <div class="joint-row" data-jname="${escapeHtml(j.name)}">
              <span class="joint-label" title="${escapeHtml(j.label)} [${j.lower}, ${j.upper}]">${escapeHtml(j.label)}</span>
              <input type="range" class="joint-slider" min="${j.lower}" max="${j.upper}" step="${step}" value="${tgtVal}">
              <span class="joint-val-box" title="当前 Mock 反馈（不是目标预览）">${Number.isFinite(currentPositions[j.name]) ? currentPositions[j.name].toFixed(2) : "—"}</span>
              <input type="number" class="joint-input" min="${j.lower}" max="${j.upper}" step="${step}" value="${tgtVal.toFixed(2)}">
            </div>
          `;
        }).join("")}
      </div>
    `).join("");

    slidersWrap.querySelectorAll(".joint-row").forEach(row => {
      const name = row.getAttribute("data-jname");
      const slider = row.querySelector(".joint-slider");
      const numInput = row.querySelector(".joint-input");

      bindLocalPreviewInput(slider, value => preview.setTarget(name, value), () => {
        numInput.value = preview.targets[name].toFixed(2);
        refreshLocalModel();
      });
      bindLocalPreviewInput(numInput, value => preview.setTarget(name, value), () => {
        slider.value = preview.targets[name];
        refreshLocalModel();
      });
    });
    updateDiffBadges();
  }

  return {
    update(state) {
      if (!state) return;
      const desc = state.descriptor || {};
      const obs = state.observation || {};
      const motrix = state.motrix || {};

      // Sync quick robot switcher active pill
      container.querySelectorAll("[data-quick-robot]").forEach(btn => {
        btn.classList.toggle("active", btn.dataset.quickRobot === desc.id);
      });

      if (!currentDescriptor || currentDescriptor.id !== desc.id) {
        const switchingRobot = Boolean(currentDescriptor && currentDescriptor.id !== desc.id);
        currentDescriptor = desc;
        if (switchingRobot) setActivePlan(null);
        ikInputsSeededFor = "";
        preview.selectRobot(desc, obs, modelForDescriptor(desc)?.hands || []);
        localTargets = preview.targets;
        currentPositions = preview.feedback;
        modelViewer.select(desc);
        refreshLocalModel();
        renderHands();

        jointSelect.innerHTML = (desc.joints || []).map(j => `
          <option value="${escapeHtml(j.name)}" ${j.name === selectedJointName ? "selected" : ""}>${escapeHtml(j.label)}</option>
        `).join("");
        if (desc.joints && desc.joints.length > 0 && !desc.joints.some(j => j.name === selectedJointName)) {
          selectedJointName = desc.joints[0].name;
          curveBuffer.length = 0;
        }

        const wsPath = container.querySelector("#motrixWsPath");
        if (wsPath) wsPath.textContent = `/ws/motrix?robot_id=${desc.id}`;

        container.querySelector("#eeCard").style.display = desc.capabilities.ee_6d ? "block" : "none";
        const armsScope = Array.isArray(desc.arms_scope) && desc.arms_scope.length ? desc.arms_scope : ["left", "right"];
        const leftArmBtn = container.querySelector("#eeArmLeftBtn");
        const rightArmBtn = container.querySelector("#eeArmRightBtn");
        if (leftArmBtn) leftArmBtn.disabled = !armsScope.includes("left");
        if (rightArmBtn) rightArmBtn.disabled = !armsScope.includes("right");
        if (!armsScope.includes(activeEEArm)) {
          activeEEArm = armsScope[0];
          if (leftArmBtn) {
            leftArmBtn.style.background = activeEEArm === "left" ? "var(--accent-light)" : "";
            leftArmBtn.style.color = activeEEArm === "left" ? "var(--accent)" : "";
          }
          if (rightArmBtn) {
            rightArmBtn.style.background = activeEEArm === "right" ? "var(--accent-light)" : "";
            rightArmBtn.style.color = activeEEArm === "right" ? "var(--accent)" : "";
          }
        }
        _renderSliders();
        refreshWorkflowPicker();
      }

      preview.observe(desc.id, obs.joint_positions || {});
      currentPositions = preview.feedback;

      refreshLocalModel();

      // Update Motrix Mirror info
      const motrixCountEl = container.querySelector("#motrixClientsCount");
      if (motrixCountEl) motrixCountEl.textContent = motrix.connected_clients !== undefined ? motrix.connected_clients : (obs.motrix_mirror_clients || 0);
      const motrixRejEl = container.querySelector("#motrixRejectedCount");
      if (motrixRejEl) motrixRejEl.textContent = motrix.rejected_write_attempts ?? 0;
      const motrixIsoBadge = container.querySelector("#motrixIsoBadge");
      if (motrixIsoBadge) motrixIsoBadge.textContent = motrix.isolated ? "Isolation: Verified (0 drivers)" : "Driver Loaded";

      // Update 20s curve (real timestamped 20-second sliding window)
      if (!curvePaused) {
        const nowTs = performance.now();
        const pVal = currentPositions[selectedJointName] || 0;
        const tVal = localTargets[selectedJointName] ?? pVal;
        curveBuffer.push({ ts: nowTs, pos: pVal, target: tVal });
        const cutoff = nowTs - CURVE_WINDOW_MS - 500;
        while (curveBuffer.length > 2 && curveBuffer[0].ts < cutoff) curveBuffer.shift();
        if (curveBuffer.length > 360) curveBuffer.shift();
      }
      drawCurve();

      // Update EE pose readout & seed IK inputs once per robot/arm
      if (obs.ee_poses) {
        latestEEPoses = obs.ee_poses;
        const ee = obs.ee_poses[activeEEArm];
        if (ee) {
          const eeDisplay = container.querySelector("#eePoseDisplay");
          if (eeDisplay) {
            eeDisplay.textContent =
              `[${activeEEArm.toUpperCase()} · ${ee.frame || "base_link"}] Pos: x=${ee.x.toFixed(3)} y=${ee.y.toFixed(3)} z=${ee.z.toFixed(3)} | RPY: ${ee.roll_deg.toFixed(1)}° ${ee.pitch_deg.toFixed(1)}° ${ee.yaw_deg.toFixed(1)}°`;
          }
          if (ikInputsSeededFor !== `${desc.id}:${activeEEArm}`) {
            ikInputsSeededFor = `${desc.id}:${activeEEArm}`;
            populateIkInputsFromEE(ee);
          }
        }
      }

      // Sync slider current readings
      container.querySelectorAll(".joint-row").forEach(row => {
        const name = row.getAttribute("data-jname");
        if (currentPositions[name] !== undefined) {
          const curBox = row.querySelector(".joint-val-box");
          if (curBox) curBox.textContent = currentPositions[name].toFixed(2);
        }
      });
    },
    destroy() {
      if (animId) cancelAnimationFrame(animId);
      window.removeEventListener("resize", onResize);
      window.removeEventListener("motphys:robots-updated", onRobotsUpdated);
      window.removeEventListener("motphys:preview-action", onPreviewAction);
      window.removeEventListener("motphys:preview-plan", onPreviewPlan);
      resizeObserver.disconnect();
      curveResizeObserver.disconnect();
      modelViewer.destroy();
      controls.dispose();
      renderer.dispose();
    },
  };
}
