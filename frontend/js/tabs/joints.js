// Tab 2: Joints, 3D Visualization, 6D EE Step & Motrix Read-Only Mirror
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { api } from "../api.js";
import { createRobotModelViewer } from "../robot-model-viewer.js";
import { modelForDescriptor } from "../model-registry.js";
import { PosePreview, bindLocalPreviewInput } from "../pose-preview.js";

export function initJointsTab(container) {
  container.innerHTML = `
    <div class="card pose-preview-toolbar">
      <label for="poseViewMode">身体 3D 姿态来源</label>
      <select id="poseViewMode" class="select-clean"><option value="target">本地目标预览 · 拖动立即显示</option><option value="feedback">当前反馈姿态 · Mock 反馈来源</option></select>
      <span id="poseModeNote">拖动仅改变本地模型，不申请 Lease、不发送动作；反馈读数独立保留。</span>
    </div>
    <div class="grid-split-view">
      <!-- Left Column: 3D Viewport & Motrix Mirror Status -->
      <div>
        <div class="viewport-3d-box" id="viewportBox">
          <canvas class="viewport-3d-canvas" id="viewportCanvas"></canvas>
          <div class="viewport-overlay-hud" id="viewportHud">
            本地只读模型视图 · 等待机型选择（不加载硬件驱动）
          </div>
        </div>

        <!-- Motrix WASM Read-Only Mirror Status Box -->
        <div class="card" style="margin-top: 14px;">
          <div class="card-header" style="margin-bottom:6px; padding-bottom:6px;">
            <span class="card-title" style="font-size:13px;">Motrix 只读消息桥（WASM 仿真器未接入）</span>
            <span class="badge badge-passed" id="motrixIsoBadge">本地协议隔离</span>
          </div>
          <div style="font-size:12px; color:var(--text-muted); display:flex; justify-content:space-between; margin-bottom:6px;">
            <span>WebSocket 地址: <code style="font-family:var(--font-mono); color:var(--text-main);">/ws/motrix</code></span>
            <span>已连接查看器: <b id="motrixClientsCount">0</b></span>
          </div>
          <p style="font-size:11px; color:#64748b; background:var(--bg-subtle); padding:6px 10px; border-radius:var(--radius-sm);">
            严格只读隔离断言：本镜像桥零加载底层 CAN/ROS 写驱动，绝不通过 WebSocket 反向写控机器人，仅提供 Observation 广播协议；尚未接入 Motrix WASM 客户端。
          </p>
        </div>

        <!-- 20-Second Live Trajectory Curve Canvas -->
        <div class="card" style="margin-top: 14px;">
          <div class="card-header" style="margin-bottom:6px; padding-bottom:6px;">
            <span class="card-title" style="font-size:13px;">实时关节轨迹曲线 (20s Live Curve)</span>
            <select class="select-clean" id="curveJointSelect" style="padding:2px 8px; font-size:12px;"></select>
          </div>
          <canvas id="curveCanvas" width="600" height="140" style="width:100%; height:140px; background:#0f172a; border-radius:var(--radius-md); display:block;"></canvas>
        </div>
      </div>

      <!-- Right Column: Grouped Joint Sliders & 6D EE Controls -->
      <div>
        <!-- Action Toolbar -->
        <div class="card" style="padding:12px 16px; margin-bottom:12px;">
          <div style="display:flex; flex-wrap:wrap; gap:8px; align-items:center; justify-content:space-between;">
            <div style="display:flex; gap:8px;">
              <button class="btn btn-sm" id="readCurrentBtn">读当前到目标</button>
              <button class="btn btn-sm btn-primary" id="execTargetBtn">执行到 Mock（需 Lease）</button>
            </div>
            <div style="display:flex; gap:8px;">
              <button class="btn btn-sm" id="saveActionBtn">保存身体原子动作</button>
              <button class="btn btn-sm" id="teachCheckBtn">示教仅释放双臂</button>
            </div>
          </div>
        </div>

        <!-- 6D End-Effector Step Panel (Dynamic) -->
        <div class="card" id="eeCard" style="margin-bottom:12px;">
          <div class="card-header" style="margin-bottom:8px; padding-bottom:6px;">
            <span class="card-title" style="font-size:13px;">末端 6D 点动控制 (base_link)</span>
            <div style="display:flex; gap:6px;">
              <button class="btn btn-sm" id="eeArmLeftBtn" style="background:var(--accent-light); color:var(--accent);">左臂 Left</button>
              <button class="btn btn-sm" id="eeArmRightBtn">右臂 Right</button>
            </div>
          </div>
          <div style="display:grid; grid-template-columns: repeat(3, 1fr); gap:6px; font-size:12px;">
            <button class="btn btn-sm" data-ee="dx,0.02">+X (前 2cm)</button>
            <button class="btn btn-sm" data-ee="dx,-0.02">-X (后 2cm)</button>
            <button class="btn btn-sm" data-ee="dz,0.02">+Z (上 2cm)</button>
            <button class="btn btn-sm" data-ee="dy,0.02">+Y (左 2cm)</button>
            <button class="btn btn-sm" data-ee="dy,-0.02">-Y (右 2cm)</button>
            <button class="btn btn-sm" data-ee="dz,-0.02">-Z (下 2cm)</button>
          </div>
          <div style="margin-top:8px; font-family:var(--font-mono); font-size:11px; color:var(--text-muted); background:var(--bg-subtle); padding:4px 8px; border-radius:var(--radius-sm);" id="eePoseDisplay">
            Pos: x=0.000 y=0.000 z=0.000 | RPY: 0.0° 0.0° 0.0°
          </div>
        </div>

        <!-- Joint Sliders Container (Grouped) -->
        <div class="card">
          <div class="card-header" style="margin-bottom:8px; padding-bottom:6px;">
            <span class="card-title" style="font-size:13px;">关节控制与限位表 (Joints)</span>
            <span style="font-size:11px; color:var(--text-muted);">单位: rad / norm</span>
          </div>
          <p class="preview-field-note">滑块 / 数值：本地目标预览；中间读数：当前 Mock 反馈。预览≠执行。</p>
          <div id="jointsSlidersContainer" style="max-height: 480px; overflow-y: auto; padding-right: 4px;">
            <div style="text-align:center; color:#94a3b8; padding:20px;">加载关节定义...</div>
          </div>
        </div>
        <div id="handPreviewPanel" class="card" style="margin-top:14px;"></div>
      </div>
    </div>
  `;

  // 1. Initialize Three.js 3D Viewport
  const canvas = container.querySelector("#viewportCanvas");
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
  renderer.setSize(canvas.clientWidth || 500, canvas.clientHeight || 350, false);

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(34, (canvas.clientWidth || 500) / (canvas.clientHeight || 350), 0.1, 50);
  camera.position.set(1.4, 1.8, 1.9);

  const controls = new OrbitControls(camera, canvas);
  controls.enableDamping = true;
  controls.target.set(0, 0.45, 0);

  // Lights & Grid
  const hemi = new THREE.HemisphereLight(0xdbeafe, 0x1e293b, 1.2);
  scene.add(hemi);
  const dir = new THREE.DirectionalLight(0xffffff, 2.0);
  dir.position.set(2, 4, 3);
  scene.add(dir);
  const grid = new THREE.GridHelper(2.4, 24, 0x0d9488, 0x334155);
  scene.add(grid);

  const modelViewer = createRobotModelViewer({scene, camera, controls, hud:container.querySelector("#viewportHud")});

  // Resize handler for 3D canvas
  function onResize() {
    const box = container.querySelector("#viewportBox");
    if (!box) return;
    const w = box.clientWidth;
    const h = box.clientHeight;
    if (w > 0 && h > 0) {
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      renderer.setSize(w, h, false);
    }
  }
  window.addEventListener("resize", onResize);
  const resizeObserver = new ResizeObserver(onResize);
  resizeObserver.observe(container.querySelector("#viewportBox"));
  setTimeout(onResize, 100);

  // 3D Render Loop
  let animId = null;
  function animate() {
    animId = requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
  }
  animate();

  // 2. Trajectory 20s Live Curve buffer
  const curveBuffer = [];
  let selectedJointName = "left_arm_1";
  const curveCanvas = container.querySelector("#curveCanvas");
  const curveCtx = curveCanvas.getContext("2d");

  function drawCurve() {
    const w = curveCanvas.width;
    const h = curveCanvas.height;
    curveCtx.clearRect(0, 0, w, h);

    // Grid lines
    curveCtx.strokeStyle = "#1e293b";
    curveCtx.lineWidth = 1;
    for (let y = 20; y < h; y += 30) {
      curveCtx.beginPath();
      curveCtx.moveTo(0, y);
      curveCtx.lineTo(w, y);
      curveCtx.stroke();
    }

    if (curveBuffer.length < 2) return;

    // Draw target line (dashed amber)
    curveCtx.beginPath();
    curveCtx.strokeStyle = "rgba(245, 158, 11, 0.7)";
    curveCtx.setLineDash([4, 4]);
    curveCtx.lineWidth = 1.5;
    for (let i = 0; i < curveBuffer.length; i++) {
      const pt = curveBuffer[i];
      const x = (i / (curveBuffer.length - 1)) * w;
      const y = h / 2 - (pt.target || 0) * 35;
      if (i === 0) curveCtx.moveTo(x, y);
      else curveCtx.lineTo(x, y);
    }
    curveCtx.stroke();
    curveCtx.setLineDash([]);

    // Draw position line (solid teal)
    curveCtx.beginPath();
    curveCtx.strokeStyle = "#14b8a6";
    curveCtx.lineWidth = 2;
    for (let i = 0; i < curveBuffer.length; i++) {
      const pt = curveBuffer[i];
      const x = (i / (curveBuffer.length - 1)) * w;
      const y = h / 2 - (pt.pos || 0) * 35;
      if (i === 0) curveCtx.moveTo(x, y);
      else curveCtx.lineTo(x, y);
    }
    curveCtx.stroke();

    // Latest value label
    const last = curveBuffer[curveBuffer.length - 1];
    curveCtx.fillStyle = "#e2e8f0";
    curveCtx.font = "11px monospace";
    curveCtx.fillText(`Pos: ${(last.pos || 0).toFixed(3)} rad | Target: ${(last.target || 0).toFixed(3)} rad`, 10, 16);
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

  // End Effector Arm Switcher
  container.querySelector("#eeArmLeftBtn").addEventListener("click", () => {
    activeEEArm = "left";
    container.querySelector("#eeArmLeftBtn").style.background = "var(--accent-light)";
    container.querySelector("#eeArmLeftBtn").style.color = "var(--accent)";
    container.querySelector("#eeArmRightBtn").style.background = "";
    container.querySelector("#eeArmRightBtn").style.color = "";
  });
  container.querySelector("#eeArmRightBtn").addEventListener("click", () => {
    activeEEArm = "right";
    container.querySelector("#eeArmRightBtn").style.background = "var(--accent-light)";
    container.querySelector("#eeArmRightBtn").style.color = "var(--accent)";
    container.querySelector("#eeArmLeftBtn").style.background = "";
    container.querySelector("#eeArmLeftBtn").style.color = "";
  });

  // EE Step Buttons
  container.querySelectorAll("[data-ee]").forEach(btn => {
    btn.addEventListener("click", async () => {
      const [axis, deltaStr] = btn.getAttribute("data-ee").split(",");
      const delta = parseFloat(deltaStr);
      const payload = { dx: 0, dy: 0, dz: 0 };
      payload[axis] = delta;
      try {
        await api.sendEEStep(activeEEArm, payload.dx, payload.dy, payload.dz);
      } catch (err) {
        alert("6D Step error: " + err.message);
      }
    });
  });

  function refreshLocalModel() {
    localTargets = preview.targets;
    container.querySelector("#poseViewMode").value = preview.mode;
    modelViewer.updatePose(preview.visiblePose());
    modelViewer.updateHandTargets(preview.handTargets);
  }
  container.querySelector("#poseViewMode").addEventListener("change", event => {
    preview.mode = event.target.value === "feedback" ? "feedback" : "target";
    refreshLocalModel();
  });

  function renderHands() {
    const panel = container.querySelector("#handPreviewPanel");
    if (!preview.handSpecs.length) { panel.textContent = "该机型未提供对应手部模型；不使用 W1 灵巧手替代。夹爪字段若存在，仍在该机型关节组中。"; return; }
    panel.innerHTML = `<div class="card-title">BrainCo Revo1 双灵巧手 · 仅本地预览</div><p class="preview-field-note">原始左右手 URDF / 24 STL、每手6个驱动关节及原始 mimic 联动。手指反馈未接入，不显示假反馈；不加入执行或保存的身体目标。</p>` + preview.handSpecs.map(hand => `
      <section class="hand-preview-group"><h4>${hand.side === "left" ? "左手" : "右手"} · ${hand.joints.length} DOF</h4>
      ${hand.joints.map(joint => `<label class="hand-preview-row"><span>${joint.name}</span><input type="range" min="${joint.lower}" max="${joint.upper}" step="0.01" value="${preview.handTargets[hand.side][joint.name]}" data-hand="${hand.side}" data-finger="${joint.name}" aria-label="${hand.side} ${joint.name} 本地预览"><output>${preview.handTargets[hand.side][joint.name].toFixed(2)} rad</output></label>`).join("")}</section>`).join("");
    panel.querySelectorAll("input[data-hand]").forEach(input => {
      bindLocalPreviewInput(input, value => preview.setHandTarget(input.dataset.hand,input.dataset.finger,value), () => {
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
  });

  container.querySelector("#execTargetBtn").addEventListener("click", async () => {
    try {
      await api.sendJointTargets(preview.commandTargets(), 1.2);
    } catch (err) {
      alert("执行目标位姿失败: " + err.message);
    }
  });

  container.querySelector("#saveActionBtn").addEventListener("click", async () => {
    const name = prompt("请输入新原子动作名称:", `pose_${new Date().toISOString().slice(11, 19).replace(/:/g, "")}`);
    if (!name) return;
    try {
      await api.saveAction({
        name,
        robot_id: api.currentRobotId,
        kind: "joint_targets",
        description: `从关节控制面板保存的目标姿态`,
        params: { targets: preview.commandTargets(), duration_s: 1.4 },
      });
      alert(`已保存原子动作 '${name}'`);
    } catch (err) {
      alert("保存原子动作失败: " + err.message);
    }
  });

  container.querySelector("#teachCheckBtn").addEventListener("click", async () => {
    try {
      const res = await api.post(`/api/robots/${api.currentRobotId}/teach/check`, {
        groups: ["left_arm", "right_arm"],
      });
      alert(`示教检查通过: ${res.policy} (仅释放 ${res.groups.join(", ")})`);
    } catch (err) {
      alert("示教限制检查失败: " + err.message);
    }
  });

  function _renderSliders() {
    if (!currentDescriptor) return;
    const slidersWrap = container.querySelector("#jointsSlidersContainer");
    const joints = currentDescriptor.joints || [];

    // Group joints by group
    const groups = {};
    for (const j of joints) {
      groups[j.group] = groups[j.group] || [];
      groups[j.group].push(j);
    }

    slidersWrap.innerHTML = Object.entries(groups).map(([grp, jList]) => `
      <div style="margin-bottom:12px;">
        <div style="font-size:11px; font-weight:700; color:var(--text-muted); text-transform:uppercase; letter-spacing:0.04em; margin-bottom:4px; padding-bottom:2px; border-bottom:1px solid var(--border);">
          ${grp.toUpperCase()}
        </div>
        ${jList.map(j => {
          const curVal = (currentPositions[j.name] !== undefined ? currentPositions[j.name] : j.default) || 0;
          const tgtVal = (localTargets[j.name] !== undefined ? localTargets[j.name] : curVal) || 0;
          const step = j.is_gripper ? 0.05 : 0.02;
          return `
            <div class="joint-row" data-jname="${j.name}">
              <span class="joint-label" title="${j.label}">${j.label}</span>
              <input type="range" class="joint-slider" min="${j.lower}" max="${j.upper}" step="${step}" value="${tgtVal}">
              <span class="joint-val-box" title="当前 Mock 反馈（不是目标预览）">${Number.isFinite(currentPositions[j.name]) ? currentPositions[j.name].toFixed(2) : "—"}</span>
              <input type="number" class="joint-input" min="${j.lower}" max="${j.upper}" step="${step}" value="${tgtVal.toFixed(2)}">
            </div>
          `;
        }).join("")}
      </div>
    `).join("");

    // Slider inputs live sync
    slidersWrap.querySelectorAll(".joint-row").forEach(row => {
      const name = row.getAttribute("data-jname");
      const slider = row.querySelector(".joint-slider");
      const numInput = row.querySelector(".joint-input");

      bindLocalPreviewInput(slider, value => preview.setTarget(name,value), () => {
        numInput.value = preview.targets[name].toFixed(2);
        refreshLocalModel();
      });
      bindLocalPreviewInput(numInput, value => preview.setTarget(name,value), () => {
        slider.value = preview.targets[name];
        refreshLocalModel();
      });
    });
  }

  return {
    update(state) {
      if (!state) return;
      const desc = state.descriptor || {};
      const obs = state.observation || {};
      const motrix = state.motrix || {};

      // If descriptor changed, update curve select & slider list
      if (!currentDescriptor || currentDescriptor.id !== desc.id) {
        currentDescriptor = desc;
        preview.selectRobot(desc,obs,modelForDescriptor(desc)?.hands || []);
        localTargets = preview.targets;
        currentPositions = preview.feedback;
        modelViewer.select(desc);
        refreshLocalModel();
        renderHands();

        jointSelect.innerHTML = (desc.joints || []).map(j => `
          <option value="${j.name}" ${j.name === selectedJointName ? "selected" : ""}>${j.label}</option>
        `).join("");
        if (desc.joints && desc.joints.length > 0 && !desc.joints.some(j => j.name === selectedJointName)) {
          selectedJointName = desc.joints[0].name;
        }

        // Hide EE card if not supported
        container.querySelector("#eeCard").style.display = desc.capabilities.ee_6d ? "block" : "none";
        _renderSliders();
      }

      preview.observe(desc.id,obs.joint_positions || {});
      currentPositions = preview.feedback;

      // Update 3D Visual Kinematic Poses
      refreshLocalModel();

      // Update Motrix Mirror info
      const motrixCountEl = container.querySelector("#motrixClientsCount");
      if (motrixCountEl) motrixCountEl.textContent = motrix.connected_clients !== undefined ? motrix.connected_clients : (obs.motrix_mirror_clients || 0);
      const motrixIsoBadge = container.querySelector("#motrixIsoBadge");
      if (motrixIsoBadge) motrixIsoBadge.textContent = motrix.isolated ? "Isolation: Verified (0 drivers)" : "Driver Loaded";

      // Update 20s curve
      const pVal = currentPositions[selectedJointName] || 0;
      const tVal = localTargets[selectedJointName] ?? pVal;
      curveBuffer.push({ pos: pVal, target: tVal });
      if (curveBuffer.length > 120) curveBuffer.shift();
      drawCurve();

      // Update EE pose readout if present
      if (obs.ee_poses && obs.ee_poses[activeEEArm]) {
        const ee = obs.ee_poses[activeEEArm];
        const eeDisplay = container.querySelector("#eePoseDisplay");
        if (eeDisplay) {
          eeDisplay.textContent =
            `Pos: x=${ee.x.toFixed(3)} y=${ee.y.toFixed(3)} z=${ee.z.toFixed(3)} | RPY: ${ee.roll_deg.toFixed(1)}° ${ee.pitch_deg.toFixed(1)}° ${ee.yaw_deg.toFixed(1)}°`;
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
      resizeObserver.disconnect();
      modelViewer.destroy();
      controls.dispose();
      renderer.dispose();
    }
  };
}
