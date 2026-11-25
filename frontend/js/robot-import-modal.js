// Motphys Robot Web — Dynamic Robot Package Import & 6-Gate Preflight Wizard
import { api } from "./api.js";
import { escapeHtml } from "./ui-format.js";
import { notify, enableDialogLightDismiss } from "./notify.js";

const CAP_ITEMS = [
  ["ee_6d", "末端 6D IK / 轨迹"],
  ["navigation", "2D / 3D 导航地图"],
  ["chassis_teleop", "底盘手动微调"],
  ["cameras", "相机视频工作台"],
  ["head_depth", "头部在线深度"],
  ["yolo_foundation_pose", "YOLO + FoundationPose"],
  ["aruco", "ArUco 对准"],
  ["policy_inference", "策略推理 / 切模"],
  ["offline_replay", "离线回放评测"],
];

const JOINT_GROUPS = ["left_arm", "right_arm", "torso", "head", "gripper", "leg"];

export function initRobotImportModal({ onRobotsChanged = async () => {} } = {}) {
  const existing = document.querySelector("#robotImportDialog");
  if (existing) existing.remove();

  const dialog = document.createElement("dialog");
  dialog.id = "robotImportDialog";
  dialog.className = "robot-import-dialog";
  dialog.setAttribute("closedby", "any");
  dialog.setAttribute("aria-labelledby", "robotImportTitle");
  dialog.innerHTML = `
    <div class="dialog-content robot-import-content">
      <div class="dialog-header">
        <div>
          <h1 class="dialog-title" id="robotImportTitle">导入新机器人（标准 Robot Package 向导）</h1>
          <p class="import-subtitle">上传含 URDF + 网格贴图（+ 可选 robot_manifest.json / maps）的 .zip 包，经 6 项入库预检后即刻启用 3D、IK 与 Mock 仿真。</p>
        </div>
        <button type="button" class="drawer-close-btn" id="btnCloseRobotImport" aria-label="关闭导入向导">✕</button>
      </div>

      <div class="import-body">
        <!-- Step 1: Upload Package -->
        <section class="import-section">
          <div class="import-section-head">
            <span class="import-step-num">1</span>
            <strong>上传机器人资产包 (.zip)</strong>
            <div style="margin-left:auto; display:flex; gap:6px; flex-wrap:wrap;">
              <a class="btn btn-sm" href="/api/robots/import/template" download="motphys_robot_template.zip">⬇ 下载标准模板 (.zip)</a>
              <button type="button" class="btn btn-sm btn-import-sample" id="btnLoadSampleRobot">⚡ 一键加载示例 6-DOF 机械臂包</button>
            </div>
          </div>

          <div class="import-dropzone" id="importDropzone" tabindex="0" role="button" aria-label="点击或拖拽上传 .zip 机器人包">
            <input type="file" id="importZipFileInput" accept=".zip,application/zip" hidden>
            <div class="import-dropzone-title">点击选择或拖拽 <code>.zip</code> 机器人包到此处</div>
            <div class="import-dropzone-hint">支持 <code>urdf/robot.urdf + meshes/*.glb|obj|stl</code>（无 <code>robot_manifest.json</code> 时自动解析关节、限位与运动学链）</div>
          </div>
          <div id="importPackageSummary" class="import-pkg-summary" hidden></div>
        </section>

        <!-- Step 2: Configuration & SDK Alignment -->
        <section class="import-section" id="importConfigSection" hidden>
          <div class="import-section-head">
            <span class="import-step-num">2</span>
            <strong>构型配比、运动学链与 SDK 协议对齐</strong>
            <span style="margin-left:auto; font-size:11px; color:var(--text-muted);">修改任意字段后点击「重新预检」验证</span>
          </div>

          <div class="import-form-grid">
            <div class="form-group">
              <label class="form-label" for="impRobotId">机器人 ID (唯一标识)</label>
              <input type="text" class="form-input" id="impRobotId" placeholder="e.g. unitree-g1-sim">
            </div>
            <div class="form-group">
              <label class="form-label" for="impRobotName">显示名称</label>
              <input type="text" class="form-input" id="impRobotName" placeholder="e.g. Unitree G1 Dual-Arm">
            </div>
            <div class="form-group">
              <label class="form-label" for="impRobotType">机型类别 (robot_type)</label>
              <input type="text" class="form-input" id="impRobotType" placeholder="e.g. unitree_g1">
            </div>
            <div class="form-group">
              <label class="form-label" for="impConnectorType">运行态连接器 (Adapter)</label>
              <select class="form-input" id="impConnectorType">
                <option value="mock">Mock 本地仿真（上传即用）</option>
                <option value="ros2_bridge">ROS 2 Bridge (rosbridge WebSocket · 零 rclpy 接入)</option>
                <option value="edge">EdgeConnector (motrix-edge /v1 推荐主线)</option>
                <option value="http_bridge">HTTP/WS Bridge（厂商 Sidecar 过渡桥）</option>
              </select>
            </div>
            <div class="form-group">
              <label class="form-label" for="impEndpoint">真机/仿真 Endpoint URL（非 Mock 必填）</label>
              <input type="text" class="form-input" id="impEndpoint" placeholder="ws://192.168.3.10:9090 或 http://.../v1">
            </div>
            <div class="form-group" style="justify-content:flex-end;">
              <label class="import-check-pill">
                <input type="checkbox" id="impReadonlyOnly">
                <span>强制只读镜像 (readonly_only · 非 Mock 必选)</span>
              </label>
            </div>
          </div>

          <!-- ROS 2 Bridge Topic Mapping & Probe Subpanel -->
          <div class="ros2-subpanel" id="impRos2Panel" hidden>
            <div style="display:flex; justify-content:space-between; align-items:center; gap:8px; flex-wrap:wrap; margin-bottom:8px;">
              <strong style="font-size:12px; color:#0f766e;">ROS 2 WebSocket (rosbridge_server) 话题映射与连通性测试</strong>
              <button type="button" class="btn btn-sm" id="btnProbeRos2Import">⚡ 测试 ROS 2 连通性</button>
            </div>
            <div class="import-form-grid">
              <div class="form-group">
                <label class="form-label" for="impRos2JointStates">关节反馈 Topic (JointState)</label>
                <input type="text" class="form-input" id="impRos2JointStates" value="/joint_states">
              </div>
              <div class="form-group">
                <label class="form-label" for="impRos2Odom">里程计 Topic (Odometry)</label>
                <input type="text" class="form-input" id="impRos2Odom" value="/odom">
              </div>
              <div class="form-group">
                <label class="form-label" for="impRos2Domain">ROS_DOMAIN_ID (备注)</label>
                <input type="number" class="form-input" id="impRos2Domain" placeholder="0 / 102">
              </div>
              <div class="form-group">
                <label class="form-label" for="impRos2JointCmd">关节指令 Topic (受控写)</label>
                <input type="text" class="form-input" id="impRos2JointCmd" value="/joint_commands">
              </div>
              <div class="form-group">
                <label class="form-label" for="impRos2CmdVel">底盘速度 Topic (Twist)</label>
                <input type="text" class="form-input" id="impRos2CmdVel" value="/cmd_vel">
              </div>
              <div class="form-group">
                <label class="form-label" for="impRos2NavGoal">导航目标 Topic (PoseStamped)</label>
                <input type="text" class="form-input" id="impRos2NavGoal" value="/goal_pose">
              </div>
            </div>
            <div id="impRos2ProbeStatus" style="font-size:11px; color:var(--text-muted); margin-top:6px;">机载端运行 <code>ros2 launch rosbridge_server rosbridge_websocket_launch.xml</code> 后填写 <code>ws://&lt;robot-ip&gt;:9090</code> 即可接入。</div>
          </div>

          <!-- Kinematics & Trail Links -->
          <div class="import-subblock">
            <div class="import-subblock-title">运动学基座与 6D IK 末端链路（自动读取自 URDF Link 列表）</div>
            <div class="import-form-grid">
              <div class="form-group">
                <label class="form-label" for="impBaseFrame">基座坐标系 (base_frame)</label>
                <select class="form-input" id="impBaseFrame"></select>
              </div>
              <div class="form-group">
                <label class="form-label" for="impLeftTip">左臂 / 主臂末端 Tip Link</label>
                <select class="form-input" id="impLeftTip"></select>
              </div>
              <div class="form-group">
                <label class="form-label" for="impRightTip">右臂末端 Tip Link（单臂可选无）</label>
                <select class="form-input" id="impRightTip"></select>
              </div>
            </div>
          </div>

          <!-- Capabilities Checkboxes -->
          <div class="import-subblock">
            <div class="import-subblock-title">能力开关 (Capabilities · 决定导航/感知/策略等页面显隐)</div>
            <div class="import-cap-grid" id="impCapGrid"></div>
          </div>

          <!-- Joint Mapping Table -->
          <div class="import-subblock">
            <div class="import-subblock-title">关节映射、软限位与 SDK 方向/零点对齐表（sign = ±1，offset = 零点偏置，scale = 行程换算）</div>
            <div class="table-wrapper import-joints-wrap">
              <table class="data-table import-joints-table">
                <thead>
                  <tr>
                    <th>平台名</th>
                    <th>URDF 关节</th>
                    <th>分组</th>
                    <th>下限</th>
                    <th>上限</th>
                    <th>初始位</th>
                    <th>方向 sign</th>
                    <th>偏置</th>
                    <th>缩放</th>
                  </tr>
                </thead>
                <tbody id="impJointsTbody"></tbody>
              </table>
            </div>
          </div>

          <!-- Advanced Raw Manifest JSON Editor -->
          <details class="import-json-details">
            <summary>高级：查看 / 手动编辑完整 robot_manifest.json</summary>
            <textarea id="impRawJson" class="import-json-textarea" spellcheck="false"></textarea>
            <div style="margin-top:6px; display:flex; justify-content:flex-end;">
              <button type="button" class="btn btn-sm" id="btnApplyRawJson">应用 JSON 并重新预检</button>
            </div>
          </details>
        </section>

        <!-- Step 3: 6-Gate Preflight -->
        <section class="import-section" id="importGatesSection" hidden>
          <div class="import-section-head">
            <span class="import-step-num">3</span>
            <strong>6 项入库预检门禁（全部通过方可注册）</strong>
            <span id="importOverallBadge" class="badge badge-idle" style="margin-left:auto;">待检查</span>
          </div>
          <div class="import-gates-grid" id="importGatesGrid"></div>
        </section>

        <!-- Managed Custom Robots List -->
        <section class="import-section import-managed-section">
          <div class="import-section-head">
            <strong>已导入自定义机型管理</strong>
            <span style="margin-left:auto; font-size:11px; color:var(--text-muted);">内置 R1 Pro / Piper Dual / Dexforce W1 受保护不可删除</span>
          </div>
          <div id="customRobotsList" class="custom-robots-list"></div>
        </section>
      </div>

      <div class="dialog-actions import-footer">
        <button type="button" class="btn btn-sm" id="btnCancelRobotImport">关闭</button>
        <button type="button" class="btn btn-sm" id="btnRevalidateImport" disabled>↻ 重新预检</button>
        <button type="button" class="btn btn-sm btn-primary" id="btnCommitRobotImport" disabled>✓ 确认导入并切换至该机型</button>
      </div>
    </div>
  `;
  document.body.appendChild(dialog);
  enableDialogLightDismiss(dialog);

  let stageId = null;
  let urdfLinks = [];
  let currentManifest = null;
  let lastReady = false;

  const dropzone = dialog.querySelector("#importDropzone");
  const fileInput = dialog.querySelector("#importZipFileInput");
  const summaryEl = dialog.querySelector("#importPackageSummary");
  const configSection = dialog.querySelector("#importConfigSection");
  const gatesSection = dialog.querySelector("#importGatesSection");
  const gatesGrid = dialog.querySelector("#importGatesGrid");
  const overallBadge = dialog.querySelector("#importOverallBadge");
  const btnRevalidate = dialog.querySelector("#btnRevalidateImport");
  const btnCommit = dialog.querySelector("#btnCommitRobotImport");
  const capGrid = dialog.querySelector("#impCapGrid");
  const jointsTbody = dialog.querySelector("#impJointsTbody");
  const rawJsonEl = dialog.querySelector("#impRawJson");

  // Render capabilities grid once
  capGrid.innerHTML = CAP_ITEMS.map(([key, label]) => `
    <label class="import-cap-item">
      <input type="checkbox" data-cap-key="${key}">
      <span>${escapeHtml(label)}</span>
    </label>
  `).join("");

  function populateLinkSelect(selectEl, selectedVal, allowNone = false) {
    const opts = [];
    if (allowNone) opts.push(`<option value="">（无 / 不配置）</option>`);
    for (const lnk of urdfLinks) {
      opts.push(`<option value="${escapeHtml(lnk)}">${escapeHtml(lnk)}</option>`);
    }
    if (selectedVal && !urdfLinks.includes(selectedVal)) {
      opts.push(`<option value="${escapeHtml(selectedVal)}">${escapeHtml(selectedVal)} (自定义)</option>`);
    }
    selectEl.innerHTML = opts.join("");
    selectEl.value = selectedVal || "";
  }

  function renderManifestToForm(manifest) {
    currentManifest = structuredClone(manifest);
    dialog.querySelector("#impRobotId").value = manifest.id || "";
    dialog.querySelector("#impRobotName").value = manifest.name || "";
    dialog.querySelector("#impRobotType").value = manifest.robot_type || "";
    dialog.querySelector("#impConnectorType").value = manifest.connector_type || "mock";
    dialog.querySelector("#impEndpoint").value = manifest.endpoint || "";
    dialog.querySelector("#impReadonlyOnly").checked = Boolean(manifest.readonly_only);

    const r2 = manifest.ros2_config || {};
    const ros2Panel = dialog.querySelector("#impRos2Panel");
    if (ros2Panel) ros2Panel.hidden = (manifest.connector_type || "mock") !== "ros2_bridge";
    dialog.querySelector("#impRos2JointStates").value = r2.joint_states_topic || "/joint_states";
    dialog.querySelector("#impRos2Odom").value = r2.odom_topic || "/odom";
    dialog.querySelector("#impRos2Domain").value = r2.ros_domain_id ?? "";
    dialog.querySelector("#impRos2JointCmd").value = r2.joint_cmd_topic || "/joint_commands";
    dialog.querySelector("#impRos2CmdVel").value = r2.cmd_vel_topic || "/cmd_vel";
    dialog.querySelector("#impRos2NavGoal").value = r2.nav_goal_topic || "/goal_pose";

    const kin = manifest.kinematics || {};
    populateLinkSelect(dialog.querySelector("#impBaseFrame"), kin.base_frame || urdfLinks[0] || "base_link", false);
    populateLinkSelect(dialog.querySelector("#impLeftTip"), kin.arms?.left?.tip || "", true);
    populateLinkSelect(dialog.querySelector("#impRightTip"), kin.arms?.right?.tip || "", true);

    const caps = manifest.capabilities || {};
    capGrid.querySelectorAll("input[data-cap-key]").forEach((cb) => {
      cb.checked = Boolean(caps[cb.dataset.capKey]);
    });

    jointsTbody.innerHTML = (manifest.joints || []).map((j, idx) => `
      <tr data-jidx="${idx}">
        <td><input type="text" class="imp-cell-input" data-jf="name" value="${escapeHtml(j.name || "")}"></td>
        <td><code style="font-size:11px;">${escapeHtml(j.urdf_joint || "")}</code></td>
        <td>
          <select class="imp-cell-input" data-jf="group">
            ${JOINT_GROUPS.map((g) => `<option value="${g}" ${j.group === g ? "selected" : ""}>${g}</option>`).join("")}
          </select>
        </td>
        <td><input type="number" step="0.01" class="imp-cell-num" data-jf="lower" value="${Number(j.lower ?? -3.14)}"></td>
        <td><input type="number" step="0.01" class="imp-cell-num" data-jf="upper" value="${Number(j.upper ?? 3.14)}"></td>
        <td><input type="number" step="0.01" class="imp-cell-num" data-jf="default" value="${Number(j.default ?? 0)}"></td>
        <td>
          <select class="imp-cell-num" data-jf="sign">
            <option value="1" ${Number(j.sign ?? 1) >= 0 ? "selected" : ""}>+1</option>
            <option value="-1" ${Number(j.sign ?? 1) < 0 ? "selected" : ""}>-1</option>
          </select>
        </td>
        <td><input type="number" step="0.01" class="imp-cell-num" data-jf="zero_offset" value="${Number(j.zero_offset ?? 0)}"></td>
        <td><input type="number" step="0.005" class="imp-cell-num" data-jf="urdf_scale" value="${Number(j.urdf_scale ?? 1)}"></td>
      </tr>
    `).join("");

    rawJsonEl.value = JSON.stringify(currentManifest, null, 2);
  }

  function collectManifestFromForm() {
    if (!currentManifest) return null;
    const m = structuredClone(currentManifest);
    m.id = dialog.querySelector("#impRobotId").value.trim();
    m.name = dialog.querySelector("#impRobotName").value.trim();
    m.robot_type = dialog.querySelector("#impRobotType").value.trim();
    m.connector_type = dialog.querySelector("#impConnectorType").value;
    m.mode = m.connector_type === "mock" ? "mock" : "real";
    m.endpoint = dialog.querySelector("#impEndpoint").value.trim();
    m.readonly_only = dialog.querySelector("#impReadonlyOnly").checked;
    if (m.connector_type === "ros2_bridge") {
      const domRaw = dialog.querySelector("#impRos2Domain").value.trim();
      m.ros2_config = {
        ws_url: m.endpoint || "ws://127.0.0.1:9090",
        ros_domain_id: domRaw === "" ? null : Number.parseInt(domRaw, 10),
        joint_states_topic: dialog.querySelector("#impRos2JointStates").value.trim() || "/joint_states",
        odom_topic: dialog.querySelector("#impRos2Odom").value.trim() || "/odom",
        joint_cmd_topic: dialog.querySelector("#impRos2JointCmd").value.trim() || "/joint_commands",
        cmd_vel_topic: dialog.querySelector("#impRos2CmdVel").value.trim() || "/cmd_vel",
        nav_goal_topic: dialog.querySelector("#impRos2NavGoal").value.trim() || "/goal_pose",
      };
    } else {
      delete m.ros2_config;
    }

    const baseFrame = dialog.querySelector("#impBaseFrame").value;
    const leftTip = dialog.querySelector("#impLeftTip").value;
    const rightTip = dialog.querySelector("#impRightTip").value;
    const arms = {};
    const trails = [];
    if (leftTip) {
      arms.left = { tip: leftTip, group: m.kinematics?.arms?.left?.group || "left_arm" };
      trails.push({ name: leftTip, color: "#00f0ff", light_color: "#0284c7", label: "L末端" });
    }
    if (rightTip) {
      arms.right = { tip: rightTip, group: m.kinematics?.arms?.right?.group || "right_arm" };
      trails.push({ name: rightTip, color: "#ff2e93", light_color: "#db2777", label: "R末端" });
    }
    m.arms_scope = Object.keys(arms).length ? Object.keys(arms) : ["left"];
    m.kinematics = {
      base_frame: baseFrame || "base_link",
      arms,
      trail_links: trails.map((t) => t.name),
    };
    m.model = {
      ...(m.model || {}),
      trail_links: trails,
    };

    m.capabilities = { ...(m.capabilities || {}) };
    capGrid.querySelectorAll("input[data-cap-key]").forEach((cb) => {
      m.capabilities[cb.dataset.capKey] = cb.checked;
    });

    jointsTbody.querySelectorAll("tr[data-jidx]").forEach((tr) => {
      const idx = Number(tr.dataset.jidx);
      const j = m.joints?.[idx];
      if (!j) return;
      j.name = tr.querySelector('[data-jf="name"]').value.trim();
      j.group = tr.querySelector('[data-jf="group"]').value;
      j.lower = parseFloat(tr.querySelector('[data-jf="lower"]').value);
      j.upper = parseFloat(tr.querySelector('[data-jf="upper"]').value);
      j.default = parseFloat(tr.querySelector('[data-jf="default"]').value);
      j.sign = parseFloat(tr.querySelector('[data-jf="sign"]').value);
      j.zero_offset = parseFloat(tr.querySelector('[data-jf="zero_offset"]').value);
      j.urdf_scale = parseFloat(tr.querySelector('[data-jf="urdf_scale"]').value);
      j.is_gripper = j.group === "gripper";
    });

    currentManifest = m;
    rawJsonEl.value = JSON.stringify(m, null, 2);
    return m;
  }

  function renderChecks(checks = [], ready = false) {
    lastReady = Boolean(ready);
    gatesSection.hidden = false;
    btnRevalidate.disabled = !stageId;
    btnCommit.disabled = !lastReady;
    overallBadge.textContent = lastReady ? "6/6 门禁全部通过" : "存在拦阻项，需修正";
    overallBadge.className = `badge ${lastReady ? "badge-passed" : "badge-failed"}`;

    gatesGrid.innerHTML = checks.map((c) => `
      <div class="import-gate-card ${c.ok ? "gate-ok" : "gate-fail"}">
        <div class="gate-head">
          <span class="gate-title">${escapeHtml(c.label)}</span>
          <span class="badge ${c.ok ? "badge-passed" : "badge-failed"}">${c.ok ? "通过" : "拦截"}</span>
        </div>
        <div class="gate-detail">${escapeHtml(c.detail)}</div>
      </div>
    `).join("");
  }

  async function refreshCustomRobotsList() {
    const wrap = dialog.querySelector("#customRobotsList");
    try {
      const res = await api.getRobots();
      const customs = (res.robots || []).filter((r) => r.is_custom);
      if (!customs.length) {
        wrap.innerHTML = `<div class="custom-robots-empty">暂无已导入的自定义机型（可点击上方「一键加载示例 6-DOF 机械臂包」快速体验）。</div>`;
        return;
      }
      wrap.innerHTML = customs.map((r) => `
        <div class="custom-robot-row">
          <div>
            <strong>${escapeHtml(r.name)}</strong>
            <span class="badge badge-info" style="margin-left:6px;">${escapeHtml(r.robot_type)} · ${r.dof} DOF · ${escapeHtml(r.connector_type.toUpperCase())}</span>
            <div style="font-size:11px; color:var(--text-muted); font-family:var(--font-mono); margin-top:2px;">ID: ${escapeHtml(r.id)}${r.endpoint ? ` · ${escapeHtml(r.endpoint)}` : ""}</div>
          </div>
          <div style="display:flex; gap:6px;">
            <button type="button" class="btn btn-sm" data-switch-custom="${escapeHtml(r.id)}">切换查看</button>
            <button type="button" class="btn btn-sm" style="color:var(--danger); border-color:#fecaca;" data-delete-custom="${escapeHtml(r.id)}">删除</button>
          </div>
        </div>
      `).join("");

      wrap.querySelectorAll("[data-switch-custom]").forEach((btn) => {
        btn.addEventListener("click", async () => {
          const rid = btn.dataset.switchCustom;
          dialog.close("switch");
          await onRobotsChanged(rid, true);
        });
      });
      wrap.querySelectorAll("[data-delete-custom]").forEach((btn) => {
        btn.addEventListener("click", async () => {
          const rid = btn.dataset.deleteCustom;
          try {
            await api.deleteCustomRobot(rid);
            notify(`已安全移除自定义机型 '${rid}'`, "info");
            await refreshCustomRobotsList();
            await onRobotsChanged(null, false);
          } catch (err) {
            notify("删除自定义机型失败: " + err.message, "error");
          }
        });
      });
    } catch (err) {
      wrap.innerHTML = `<div class="custom-robots-empty">加载列表失败: ${escapeHtml(err.message)}</div>`;
    }
  }

  async function handleInspect(file = null, useSample = false) {
    summaryEl.hidden = false;
    summaryEl.textContent = useSample ? "正在加载内置 6-DOF 机械臂示例包并运行 6 项入库预检..." : `正在解包 '${file?.name || "archive.zip"}' 并运行 6 项入库预检...`;
    btnCommit.disabled = true;
    try {
      const res = await api.inspectRobotZip(file, useSample);
      stageId = res.stage_id;
      urdfLinks = res.urdf_summary?.links || [];
      const srcLabel = res.manifest_source === "manifest_json" ? "已读取包内 robot_manifest.json" : "无 Manifest，已从 URDF 自动推断关节与运动学链";
      const normCount = (res.urdf_summary?.normalized_uris || []).length;
      summaryEl.innerHTML =
        `<strong>已解包 ${res.extracted_count} 个文件</strong> · URDF: <code>${escapeHtml(res.urdf_summary?.urdf_path || "")}</code> · ` +
        `视觉网格 <b>${res.urdf_summary?.visual_count || 0}</b> (${escapeHtml(res.urdf_summary?.format_label || "URDF")}) · ` +
        `${escapeHtml(srcLabel)}` +
        (normCount ? ` · 自动归一化 ${normCount} 条 package:// 路径` : "");
      configSection.hidden = false;
      renderManifestToForm(res.manifest);
      renderChecks(res.checks, res.ready_to_commit);
      notify(res.ready_to_commit ? "6 项入库预检全部通过，可确认导入！" : "预检发现待修正项，请查看下方门禁提示", res.ready_to_commit ? "success" : "info");
    } catch (err) {
      stageId = null;
      configSection.hidden = true;
      gatesSection.hidden = true;
      summaryEl.innerHTML = `<span style="color:var(--danger); font-weight:600;">解包/预检失败：</span>${escapeHtml(err.message)}`;
      notify("导入包校验失败: " + err.message, "error");
    }
  }

  async function revalidateCurrent() {
    if (!stageId) return;
    const manifest = collectManifestFromForm();
    try {
      const res = await api.validateRobotImport(stageId, manifest);
      renderChecks(res.checks, res.ready_to_commit);
    } catch (err) {
      notify("重新预检失败: " + err.message, "error");
    }
  }

  // Event Bindings
  dialog.querySelector("#btnCloseRobotImport").addEventListener("click", () => dialog.close("cancel"));
  dialog.querySelector("#btnCancelRobotImport").addEventListener("click", () => dialog.close("cancel"));

  dialog.querySelector("#btnLoadSampleRobot").addEventListener("click", () => handleInspect(null, true));

  dropzone.addEventListener("click", () => fileInput.click());
  dropzone.addEventListener("keydown", (e) => {
    if (e.key === "Enter" || e.key === " ") {
      e.preventDefault();
      fileInput.click();
    }
  });
  dropzone.addEventListener("dragover", (e) => {
    e.preventDefault();
    dropzone.classList.add("drag-over");
  });
  dropzone.addEventListener("dragleave", () => dropzone.classList.remove("drag-over"));
  dropzone.addEventListener("drop", (e) => {
    e.preventDefault();
    dropzone.classList.remove("drag-over");
    const f = e.dataTransfer?.files?.[0];
    if (f) handleInspect(f, false);
  });
  fileInput.addEventListener("change", () => {
    const f = fileInput.files?.[0];
    fileInput.value = "";
    if (f) handleInspect(f, false);
  });

  // Auto-toggle readonly_only & ROS 2 topic subpanel when switching connector type
  dialog.querySelector("#impConnectorType").addEventListener("change", (e) => {
    const ctype = e.target.value;
    const isNonMock = ctype !== "mock";
    dialog.querySelector("#impReadonlyOnly").checked = isNonMock;
    const ros2Panel = dialog.querySelector("#impRos2Panel");
    if (ros2Panel) ros2Panel.hidden = ctype !== "ros2_bridge";
    const epInput = dialog.querySelector("#impEndpoint");
    if (ctype === "ros2_bridge" && !epInput.value.trim()) {
      epInput.value = "ws://127.0.0.1:9090";
    }
    revalidateCurrent();
  });

  dialog.querySelector("#btnProbeRos2Import").addEventListener("click", async () => {
    const wsUrl = dialog.querySelector("#impEndpoint").value.trim() || "ws://127.0.0.1:9090";
    const statusEl = dialog.querySelector("#impRos2ProbeStatus");
    statusEl.textContent = `正在探测 ${wsUrl} ...`;
    try {
      const res = await api.probeRos2Bridge(wsUrl, 2.0);
      if (res.ok) {
        statusEl.innerHTML = `<span style="color:var(--success); font-weight:600;">✓ ${escapeHtml(res.message)}</span>`;
        notify(res.message, "success");
      } else {
        statusEl.innerHTML = `<span style="color:var(--danger); font-weight:600;">✕ ${escapeHtml(res.error || "连接失败")}</span>`;
        notify(res.error || "ROS 2 探测失败", "error");
      }
    } catch (err) {
      statusEl.innerHTML = `<span style="color:var(--danger);">${escapeHtml(err.message)}</span>`;
    }
  });

  dialog.querySelector("#btnApplyRawJson").addEventListener("click", async () => {
    try {
      const parsed = JSON.parse(rawJsonEl.value);
      renderManifestToForm(parsed);
      await revalidateCurrent();
      notify("已应用 JSON 配置并完成预检", "info");
    } catch (err) {
      notify("JSON 格式错误: " + err.message, "error");
    }
  });

  btnRevalidate.addEventListener("click", revalidateCurrent);

  btnCommit.addEventListener("click", async () => {
    if (!stageId) return;
    const manifest = collectManifestFromForm();
    btnCommit.disabled = true;
    try {
      const res = await api.commitRobotImport(stageId, manifest);
      const newRobot = res.robot;
      stageId = null;
      notify(`已成功导入并热注册机型 '${newRobot.name}' (${newRobot.joints.length} DOF)`, "success");
      dialog.close("imported");
      await onRobotsChanged(newRobot.id, true);
    } catch (err) {
      notify("导入入库失败: " + err.message, "error");
      await revalidateCurrent();
    }
  });

  return {
    async open() {
      await refreshCustomRobotsList();
      dialog.showModal();
    },
  };
}
