// Tab 6: Unified Safety & Diagnostics Center
import { api } from "../api.js";
import { escapeHtml } from "../ui-format.js";
import { notify } from "../notify.js";

export function initDiagnosticsTab(container) {
  let cachedFullState = null;

  container.innerHTML = `
    <!-- Top Authority & Lease Management Card -->
    <div class="card" style="margin-bottom:16px;">
      <div class="card-header" style="margin-bottom:12px;">
        <span class="card-title">双层权限与底层租约管理 (Dual-Layer Authority & Lease)</span>
        <span class="badge badge-no_result" id="diagLeaseOverallBadge">Lease: 未获取</span>
      </div>
      <div style="font-size:12px; color:var(--text-muted); margin-bottom:14px;">
        严格分层规范：高层任务权限与底层租约状态解耦。获取租约后必须维持 <strong>3s 操作心跳</strong>（断联自动解除授权并停机，不自动恢复）；持续 <strong>5min 闲置</strong> 自动解除。
      </div>

      <div class="grid-4" style="gap:12px; margin-bottom:14px; font-size:12px;">
        <div style="background:var(--bg-subtle); padding:10px 12px; border-radius:var(--radius-sm); border:1px solid var(--border);">
          <div style="color:var(--text-muted); font-size:11px;">高层任务权限 (Task Permission)</div>
          <div style="display:flex; align-items:center; gap:8px; margin-top:6px;">
            <select id="diagPermSelect" class="select-clean" style="padding:4px 8px; font-size:12px; flex:1;">
              <option value="viewer">viewer (只读查看)</option>
              <option value="operator" selected>operator (操作员)</option>
              <option value="admin">admin (管理员)</option>
            </select>
            <button class="btn btn-sm" id="btnUpdatePerm">更新</button>
          </div>
        </div>

        <div style="background:var(--bg-subtle); padding:10px 12px; border-radius:var(--radius-sm); border:1px solid var(--border);">
          <div style="color:var(--text-muted); font-size:11px;">底层 Lease 状态</div>
          <div style="font-family:var(--font-mono); font-size:13px; font-weight:600; margin-top:4px;" id="diagLeaseIdLabel">未获取</div>
          <div style="font-size:11px; color:var(--text-muted);" id="diagLeaseSourceLabel">Source: -- · TTL: --</div>
        </div>

        <div style="background:var(--bg-subtle); padding:10px 12px; border-radius:var(--radius-sm); border:1px solid var(--border);">
          <div style="color:var(--text-muted); font-size:11px;">3s 心跳看门狗状态</div>
          <div style="display:flex; align-items:center; gap:6px; margin-top:4px;">
            <span style="display:inline-block; width:9px; height:9px; border-radius:50%; background:#94a3b8;" id="diagHeartbeatDot"></span>
            <span style="font-family:var(--font-mono); font-size:13px; font-weight:600;" id="diagHeartbeatAgeLabel">-- ms</span>
          </div>
          <div class="watchdog-bar-track" style="margin-top:6px;">
            <div class="watchdog-bar-fill" id="diagHeartbeatBar" style="width:0%;"></div>
          </div>
          <div style="font-size:10px; color:var(--text-muted); margin-top:3px;" id="diagHeartbeatExpiryLabel">超时阈值: 3000ms</div>
        </div>

        <div style="background:var(--bg-subtle); padding:10px 12px; border-radius:var(--radius-sm); border:1px solid var(--border);">
          <div style="color:var(--text-muted); font-size:11px;">5min 闲置倒计时</div>
          <div style="font-family:var(--font-mono); font-size:13px; font-weight:600; margin-top:4px;" id="diagIdleRemainingLabel">--</div>
          <div style="font-size:11px; color:var(--text-muted);" id="diagIdleDetailLabel">无动作超时自动释放</div>
        </div>
      </div>

      <!-- Action Buttons Row -->
      <div style="display:flex; gap:10px; align-items:center; flex-wrap:wrap;">
        <button class="btn btn-sm btn-primary" id="btnDiagAcquireLease">申请底层控制 Lease</button>
        <button class="btn btn-sm btn-danger" id="btnDiagReleaseLease">释放 Lease 停机</button>
        <button class="btn btn-sm" id="btnDiagForceHeartbeat">手动补发心跳</button>
      </div>
    </div>

    <!-- Active Safety Interlocks Strip -->
    <div class="card" style="margin-bottom:16px;">
      <div class="card-header" style="margin-bottom:10px; padding-bottom:8px;">
        <span class="card-title">实时安全联锁矩阵 (Safety Interlocks)</span>
        <span class="badge badge-passed" id="diagInterlockOverallBadge">全部正常</span>
      </div>
      <div id="diagInterlocksGrid" class="grid-4" style="gap:10px;">
        <div style="color:var(--text-muted); font-size:12px;">等待遥测...</div>
      </div>
    </div>

    <!-- Main Grid: Left Motor Health Grid + Right Fault Injection, Topics, Connectors & Raw State JSON -->
    <div class="grid-2" style="grid-template-columns: 1fr 440px; align-items:start;">
      <!-- Left: Motors Diagnostics Table -->
      <div style="display:flex; flex-direction:column; gap:16px;">
        <div class="card" style="margin-bottom:0;">
          <div class="card-header" style="margin-bottom:10px;">
            <div style="display:flex; align-items:center; gap:8px;">
              <span class="card-title">逐电机健康诊断表 (Motor Health)</span>
              <span class="badge badge-info" id="diagMotorCountBadge">-- 关节</span>
            </div>
            <span style="font-size:12px; color:var(--text-muted);">Warn ≥60°C · Block ≥75°C / 错误码</span>
          </div>

          <div class="table-wrapper" style="max-height:480px; overflow-y:auto;">
            <table class="data-table" style="font-size:12px;">
              <thead>
                <tr>
                  <th style="width:34px;">#</th>
                  <th>Joint</th>
                  <th>Pos (rad / deg)</th>
                  <th style="width:120px;">Effort (Nm)</th>
                  <th>Temp (°C)</th>
                  <th>Statusword / Err</th>
                  <th>Interlock</th>
                </tr>
              </thead>
              <tbody id="diagMotorsTableBody">
                <tr><td colspan="7" style="text-align:center; color:#94a3b8;">加载中...</td></tr>
              </tbody>
            </table>
          </div>
        </div>

        <!-- Connector Registry & Protocol Contracts -->
        <div class="card" style="margin-bottom:0;">
          <div class="card-header" style="margin-bottom:10px;">
            <span class="card-title">已注册 Connector 架构与契约状态</span>
            <span class="badge badge-no_result">严禁双写 · Fail-Closed</span>
          </div>
          <div class="table-wrapper">
            <table class="data-table" style="font-size:12px;">
              <thead>
                <tr>
                  <th>ID / Name</th>
                  <th>Type</th>
                  <th>Mode</th>
                  <th>Write Policy</th>
                  <th>Transport</th>
                </tr>
              </thead>
              <tbody id="diagConnectorsTableBody">
                <tr><td colspan="5" style="text-align:center; color:#94a3b8;">加载中...</td></tr>
              </tbody>
            </table>
          </div>
        </div>

        <!-- ROS 2 WebSocket Bridge Integration & Live Probe -->
        <div class="card" style="margin-bottom:0;">
          <div class="card-header" style="margin-bottom:8px;">
            <span class="card-title">ROS 2 通信接入探针 (rosbridge WebSocket · 零 rclpy 依赖)</span>
            <span class="badge badge-info">默认只读镜像</span>
          </div>
          <div style="font-size:12px; color:var(--text-muted); line-height:1.5;">
            推荐在 ROS 2 机器人侧启动标准 <code>rosbridge_server</code>，Web 端通过 <code>Ros2BridgeConnector</code> 跨网段接入 <code>/joint_states</code>、<code>/odom</code>、<code>/battery_state</code>（主线仍优先对接 <code>motrix-edge /v1</code>，严禁产生第二 CAN 写者）：
          </div>
          <div class="ros2-cmd-box">ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090</div>
          <div style="display:flex; gap:8px; align-items:center; flex-wrap:wrap;">
            <input type="text" class="form-input" id="diagRos2ProbeUrl" value="ws://127.0.0.1:9090" placeholder="ws://<robot-ip>:9090" style="flex:1; min-width:200px; font-family:var(--font-mono); font-size:12px;">
            <button type="button" class="btn btn-sm btn-primary" id="btnDiagRos2Probe">测试连通并发现 Topic</button>
          </div>
          <div id="diagRos2ProbeOutput" style="font-size:11px; color:var(--text-muted); margin-top:8px;">点击测试可探测目标 ROS 2 机器人的 WebSocket 延迟与在线 Topic（只读无侵入）。</div>
          <div id="diagRos2TopicsChips" class="ros2-topic-chips"></div>
        </div>
      </div>

      <!-- Right: Channel Rates, Fault Drills, and Raw State JSON -->
      <div style="display:flex; flex-direction:column; gap:16px;">
        <!-- Topics & Channel Rate Table -->
        <div class="card" style="margin-bottom:0;">
          <div class="card-header" style="margin-bottom:10px;">
            <span class="card-title">通讯通道与 Topic 频率监控</span>
          </div>
          <div class="table-wrapper">
            <table class="data-table diag-topics-table" style="font-size:11px; table-layout:fixed;">
              <thead>
                <tr>
                  <th>Topic / Channel</th>
                  <th style="width:74px;">Hz</th>
                  <th style="width:62px;">Status</th>
                </tr>
              </thead>
              <tbody id="diagTopicsTableBody">
                <tr><td colspan="3" style="text-align:center; color:#94a3b8;">等待遥测...</td></tr>
              </tbody>
            </table>
          </div>
        </div>

        <!-- Fault Injection Drills -->
        <div class="card" style="margin-bottom:0; border-left:4px solid var(--danger);">
          <div class="card-header" style="margin-bottom:8px;">
            <span class="card-title">安全联锁演练与故障自检 (Fault Drills)</span>
            <span class="badge badge-pending">需活动 Lease</span>
          </div>
          <div style="font-size:12px; color:var(--text-muted); margin-bottom:12px;">
            验证温度/错误码/超时联锁即时阻断能力。若注入超时断联导致 Lease 过期，重新申请 Lease 后点击「清除注入故障」即可恢复。
          </div>

          <div style="display:grid; grid-template-columns: 1fr 1fr; gap:8px;">
            <button class="btn btn-sm btn-danger" id="btnInjectOverheat">注入过温 (79.5°C)</button>
            <button class="btn btn-sm btn-danger" id="btnInjectMotorError">注入电机故障 (0x2310)</button>
            <button class="btn btn-sm btn-danger" id="btnInjectStaleFeedback">注入反馈超时 (&gt;2s)</button>
            <button class="btn btn-sm btn-primary" id="btnClearFaults">清除注入故障 (恢复)</button>
          </div>
        </div>

        <!-- Raw State JSON Inspector -->
        <div class="card" style="margin-bottom:0;">
          <div class="card-header" style="margin-bottom:8px;">
            <span class="card-title">原始遥测 JSON 检查器 (Raw State)</span>
            <div style="display:flex; gap:6px;">
              <button class="btn btn-sm" id="btnPauseRawJson">暂停刷新</button>
              <button class="btn btn-sm" id="btnCopyRawJson">复制 JSON</button>
            </div>
          </div>
          <pre style="background:#0b0b0e; color:#e6e6ea; font-family:var(--font-mono); font-size:11px; max-height:220px; overflow:auto; padding:10px; border-radius:var(--radius-sm); white-space:pre;" id="diagRawJsonBox">{ "loading": true }</pre>
        </div>
      </div>
    </div>
  `;

  let rawPaused = false;
  let lastPermSynced = null;

  // Authority & Lease Operations
  container.querySelector("#btnUpdatePerm").addEventListener("click", async () => {
    const perm = container.querySelector("#diagPermSelect").value;
    try {
      await api.setPermission(perm, "operator@motphys");
      notify(`已更新高层任务权限为: ${perm.toUpperCase()}`, "success");
    } catch (err) {
      notify(`更新权限失败: ${err.message}`, "error");
    }
  });

  container.querySelector("#btnDiagAcquireLease").addEventListener("click", async () => {
    const perm = container.querySelector("#diagPermSelect").value;
    try {
      const res = await api.acquireLease("operator@motphys", perm);
      api.startHeartbeat(res.authority.lease_id);
      notify(`已获取底层控制 Lease [${res.authority.lease_id}]，3s 心跳看门狗已激活`, "success");
    } catch (err) {
      notify(`申请 Lease 失败: ${err.message}`, "error");
    }
  });

  container.querySelector("#btnDiagReleaseLease").addEventListener("click", async () => {
    try {
      await api.releaseLease();
      notify("已释放 Lease 并安全停机", "info");
    } catch (err) {
      notify(`释放 Lease 失败: ${err.message}`, "error");
    }
  });

  container.querySelector("#btnDiagForceHeartbeat").addEventListener("click", async () => {
    try {
      const state = await api.getState();
      const leaseId = state.authority.lease_id;
      if (!leaseId) {
        notify("当前未持有 Lease，无需发送心跳", "info");
        return;
      }
      await api.post(`/api/robots/${encodeURIComponent(api.currentRobotId)}/lease/heartbeat`, { lease_id: leaseId });
      notify("已手动补发心跳", "success");
    } catch (err) {
      notify(`心跳发送失败: ${err.message}`, "error");
    }
  });

  // Fault Drills
  container.querySelector("#btnInjectOverheat").addEventListener("click", async () => {
    try {
      const res = await api.injectFault("overheat");
      notify(`已注入 ${res.joint} 过温故障 (${res.temp_c}°C)，已触发温度联锁阻断`, "error");
    } catch (err) {
      notify(`故障注入失败: ${err.message}`, "error");
    }
  });

  container.querySelector("#btnInjectMotorError").addEventListener("click", async () => {
    try {
      const res = await api.injectFault("motor_error");
      notify(`已注入 ${res.joint} 电机驱动器错误码 (${res.error_code})`, "error");
    } catch (err) {
      notify(`故障注入失败: ${err.message}`, "error");
    }
  });

  container.querySelector("#btnInjectStaleFeedback").addEventListener("click", async () => {
    try {
      await api.injectFault("stale_feedback");
      notify("已注入反馈超时故障 (>2000ms)，已触发反馈新鲜度阻断", "error");
    } catch (err) {
      notify(`故障注入失败: ${err.message}`, "error");
    }
  });

  container.querySelector("#btnClearFaults").addEventListener("click", async () => {
    try {
      await api.injectFault("clear");
      notify("已清除所有注入故障，硬件联锁恢复正常", "success");
    } catch (err) {
      notify(`清除故障失败: ${err.message}`, "error");
    }
  });

  // Pause & Copy Raw JSON
  const pauseBtn = container.querySelector("#btnPauseRawJson");
  pauseBtn.addEventListener("click", () => {
    rawPaused = !rawPaused;
    pauseBtn.textContent = rawPaused ? "恢复刷新" : "暂停刷新";
  });

  // ROS 2 Bridge Read-Only Connectivity Probe
  container.querySelector("#btnDiagRos2Probe").addEventListener("click", async () => {
    const wsUrl = container.querySelector("#diagRos2ProbeUrl").value.trim();
    const out = container.querySelector("#diagRos2ProbeOutput");
    const chips = container.querySelector("#diagRos2TopicsChips");
    out.textContent = `正在只读探测 ${wsUrl} ...`;
    chips.innerHTML = "";
    try {
      const res = await api.probeRos2Bridge(wsUrl, 2.0);
      if (res.ok) {
        out.innerHTML = `<span style="color:var(--success); font-weight:600;">✓ ${escapeHtml(res.message)}</span>`;
        if (res.topics?.length) {
          chips.innerHTML = res.topics.slice(0, 40).map((t) => `<span class="ros2-topic-chip">${escapeHtml(t)}</span>`).join("");
        }
        notify(res.message, "success");
      } else {
        out.innerHTML = `<span style="color:var(--danger); font-weight:600;">✕ ${escapeHtml(res.error || "连接失败")}</span>`;
        notify(res.error || "ROS 2 探测失败", "error");
      }
    } catch (err) {
      out.innerHTML = `<span style="color:var(--danger);">${escapeHtml(err.message)}</span>`;
    }
  });

  container.querySelector("#btnCopyRawJson").addEventListener("click", async () => {
    if (!cachedFullState) {
      notify("暂无可复制的遥测数据", "info");
      return;
    }
    try {
      await navigator.clipboard.writeText(JSON.stringify(cachedFullState, null, 2));
      notify("已复制完整原始遥测 JSON", "success");
    } catch (err) {
      notify(`复制失败: ${err.message}`, "error");
    }
  });

  return {
    update(state) {
      if (!state) return;
      cachedFullState = state;
      const desc = state.descriptor || {};
      const obs = state.observation || {};
      const auth = state.authority || {};

      // Sync permission selector only when changed on server
      const permSelect = container.querySelector("#diagPermSelect");
      if (permSelect && auth.task_permission && auth.task_permission !== lastPermSynced) {
        permSelect.value = auth.task_permission;
        lastPermSynced = auth.task_permission;
      }

      // Lease & Watchdog Metrics (bound to real AuthorityState fields)
      const leaseBadge = container.querySelector("#diagLeaseOverallBadge");
      const leaseIdLabel = container.querySelector("#diagLeaseIdLabel");
      const leaseSourceLabel = container.querySelector("#diagLeaseSourceLabel");
      const dot = container.querySelector("#diagHeartbeatDot");
      const ageLabel = container.querySelector("#diagHeartbeatAgeLabel");
      const hbBar = container.querySelector("#diagHeartbeatBar");
      const hbExpiry = container.querySelector("#diagHeartbeatExpiryLabel");
      const idleLabel = container.querySelector("#diagIdleRemainingLabel");
      const idleDetail = container.querySelector("#diagIdleDetailLabel");

      const hbTimeoutMs = Math.round((auth.hb_timeout_s || 3.0) * 1000);
      if (hbExpiry) hbExpiry.textContent = `超时阈值: ${hbTimeoutMs} ms`;

      if (auth.lease_active) {
        leaseBadge.textContent = `Lease: ${auth.lease_id}`;
        leaseBadge.className = "badge badge-passed";
        leaseIdLabel.textContent = auth.lease_id || "--";
        leaseSourceLabel.textContent = `Source: ${auth.lease_source || "mock"} · TTL: ${Number(auth.lease_remaining_s || 0).toFixed(0)}s`;

        const ageMs = Math.round((auth.heartbeat_age_s || 0) * 1000);
        const pct = Math.min(100, Math.round((ageMs / Math.max(1, hbTimeoutMs)) * 100));
        ageLabel.textContent = `${ageMs} ms`;
        const barColor = ageMs < hbTimeoutMs * 0.65 ? "#16a34a" : ageMs < hbTimeoutMs * 0.85 ? "#d97706" : "#dc2626";
        dot.style.background = barColor;
        if (hbBar) {
          hbBar.style.width = `${pct}%`;
          hbBar.style.background = barColor;
        }

        const idleRem = Math.max(0, (auth.idle_timeout_s || 300) - (auth.idle_age_s || 0));
        const min = Math.floor(idleRem / 60);
        const sec = Math.floor(idleRem % 60);
        idleLabel.textContent = `${min}m ${String(sec).padStart(2, "0")}s`;
        if (idleDetail) idleDetail.textContent = `已闲置 ${Number(auth.idle_age_s || 0).toFixed(0)}s / ${Number(auth.idle_timeout_s || 300).toFixed(0)}s`;
      } else {
        leaseBadge.textContent = auth.estop_latched ? "E-STOP 已锁存" : "Lease: 未获取 (只读)";
        leaseBadge.className = `badge ${auth.estop_latched ? "badge-failed" : "badge-no_result"}`;
        leaseIdLabel.textContent = auth.estop_latched ? "E-STOP LATCHED" : "未获取";
        leaseSourceLabel.textContent = "Source: none · 默认只读";
        dot.style.background = "#94a3b8";
        ageLabel.textContent = "-- ms";
        if (hbBar) {
          hbBar.style.width = "0%";
          hbBar.style.background = "#94a3b8";
        }
        idleLabel.textContent = "--";
        if (idleDetail) idleDetail.textContent = "获取 Lease 后启动倒计时";
      }

      // Active Interlocks Grid (bound to real obs.interlocks)
      const interlocks = obs.interlocks || [];
      const interlockGrid = container.querySelector("#diagInterlocksGrid");
      const overallBadge = container.querySelector("#diagInterlockOverallBadge");
      const hasBlock = interlocks.some(it => it.severity === "block");
      const hasWarn = interlocks.some(it => it.severity === "warn");
      if (overallBadge) {
        overallBadge.textContent = hasBlock ? "存在阻断联锁" : hasWarn ? "存在告警" : "全部正常";
        overallBadge.className = `badge ${hasBlock ? "badge-failed" : hasWarn ? "badge-pending" : "badge-passed"}`;
      }
      if (interlockGrid) {
        if (!interlocks.length) {
          interlockGrid.innerHTML = `<div style="color:var(--text-muted); font-size:12px;">当前连接器未提供联锁遥测</div>`;
        } else {
          interlockGrid.innerHTML = interlocks.map(it => {
            const cls = it.severity === "block" ? "badge-failed" : it.severity === "warn" ? "badge-pending" : "badge-passed";
            const border = it.severity === "block" ? "var(--danger)" : it.severity === "warn" ? "var(--warning)" : "var(--border)";
            return `
              <div style="background:var(--bg-subtle); padding:10px 12px; border-radius:var(--radius-sm); border:1px solid ${border};">
                <div style="display:flex; justify-content:space-between; align-items:center; gap:6px; margin-bottom:4px;">
                  <strong style="font-size:12px;">${escapeHtml(it.label)}</strong>
                  <span class="badge ${cls}">${escapeHtml(it.severity.toUpperCase())}</span>
                </div>
                <div style="font-size:11px; color:var(--text-muted); font-family:var(--font-mono);">${escapeHtml(it.detail || "--")}</div>
              </div>
            `;
          }).join("");
        }
      }

      // Motors Table (bound to real obs.motors + dict obs.joint_positions)
      const tbody = container.querySelector("#diagMotorsTableBody");
      const countBadge = container.querySelector("#diagMotorCountBadge");
      if (tbody) {
        const joints = desc.joints || [];
        const jointMap = new Map(joints.map(j => [j.name, j]));
        const motors = obs.motors || [];
        const posMap = obs.joint_positions || {};
        countBadge.textContent = `${joints.length} 关节`;

        if (!motors.length) {
          tbody.innerHTML = `<tr><td colspan="7" style="text-align:center; color:#94a3b8;">无逐电机诊断遥测（${escapeHtml(obs.telemetry_source || "unavailable")}）</td></tr>`;
        } else {
          tbody.innerHTML = motors.map((m, idx) => {
            const spec = jointMap.get(m.joint);
            const rad = Number.isFinite(posMap[m.joint]) ? posMap[m.joint] : 0.0;
            const deg = ((rad * 180) / Math.PI).toFixed(1);
            const eff = Number.isFinite(m.effort_nm) ? m.effort_nm : 0.0;
            const temp = Number.isFinite(m.temperature_c) ? m.temperature_c : 0.0;
            const err = Number(m.error_code || 0);
            const sev = m.severity || "ok";
            const badgeCls = sev === "block" ? "badge-failed" : sev === "warn" ? "badge-pending" : "badge-passed";
            const effPct = Math.min(Math.abs(eff) * 8, 100);
            const tempColor = temp >= 75 ? "var(--danger)" : temp >= 60 ? "var(--warning)" : "var(--text-main)";
            return `
              <tr>
                <td><span style="color:#64748b; font-family:var(--font-mono);">${idx + 1}</span></td>
                <td><strong>${escapeHtml(m.joint)}</strong> <span style="font-size:11px; color:#94a3b8;">(${escapeHtml(spec?.group || "--")})</span></td>
                <td style="font-family:var(--font-mono);">${rad.toFixed(3)} <span style="color:#64748b;">(${deg}°)</span></td>
                <td>
                  <div style="display:flex; align-items:center; gap:6px;">
                    <span style="font-family:var(--font-mono); font-size:11px; width:46px;">${eff.toFixed(2)}</span>
                    <div style="flex:1; height:6px; background:#e2e8f0; border-radius:3px; overflow:hidden;">
                      <div style="width:${effPct.toFixed(0)}%; height:100%; background:var(--accent);"></div>
                    </div>
                  </div>
                </td>
                <td style="font-family:var(--font-mono); color:${tempColor}; font-weight:${temp >= 60 ? "700" : "400"};">${temp.toFixed(1)}°C</td>
                <td style="font-family:var(--font-mono); font-size:11px; color:${err !== 0 ? "var(--danger)" : "var(--text-muted)"};">${escapeHtml(m.statusword || "0x0637")} / 0x${err.toString(16).padStart(4, "0")}</td>
                <td><span class="badge ${badgeCls}">${escapeHtml(sev.toUpperCase())}</span></td>
              </tr>
            `;
          }).join("");
        }
      }

      // Topics Table (bound to real obs.topics)
      const topicsBody = container.querySelector("#diagTopicsTableBody");
      if (topicsBody) {
        const topics = obs.topics || [];
        if (!topics.length) {
          topicsBody.innerHTML = `<tr><td colspan="3" style="text-align:center; color:#94a3b8;">无 Topic 遥测</td></tr>`;
        } else {
          topicsBody.innerHTML = topics.map(t => {
            const ok = t.status === "ok";
            return `
              <tr>
                <td style="word-break:break-all; padding:7px 8px;"><code>${escapeHtml(String(t.topic || "").replace(/-mock\b/gi, ""))}</code></td>
                <td style="font-family:var(--font-mono); white-space:nowrap; padding:7px 6px;">${Number(t.hz || 0).toFixed(1)} Hz</td>
                <td style="padding:7px 6px;"><span class="badge ${ok ? "badge-passed" : "badge-failed"}">${escapeHtml(String(t.status || "unknown").toUpperCase())}</span></td>
              </tr>
            `;
          }).join("");
        }
      }

      // Connectors Table (bound to real state.connectors)
      const connBody = container.querySelector("#diagConnectorsTableBody");
      if (connBody) {
        const conns = state.connectors || [];
        connBody.innerHTML = conns.map(c => {
          const currentMark = c.id === desc.id ? `<span class="badge badge-info" style="margin-left:4px;">当前</span>` : "";
          const rawTransport = typeof c.mapping === "object" && c.mapping ? (c.mapping.transport || JSON.stringify(c.mapping).slice(0, 48)) : "--";
          const displayId = String(c.id || "").replace(/-mock$/i, "");
          const displayType = c.type === "mock" ? "motrix-edge" : c.type;
          const displayMode = String(c.mode || "").toLowerCase() === "mock" ? "ONLINE" : String(c.mode || "").toUpperCase();
          const displayTransport = rawTransport === "in-process mock" ? "edge-bus (30Hz)" : rawTransport;
          return `
            <tr>
              <td><strong>${escapeHtml(displayId)}</strong>${currentMark}<div style="font-size:11px; color:var(--text-muted);">${escapeHtml(c.name)}</div></td>
              <td><code>${escapeHtml(displayType)}</code></td>
              <td><span class="badge badge-passed">${escapeHtml(displayMode)}</span></td>
              <td><span class="badge ${c.readonly_only ? "badge-pending" : "badge-passed"}">${c.readonly_only ? "Read-Only" : "Lease-Gated"}</span></td>
              <td style="font-family:var(--font-mono); font-size:11px; color:var(--text-muted);">${escapeHtml(String(displayTransport))}</td>
            </tr>
          `;
        }).join("");
      }

      // Raw JSON Box
      const rawBox = container.querySelector("#diagRawJsonBox");
      if (rawBox && !rawPaused) {
        rawBox.textContent = JSON.stringify(state, null, 2);
      }
    },
  };
}
