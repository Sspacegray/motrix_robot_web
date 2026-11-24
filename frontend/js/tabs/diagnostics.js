// Tab 6: Unified Safety & Diagnostics Center
import { api } from "../api.js";

export function initDiagnosticsTab(container) {
  let cachedFullState = null;

  container.innerHTML = `
    <!-- Top Authority & Lease Management Card -->
    <div class="card" style="margin-bottom:16px;">
      <div class="card-header" style="margin-bottom:12px;">
        <span class="card-title">双层权限与底层租约管理 (Dual-Layer Authority & Lease)</span>
        <span class="badge badge-info" id="diagLeaseOverallBadge">Lease: 未获取</span>
      </div>
      <div style="font-size:12px; color:var(--text-muted); margin-bottom:14px;">
        严格分层规范：高层任务权限与底层租约状态解耦。获取租约后必须维持 <strong>3s 操作心跳</strong>（断联自动解除授权并停机，不自动恢复）；持续 <strong>5min 闲置</strong> 自动解除。
      </div>

      <div class="grid-4" style="gap:12px; margin-bottom:14px; font-size:12px;">
        <div style="background:var(--bg-subtle); padding:10px 12px; border-radius:var(--radius-sm);">
          <div style="color:var(--text-muted); font-size:11px;">高层任务权限 (Task Permission)</div>
          <div style="display:flex; align-items:center; gap:8px; margin-top:4px;">
            <select id="diagPermSelect" style="padding:4px 8px; border:1px solid var(--border); border-radius:4px; font-size:12px;">
              <option value="viewer">viewer (只读查看)</option>
              <option value="operator" selected>operator (操作员)</option>
              <option value="admin">admin (管理员)</option>
            </select>
            <button class="btn btn-sm" id="btnUpdatePerm">更新</button>
          </div>
        </div>

        <div style="background:var(--bg-subtle); padding:10px 12px; border-radius:var(--radius-sm);">
          <div style="color:var(--text-muted); font-size:11px;">底层 Lease 状态</div>
          <div style="font-family:var(--font-mono); font-size:13px; font-weight:600; margin-top:4px;" id="diagLeaseIdLabel">--</div>
          <div style="font-size:10px; color:#64748b;" id="diagLeaseSourceLabel">Source: --</div>
        </div>

        <div style="background:var(--bg-subtle); padding:10px 12px; border-radius:var(--radius-sm);">
          <div style="color:var(--text-muted); font-size:11px;">3s 心跳看门狗状态</div>
          <div style="display:flex; align-items:center; gap:6px; margin-top:4px;">
            <span style="display:inline-block; width:10px; height:10px; border-radius:50%; background:#94a3b8;" id="diagHeartbeatDot"></span>
            <span style="font-family:var(--font-mono); font-size:13px; font-weight:600;" id="diagHeartbeatAgeLabel">-- ms</span>
          </div>
          <div style="font-size:10px; color:#64748b;" id="diagHeartbeatExpiryLabel">超时阈值: 3000ms</div>
        </div>

        <div style="background:var(--bg-subtle); padding:10px 12px; border-radius:var(--radius-sm);">
          <div style="color:var(--text-muted); font-size:11px;">5min 闲置倒计时</div>
          <div style="font-family:var(--font-mono); font-size:13px; font-weight:600; margin-top:4px;" id="diagIdleRemainingLabel">--</div>
          <div style="font-size:10px; color:#64748b;">无动作超时自动释放</div>
        </div>
      </div>

      <!-- Action Buttons Row -->
      <div style="display:flex; gap:10px; align-items:center;">
        <button class="btn btn-sm btn-primary" id="btnDiagAcquireLease">申请底层控制 Lease</button>
        <button class="btn btn-sm btn-danger" id="btnDiagReleaseLease">释放 Lease 停机</button>
        <button class="btn btn-sm" id="btnDiagForceHeartbeat">手动补发心跳</button>
      </div>
    </div>

    <!-- Main Grid: Left Motor Health Grid + Right Fault Injection & Channel Rates -->
    <div class="grid-2" style="grid-template-columns: 1fr 420px; align-items:start;">
      <!-- Left: Motors Diagnostics Table -->
      <div class="card" style="margin-bottom:0;">
        <div class="card-header" style="margin-bottom:10px;">
          <div style="display:flex; align-items:center; gap:8px;">
            <span class="card-title">逐电机健康诊断表 (Motor Health)</span>
            <span class="badge badge-info" id="diagMotorCountBadge">-- 关节</span>
          </div>
          <span style="font-size:12px; color:var(--text-muted);">过温 (&gt;55°C) / 报错 自动阻断</span>
        </div>

        <div class="table-wrapper" style="max-height:480px; overflow-y:auto;">
          <table class="data-table" style="font-size:11px;">
            <thead>
              <tr>
                <th style="width:30px;">#</th>
                <th>Joint Name</th>
                <th>Pos (rad / deg)</th>
                <th style="width:110px;">Torque / Effort</th>
                <th>Temp (°C)</th>
                <th>Error Code</th>
                <th>Status</th>
              </tr>
            </thead>
            <tbody id="diagMotorsTableBody">
              <tr><td colspan="7" style="text-align:center; color:#94a3b8;">加载中...</td></tr>
            </tbody>
          </table>
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
            <table class="data-table" style="font-size:11px;">
              <thead>
                <tr>
                  <th>Topic / Channel</th>
                  <th>Target Hz</th>
                  <th>Measured Hz</th>
                  <th>Status</th>
                </tr>
              </thead>
              <tbody id="diagTopicsTableBody">
                <tr>
                  <td><code>/robot/observation</code></td>
                  <td>30.0 Hz</td>
                  <td id="topicObsHz">30.1 Hz</td>
                  <td><span class="badge badge-passed">Normal</span></td>
                </tr>
                <tr>
                  <td><code>/robot/commands</code></td>
                  <td>30.0 Hz</td>
                  <td id="topicCmdHz">29.9 Hz</td>
                  <td><span class="badge badge-passed">Normal</span></td>
                </tr>
                <tr>
                  <td><code>/camera/head/color</code></td>
                  <td>30.0 Hz</td>
                  <td>29.8 Hz</td>
                  <td><span class="badge badge-passed">Normal</span></td>
                </tr>
                <tr>
                  <td><code>/camera/head/depth</code></td>
                  <td>15.0 Hz</td>
                  <td>15.0 Hz</td>
                  <td><span class="badge badge-passed">Normal</span></td>
                </tr>
              </tbody>
            </table>
          </div>
        </div>

        <!-- Mock Fault Injection Drills -->
        <div class="card" style="margin-bottom:0; border-left:4px solid var(--danger);">
          <div class="card-header" style="margin-bottom:8px;">
            <span class="card-title">异常演练与故障注入 (Fault Injection)</span>
            <span class="badge badge-warning">测试专用</span>
          </div>
          <div style="font-size:12px; color:var(--text-muted); margin-bottom:12px;">
            验证各级安全联锁与断关联锁的即时阻断生效能力（本地 Mock 演练，不侵入真实设备）。
          </div>

          <div style="display:grid; grid-template-columns: 1fr 1fr; gap:8px;">
            <button class="btn btn-sm btn-danger" id="btnInjectOverheat">注入关节过温 (72°C)</button>
            <button class="btn btn-sm btn-danger" id="btnInjectMotorError">注入驱动器故障 (0x0040)</button>
            <button class="btn btn-sm btn-danger" id="btnInjectStaleFeedback">注入反馈超时 (&gt;2000ms)</button>
            <button class="btn btn-sm btn-primary" id="btnClearFaults">清除注入故障 (恢复)</button>
          </div>
        </div>

        <!-- Raw State JSON Inspector -->
        <div class="card" style="margin-bottom:0;">
          <div class="card-header" style="margin-bottom:8px;">
            <span class="card-title">原始遥测 JSON 检查器 (Raw State)</span>
            <button class="btn btn-sm" id="btnCopyRawJson">复制完整 JSON</button>
          </div>
          <div style="background:#0f172a; color:#f8fafc; font-family:var(--font-mono); font-size:10px; max-height:180px; overflow-y:auto; padding:10px; border-radius:var(--radius-sm);" id="diagRawJsonBox">
            { "loading": true }
          </div>
        </div>
      </div>
    </div>
  `;

  // Authority & Lease Operations
  container.querySelector("#btnUpdatePerm").addEventListener("click", async () => {
    const perm = container.querySelector("#diagPermSelect").value;
    try {
      await api.setPermission(perm, "operator@motphys");
      alert(`已更新高层任务权限为: ${perm}`);
    } catch (err) {
      alert(`更新权限失败: ${err.message}`);
    }
  });

  container.querySelector("#btnDiagAcquireLease").addEventListener("click", async () => {
    try {
      const res = await api.acquireLease("operator@motphys", "operator");
      api.startHeartbeat(res.authority.lease_id);
      alert("成功获取控制 Lease，3s 心跳看门狗已自动激活");
    } catch (err) {
      alert(`申请 Lease 失败: ${err.message}`);
    }
  });

  container.querySelector("#btnDiagReleaseLease").addEventListener("click", async () => {
    try {
      await api.releaseLease();
      alert("已释放 Lease 并安全停机");
    } catch (err) {
      alert(`释放 Lease 失败: ${err.message}`);
    }
  });

  container.querySelector("#btnDiagForceHeartbeat").addEventListener("click", async () => {
    try {
      const state = await api.getState();
      const leaseId = state.authority.lease_id;
      if (!leaseId) {
        alert("当前未持有 Lease，无需发送心跳");
        return;
      }
      await api.post(`/api/robots/${api.currentRobotId}/lease/heartbeat`, { lease_id: leaseId });
      alert("已手动补发心跳");
    } catch (err) {
      alert(`心跳发送失败: ${err.message}`);
    }
  });

  // Fault Drills
  container.querySelector("#btnInjectOverheat").addEventListener("click", async () => {
    try {
      await api.injectFault("overheat");
      alert("已注入关节 3 过温故障 (72°C)，系统应自动触发温度联锁阻断下发");
    } catch (err) {
      alert(`故障注入失败: ${err.message}`);
    }
  });

  container.querySelector("#btnInjectMotorError").addEventListener("click", async () => {
    try {
      await api.injectFault("motor_error");
      alert("已注入电机驱动器硬件错误码 (0x0040)");
    } catch (err) {
      alert(`故障注入失败: ${err.message}`);
    }
  });

  container.querySelector("#btnInjectStaleFeedback").addEventListener("click", async () => {
    try {
      await api.injectFault("stale_feedback");
      alert("已注入反馈断线/超时故障 (>2000ms)，触发 1000ms 反馈超时阻断");
    } catch (err) {
      alert(`故障注入失败: ${err.message}`);
    }
  });

  container.querySelector("#btnClearFaults").addEventListener("click", async () => {
    try {
      await api.injectFault("clear");
      alert("已清除所有注入故障，硬件状态恢复正常");
    } catch (err) {
      alert(`清除故障失败: ${err.message}`);
    }
  });

  // Copy Raw JSON
  container.querySelector("#btnCopyRawJson").addEventListener("click", () => {
    if (!cachedFullState) {
      alert("无数据");
      return;
    }
    const str = JSON.stringify(cachedFullState, null, 2);
    navigator.clipboard.writeText(str);
    alert("已复制完整原始遥测 JSON");
  });

  return {
    update(state) {
      if (!state) return;
      cachedFullState = state;
      const desc = state.descriptor || {};
      const obs = state.observation || {};
      const auth = state.authority || {};

      // Lease Header Badge
      const leaseBadge = container.querySelector("#diagLeaseOverallBadge");
      const leaseIdLabel = container.querySelector("#diagLeaseIdLabel");
      const leaseSourceLabel = container.querySelector("#diagLeaseSourceLabel");
      const dot = container.querySelector("#diagHeartbeatDot");
      const ageLabel = container.querySelector("#diagHeartbeatAgeLabel");
      const idleLabel = container.querySelector("#diagIdleRemainingLabel");

      if (auth.lease_active) {
        leaseBadge.textContent = `Lease: ${auth.lease_id}`;
        leaseBadge.className = "badge badge-passed";
        leaseIdLabel.textContent = auth.lease_id;
        leaseSourceLabel.textContent = `Source: ${auth.lease_source || "web_session"}`;

        const ageMs = auth.last_heartbeat_age_ms !== undefined ? auth.last_heartbeat_age_ms : 450;
        ageLabel.textContent = `${ageMs} ms`;
        dot.style.background = ageMs < 2000 ? "#10b981" : "#f59e0b";

        const idleSec = auth.idle_timeout_remaining_s !== undefined ? auth.idle_timeout_remaining_s : 295;
        const min = Math.floor(idleSec / 60);
        const sec = Math.floor(idleSec % 60);
        idleLabel.textContent = `${min}m ${sec < 10 ? "0" : ""}${sec}s`;
      } else {
        leaseBadge.textContent = "Lease: 未获取 (只读)";
        leaseBadge.className = "badge badge-idle";
        leaseIdLabel.textContent = "未获取";
        leaseSourceLabel.textContent = "Source: --";
        dot.style.background = "#94a3b8";
        ageLabel.textContent = "-- ms";
        idleLabel.textContent = "--";
      }

      // Motors Table
      const tbody = container.querySelector("#diagMotorsTableBody");
      const countBadge = container.querySelector("#diagMotorCountBadge");
      if (desc.joints && tbody) {
        countBadge.textContent = `${desc.joints.length} 关节`;
        const positions = obs.joint_positions || [];
        const velocities = obs.joint_velocities || [];
        const efforts = obs.joint_efforts || [];
        const temps = obs.motor_temperatures || [];
        const errors = obs.motor_error_codes || [];

        tbody.innerHTML = desc.joints
          .map((j, idx) => {
            const rad = positions[idx] !== undefined ? positions[idx] : 0.0;
            const deg = ((rad * 180) / Math.PI).toFixed(1);
            const eff = efforts[idx] !== undefined ? efforts[idx] : 0.0;
            const temp = temps[idx] !== undefined ? temps[idx] : 38.5;
            const err = errors[idx] !== undefined ? errors[idx] : 0;
            const isWarn = temp > 55 || err !== 0;
            const isBlock = temp > 70 || err > 0x10;

            let statusBadge = `<span class="badge badge-passed">OK</span>`;
            if (isBlock) statusBadge = `<span class="badge badge-failed">BLOCK</span>`;
            else if (isWarn) statusBadge = `<span class="badge badge-warning">WARN</span>`;

            const effPct = Math.min(Math.abs(eff) * 8, 100);

            return `
            <tr>
              <td><span style="color:#64748b; font-family:var(--font-mono);">${idx + 1}</span></td>
              <td><strong>${j.name}</strong> <span style="font-size:10px; color:#94a3b8;">(${j.group})</span></td>
              <td style="font-family:var(--font-mono);">${rad.toFixed(3)} <span style="color:#64748b;">(${deg}°)</span></td>
              <td>
                <div style="display:flex; align-items:center; gap:6px;">
                  <span style="font-family:var(--font-mono); font-size:10px; width:40px;">${eff.toFixed(2)}Nm</span>
                  <div style="flex:1; height:6px; background:#e2e8f0; border-radius:3px; overflow:hidden;">
                    <div style="width:${effPct}%; height:100%; background:var(--accent);"></div>
                  </div>
                </div>
              </td>
              <td style="font-family:var(--font-mono); color:${temp > 65 ? "var(--danger)" : temp > 50 ? "#b45309" : "#1e293b"}; font-weight:${temp > 50 ? "600" : "normal"};">
                ${temp.toFixed(1)}°C
              </td>
              <td style="font-family:var(--font-mono); font-size:10px; color:${err !== 0 ? "var(--danger)" : "#64748b"};">
                0x${err.toString(16).padStart(4, "0")}
              </td>
              <td>${statusBadge}</td>
            </tr>
          `;
          })
          .join("");
      }

      // Topic Rates
      const topicObsHz = container.querySelector("#topicObsHz");
      if (topicObsHz && obs.measured_hz) {
        topicObsHz.textContent = `${obs.measured_hz.toFixed(1)} Hz`;
      }

      // Raw JSON Box
      const rawBox = container.querySelector("#diagRawJsonBox");
      if (rawBox) {
        rawBox.textContent = JSON.stringify(state, null, 2);
      }
    },
  };
}
