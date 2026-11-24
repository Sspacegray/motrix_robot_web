// Tab 1: Overview
import { api } from "../api.js";
import { drawer } from "../drawer.js";

export function initOverviewTab(container) {
  container.innerHTML = `
    <!-- Top KPI Cards -->
    <div class="grid-4" style="margin-bottom: 20px;">
      <div class="kpi-card">
        <div class="kpi-label">连接与反馈</div>
        <div class="kpi-val" id="kpiConn">-- <span class="kpi-sub" id="kpiAge">-- ms</span></div>
        <div class="kpi-sub" id="kpiHz">-- Hz · DDS/CAN</div>
      </div>
      <div class="kpi-card">
        <div class="kpi-label">底层租约 (Lease)</div>
        <div class="kpi-val" id="kpiLease">--</div>
        <div class="kpi-sub" id="kpiLeaseSub">心跳 3s 看门狗 · 5min 闲置</div>
      </div>
      <div class="kpi-card">
        <div class="kpi-label">电池与电源</div>
        <div class="kpi-val" id="kpiBattery">-- %</div>
        <div class="kpi-sub" id="kpiPower">-- V · -- A</div>
      </div>
      <div class="kpi-card">
        <div class="kpi-label">电机与联锁诊断</div>
        <div class="kpi-val" id="kpiMotorStatus">--</div>
        <div class="kpi-sub" id="kpiInterlocks">0 个阻断 · 0 个告警</div>
      </div>
    </div>

    <!-- Architecture & Network Scope Banner -->
    <div class="card" style="border-left: 4px solid var(--accent);">
      <div class="card-header" style="margin-bottom:8px; padding-bottom:6px;">
        <span class="card-title">高层控制与感知架构对齐（对齐路线图规范）</span>
        <span class="badge badge-passed" id="overviewConnectorBadge">Connector: mock</span>
      </div>
      <p style="font-size:13px; color:var(--text-muted); margin-bottom:8px;" id="overviewNetworkNote">
        加载网络与架构拓扑说明...
      </p>
      <div style="font-size:12px; color:#64748b; background:var(--bg-subtle); padding:8px 12px; border-radius:var(--radius-sm);" id="overviewSafetyNote">
        安全原则：高层任务权限与底层租约解耦；统一平台不自建控制底座；Piper 唯一 CAN 写者归 Edge，严禁双写。
      </div>
    </div>

    <!-- Recent Execution Jobs (Motphys Loop Layout) -->
    <div class="card">
      <div class="card-header">
        <span class="card-title">最近执行任务 (Recent Jobs)</span>
        <button class="btn btn-sm" id="overviewViewAllJobsBtn">查看全部 Actions & Jobs</button>
      </div>
      <div class="table-wrapper">
        <table class="data-table">
          <thead>
            <tr>
              <th style="width:110px;">State</th>
              <th style="width:100px;">Kind</th>
              <th>Flow</th>
              <th>Run ID</th>
              <th>Created</th>
              <th>Operator</th>
            </tr>
          </thead>
          <tbody id="overviewJobsBody">
            <tr><td colspan="6" style="text-align:center; color:#94a3b8;">加载中...</td></tr>
          </tbody>
        </table>
      </div>
    </div>
  `;

  container.querySelector("#overviewViewAllJobsBtn").addEventListener("click", () => {
    const actTab = document.querySelector('[data-tab="actions"]');
    if (actTab) actTab.click();
  });

  return {
    update(state) {
      if (!state) return;
      const obs = state.observation || {};
      const auth = state.authority || {};
      const desc = state.descriptor || {};

      // KPI 1: Connection & Age
      const connEl = container.querySelector("#kpiConn");
      const ageEl = container.querySelector("#kpiAge");
      const hzEl = container.querySelector("#kpiHz");
      if (connEl && ageEl && hzEl) {
        if (obs.connected) {
          connEl.innerHTML = `<span style="color:var(--success);">Connected</span>`;
          ageEl.textContent = `${obs.feedback_age_ms || 18} ms`;
          hzEl.textContent = `${obs.measured_hz || 30} Hz · ${String(desc.connector_type || "mock").toUpperCase()}`;
        } else {
          connEl.innerHTML = `<span style="color:var(--danger);">Disconnected</span>`;
          ageEl.textContent = `${obs.feedback_age_ms || 2400} ms`;
          hzEl.textContent = `0 Hz`;
        }
      }

      // KPI 2: Lease
      const leaseEl = container.querySelector("#kpiLease");
      const leaseSubEl = container.querySelector("#kpiLeaseSub");
      if (leaseEl && leaseSubEl) {
        if (auth.lease_active) {
          leaseEl.innerHTML = `<span style="color:var(--success); font-size:18px;">Active (${auth.lease_remaining_s || 0}s)</span>`;
          leaseSubEl.textContent = `持有者: ${auth.operator_name || "operator"} · 心跳: ${auth.heartbeat_age_s || 0}s`;
        } else {
          leaseEl.innerHTML = `<span style="color:var(--text-muted); font-size:18px;">未授权 (只读)</span>`;
          leaseSubEl.textContent = `高层权限: ${String(auth.task_permission || "operator").toUpperCase()} · 需申请 Lease`;
        }
      }

      // KPI 3: Battery
      const bat = obs.battery || {};
      const kpiBat = container.querySelector("#kpiBattery");
      const kpiPow = container.querySelector("#kpiPower");
      if (kpiBat) kpiBat.textContent = `${bat.percentage || 88} %`;
      if (kpiPow) kpiPow.textContent = `${bat.voltage_v || 48.2} V · ${bat.current_a || -2.4} A`;

      // KPI 4: Motors
      const motors = obs.motors || [];
      const blocks = motors.filter(m => m.severity === "block");
      const warns = motors.filter(m => m.severity === "warn");
      const motorStatusEl = container.querySelector("#kpiMotorStatus");
      if (motorStatusEl) {
        if (blocks.length > 0) {
          motorStatusEl.innerHTML = `<span style="color:var(--danger);">${blocks.length} 轴阻断</span>`;
        } else if (warns.length > 0) {
          motorStatusEl.innerHTML = `<span style="color:var(--warning);">${warns.length} 轴告警</span>`;
        } else {
          motorStatusEl.innerHTML = `<span style="color:var(--success);">全部正常</span>`;
        }
      }
      const kpiInterlocks = container.querySelector("#kpiInterlocks");
      if (kpiInterlocks) kpiInterlocks.textContent = `${blocks.length} 个阻断 · ${warns.length} 个告警 · ${motors.length} 轴`;

      // Notes
      const badgeEl = container.querySelector("#overviewConnectorBadge");
      if (badgeEl) badgeEl.textContent = `Connector: ${desc.connector_type || "mock"} (${desc.mode || "mock"})`;
      const netNote = container.querySelector("#overviewNetworkNote");
      if (netNote) netNote.textContent = desc.network_note || "无特定网络限制";
      const safeNote = container.querySelector("#overviewSafetyNote");
      if (safeNote) safeNote.textContent = desc.safety_note || "安全原则受控下发";

      // Recent Jobs Table
      const jobsBody = container.querySelector("#overviewJobsBody");
      const recent = state.recent_jobs || [];
      if (recent.length > 0) {
        jobsBody.innerHTML = recent.map(j => `
          <tr data-job-id="${j.job_id}">
            <td><span class="badge badge-${j.state.toLowerCase()}">${j.state}</span></td>
            <td><span style="font-weight:600; color:var(--text-muted);">${j.kind}</span></td>
            <td style="font-weight:600;">${j.flow_name}</td>
            <td style="font-family:var(--font-mono); font-size:12px; color:var(--text-muted);">${j.run_id}</td>
            <td style="font-size:12px; color:var(--text-muted);">${j.created_at ? j.created_at.slice(11, 19) : "--"}</td>
            <td style="font-size:12px;">${j.operator || "--"}</td>
          </tr>
        `).join("");

        jobsBody.querySelectorAll("tr").forEach(tr => {
          tr.addEventListener("click", () => {
            const jid = tr.getAttribute("data-job-id");
            if (jid) drawer.open(jid);
          });
        });
      } else {
        jobsBody.innerHTML = `<tr><td colspan="6" style="text-align:center; color:#94a3b8;">暂无执行任务</td></tr>`;
      }
    }
  };
}
