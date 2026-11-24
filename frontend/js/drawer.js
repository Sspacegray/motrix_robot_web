// Motphys Loop Execution Task Drawer
import { api } from "./api.js";
import { escapeHtml, formatLocalTime } from "./ui-format.js";

class ExecutionDrawer {
  constructor() {
    this.panel = null;
    this.backdrop = null;
    this.currentJobId = null;
    this.pollInterval = null;
    this.followLogs = true;
    this.currentJob = null;
    this._initDOMElements();
  }

  _initDOMElements() {
    this.backdrop = document.createElement("div");
    this.backdrop.className = "drawer-backdrop";
    this.backdrop.addEventListener("click", () => this.close());

    this.panel = document.createElement("div");
    this.panel.className = "drawer-panel";
    this.panel.setAttribute("role", "dialog");
    this.panel.setAttribute("aria-label", "Execution details");
    this.panel.setAttribute("aria-modal", "true");
    this.panel.tabIndex = -1;
    this.panel.inert = true;
    this.panel.innerHTML = `
      <div class="drawer-header">
        <div class="drawer-title-wrap">
          <span class="drawer-title">Execution task</span>
        </div>
        <div class="drawer-actions">
          <button class="btn btn-sm" id="drawerRetryBtn" disabled title="重试仅执行无运动指令的 Mock dry-run">Retry dry-run</button>
          <button class="btn btn-sm btn-danger" id="drawerCancelBtn" style="display:none;">Cancel</button>
          <button class="drawer-close-btn" id="drawerCloseBtn" title="Close (Esc)">✕</button>
        </div>
      </div>
      <div class="drawer-body">
        <div class="drawer-record"><span class="badge" id="drawerBadge">Pending</span><span id="drawerJobId">--</span><span>LOCAL MOCK</span></div>
        <div class="meta-grid">
          <div class="meta-item">
            <span class="meta-label">Flow</span>
            <span class="meta-val" id="drawerFlow">--</span>
          </div>
          <div class="meta-item">
            <span class="meta-label">Run</span>
            <span class="meta-val" id="drawerRun">--</span>
          </div>
          <div class="meta-item">
            <span class="meta-label">Created · UTC+8</span>
            <span class="meta-val" id="drawerCreated">--</span>
          </div>
          <div class="meta-item">
            <span class="meta-label">Started · UTC+8</span>
            <span class="meta-val" id="drawerStarted">--</span>
          </div>
          <div class="meta-item">
            <span class="meta-label">Finished · UTC+8</span>
            <span class="meta-val" id="drawerFinished">--</span>
          </div>
          <div class="meta-item">
            <span class="meta-label">Exit code</span>
            <span class="meta-val" id="drawerExit">--</span>
          </div>
          <div class="meta-item" style="grid-column: span 2;">
            <span class="meta-label">Pipeline</span>
            <span class="meta-val" id="drawerPipeline">--</span>
          </div>
        </div>

        <h4 style="font-size:14px; font-weight:700; margin-bottom:10px;">Steps</h4>
        <div class="table-wrapper" style="margin-bottom:20px;">
          <table class="data-table">
            <thead>
              <tr>
                <th style="width:140px;">Step</th>
                <th style="width:100px;">Phase</th>
                <th>Jobs (detail, digest, reason)</th>
              </tr>
            </thead>
            <tbody id="drawerStepsBody">
              <tr><td colspan="3" style="text-align:center; color:#94a3b8;">No steps recorded</td></tr>
            </tbody>
          </table>
        </div>

        <div class="terminal-box">
          <div class="terminal-header">
            <span style="font-weight:600; color:#cbd5e1;">Logs</span>
            <label class="switch-control" style="font-size:12px; color:#cbd5e1;">
              Follow
              <input type="checkbox" id="drawerFollowSwitch" checked switch>
            </label>
          </div>
          <pre class="terminal-logs" id="drawerLogsPre">Loading logs...</pre>
        </div>
      </div>
    `;

    document.body.appendChild(this.backdrop);
    document.body.appendChild(this.panel);

    // Event listeners
    this.panel.querySelector("#drawerCloseBtn").addEventListener("click", () => this.close());
    this.panel.querySelector("#drawerRetryBtn").addEventListener("click", async () => {
      if (!this.currentJobId || !this.currentJob?.workflow_id || this.currentJob.state === "Running") return;
      const button = this.panel.querySelector("#drawerRetryBtn");
      button.disabled = true;
      try {
        const res = await api.retryJob(this.currentJobId, true);
        this.open(res.job.job_id);
      } catch (err) {
        alert("Retry failed: " + err.message);
        this._fetchJob();
      }
    });

    const cancelBtn = this.panel.querySelector("#drawerCancelBtn");
    cancelBtn.addEventListener("click", async () => {
      if (!this.currentJobId) return;
      cancelBtn.disabled = true;
      const jobId = this.currentJobId;
      try {
        await api.cancelJob(jobId);
        if (this.currentJobId === jobId) await this._fetchJob();
      } catch (err) {
        alert("Cancel failed: " + err.message);
        if (this.currentJobId === jobId) await this._fetchJob();
      } finally {
        cancelBtn.disabled = false;
      }
    });

    const followSwitch = this.panel.querySelector("#drawerFollowSwitch");
    followSwitch.addEventListener("change", (e) => {
      this.followLogs = e.target.checked;
    });

    this.panel.addEventListener("keydown", (event) => {
      if (event.key !== "Tab") return;
      const controls = [...this.panel.querySelectorAll("button:not(:disabled), input:not(:disabled)")].filter(el => el.offsetParent !== null);
      const first = controls[0], last = controls[controls.length - 1];
      if (event.shiftKey && document.activeElement === first) { event.preventDefault(); last?.focus(); }
      else if (!event.shiftKey && document.activeElement === last) { event.preventDefault(); first?.focus(); }
    });
    window.addEventListener("keydown", (e) => {
      if (e.key === "Escape" && this.panel.classList.contains("open")) {
        this.close();
      }
    });
  }

  async open(jobId) {
    if (this.pollInterval) clearInterval(this.pollInterval);
    this.currentJobId = jobId;
    this.currentJob = null;
    this.previousFocus = document.activeElement;
    this.panel.inert = false;
    document.body.classList.add("execution-open");
    this.panel.querySelector("#drawerRetryBtn").disabled = true;
    this.panel.querySelector("#drawerCancelBtn").style.display = "none";
    this.panel.querySelector("#drawerLogsPre").textContent = "Loading local Mock logs…";
    this.backdrop.classList.add("open");
    this.panel.classList.add("open");
    this.panel.querySelector("#drawerCloseBtn").focus();
    await this._fetchJob();

    if (this.pollInterval) clearInterval(this.pollInterval);
    if (this.currentJob?.state === "Running") this.pollInterval = setInterval(() => this._fetchJob(), 1000);
  }

  close() {
    this.backdrop.classList.remove("open");
    this.panel.classList.remove("open");
    this.panel.inert = true;
    this.currentJobId = null;
    document.body.classList.remove("execution-open");
    this.previousFocus?.focus();
    if (this.pollInterval) {
      clearInterval(this.pollInterval);
      this.pollInterval = null;
    }
  }

  async _fetchJob() {
    if (!this.currentJobId) return;
    try {
      const jobId = this.currentJobId;
      const data = await api.getJob(jobId);
      if (jobId !== this.currentJobId) return;
      this.currentJob = data.job;
      this._render(data.job);
      if (data.job.state !== "Running" && this.pollInterval) {
        clearInterval(this.pollInterval);
        this.pollInterval = null;
      }
    } catch (err) {
      console.warn("Fetch job failed", err);
      this.panel.querySelector("#drawerLogsPre").textContent = "Unable to load local job: " + err.message;
    }
  }

  _render(job) {
    const badge = this.panel.querySelector("#drawerBadge");
    badge.className = `badge badge-${job.state.toLowerCase()}`;
    badge.textContent = job.state;

    this.panel.querySelector("#drawerJobId").textContent = job.job_id;
    this.panel.querySelector("#drawerFlow").textContent = job.flow_name;
    this.panel.querySelector("#drawerRun").textContent = job.run_id;
    this.panel.querySelector("#drawerCreated").textContent = formatLocalTime(job.created_at);
    this.panel.querySelector("#drawerStarted").textContent = formatLocalTime(job.started_at);
    this.panel.querySelector("#drawerFinished").textContent = formatLocalTime(job.finished_at);
    this.panel.querySelector("#drawerExit").textContent = job.exit_code !== null ? job.exit_code : "--";
    this.panel.querySelector("#drawerPipeline").textContent = job.pipeline_sha;

    this.panel.querySelector("#drawerRetryBtn").disabled = !job.workflow_id || job.state === "Running";
    const cancelBtn = this.panel.querySelector("#drawerCancelBtn");
    cancelBtn.style.display = job.state === "Running" ? "inline-flex" : "none";

    // Render Steps
    const stepsBody = this.panel.querySelector("#drawerStepsBody");
    if (job.steps && job.steps.length > 0) {
      stepsBody.innerHTML = job.steps.map(st => `
        <tr>
          <td style="font-weight:600;">${escapeHtml(st.step)}</td>
          <td><span class="badge badge-${escapeHtml(st.phase)}">${escapeHtml(st.phase)}</span></td>
          <td style="font-family:var(--font-mono); font-size:12px; color:var(--text-muted);">${escapeHtml(st.detail || "--")}</td>
        </tr>
      `).join("");
    } else {
      stepsBody.innerHTML = `<tr><td colspan="3" style="text-align:center; color:#94a3b8;">No steps</td></tr>`;
    }

    // Render Logs
    const logsPre = this.panel.querySelector("#drawerLogsPre");
    logsPre.textContent = (job.logs || []).join("\n") || "No logs available";

    if (this.followLogs) {
      logsPre.scrollTop = logsPre.scrollHeight;
    }
  }
}

export const drawer = new ExecutionDrawer();
