// Motphys Loop Execution Task Drawer
import { api } from "./api.js";

class ExecutionDrawer {
  constructor() {
    this.panel = null;
    this.backdrop = null;
    this.currentJobId = null;
    this.pollInterval = null;
    this.followLogs = true;
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
    this.panel.innerHTML = `
      <div class="drawer-header">
        <div class="drawer-title-wrap">
          <span class="badge" id="drawerBadge">Pending</span>
          <span class="drawer-title" id="drawerJobId">--</span>
        </div>
        <div class="drawer-actions">
          <button class="btn btn-sm" id="drawerRetryBtn">Retry</button>
          <button class="btn btn-sm btn-danger" id="drawerCancelBtn" style="display:none;">Cancel</button>
          <button class="drawer-close-btn" id="drawerCloseBtn" title="Close (Esc)">✕</button>
        </div>
      </div>
      <div class="drawer-body">
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
            <span class="meta-label">Created</span>
            <span class="meta-val" id="drawerCreated">--</span>
          </div>
          <div class="meta-item">
            <span class="meta-label">Started</span>
            <span class="meta-val" id="drawerStarted">--</span>
          </div>
          <div class="meta-item">
            <span class="meta-label">Finished</span>
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
      if (!this.currentJobId) return;
      try {
        const res = await api.retryJob(this.currentJobId);
        this.open(res.job.job_id);
      } catch (err) {
        alert("Retry failed: " + err.message);
      }
    });

    const cancelBtn = this.panel.querySelector("#drawerCancelBtn");
    cancelBtn.addEventListener("click", async () => {
      if (!this.currentJobId) return;
      try {
        await api.cancelJob(this.currentJobId);
        this._fetchJob();
      } catch (err) {
        alert("Cancel failed: " + err.message);
      }
    });

    const followSwitch = this.panel.querySelector("#drawerFollowSwitch");
    followSwitch.addEventListener("change", (e) => {
      this.followLogs = e.target.checked;
    });

    window.addEventListener("keydown", (e) => {
      if (e.key === "Escape" && this.panel.classList.contains("open")) {
        this.close();
      }
    });
  }

  async open(jobId) {
    this.currentJobId = jobId;
    this.backdrop.classList.add("open");
    this.panel.classList.add("open");
    await this._fetchJob();

    if (this.pollInterval) clearInterval(this.pollInterval);
    this.pollInterval = setInterval(() => this._fetchJob(), 1000);
  }

  close() {
    this.backdrop.classList.remove("open");
    this.panel.classList.remove("open");
    if (this.pollInterval) {
      clearInterval(this.pollInterval);
      this.pollInterval = null;
    }
  }

  async _fetchJob() {
    if (!this.currentJobId) return;
    try {
      const data = await api.getJob(this.currentJobId);
      this._render(data.job);
      if (data.job.state !== "Running" && this.pollInterval) {
        clearInterval(this.pollInterval);
        this.pollInterval = null;
      }
    } catch (err) {
      console.warn("Fetch job failed", err);
    }
  }

  _render(job) {
    const badge = this.panel.querySelector("#drawerBadge");
    badge.className = `badge badge-${job.state.toLowerCase()}`;
    badge.textContent = job.state;

    this.panel.querySelector("#drawerJobId").textContent = job.job_id;
    this.panel.querySelector("#drawerFlow").textContent = job.flow_name;
    this.panel.querySelector("#drawerRun").textContent = job.run_id;
    this.panel.querySelector("#drawerCreated").textContent = job.created_at || "--";
    this.panel.querySelector("#drawerStarted").textContent = job.started_at || "--";
    this.panel.querySelector("#drawerFinished").textContent = job.finished_at || "--";
    this.panel.querySelector("#drawerExit").textContent = job.exit_code !== null ? job.exit_code : "--";
    this.panel.querySelector("#drawerPipeline").textContent = job.pipeline_sha;

    const cancelBtn = this.panel.querySelector("#drawerCancelBtn");
    cancelBtn.style.display = job.state === "Running" ? "inline-flex" : "none";

    // Render Steps
    const stepsBody = this.panel.querySelector("#drawerStepsBody");
    if (job.steps && job.steps.length > 0) {
      stepsBody.innerHTML = job.steps.map(st => `
        <tr>
          <td style="font-weight:600;">${st.step}</td>
          <td><span class="badge badge-${st.phase}">${st.phase}</span></td>
          <td style="font-family:var(--font-mono); font-size:12px; color:var(--text-muted);">${st.detail || "--"}</td>
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
