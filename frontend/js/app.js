// Motphys Robot Web — Main Application Controller
import { api } from "./api.js";
import { drawer } from "./drawer.js";
import { featureAvailability, applyFeatureNavigation } from "./navigation-state.js";
import { initOverviewTab } from "./tabs/overview.js";
import { initJointsTab } from "./tabs/joints.js?v=LOO1-20261124-navigation-v2";
import { initActionsTab } from "./tabs/actions.js";
import { initNavigationTab } from "./tabs/navigation.js";
import { initPerceptionTab } from "./tabs/perception.js";
import { initPolicyTab } from "./tabs/policy.js";
import { initDiagnosticsTab } from "./tabs/diagnostics.js";

class MotphysRobotApp {
  constructor() {
    this.tabs = {};
    this.activeTabName = "actions";
    this.cachedState = null;
    this.allRobots = [];
  }

  async init() {
    console.log("Initializing Motphys Robot Web Console...");

    // 1. Initialize tabs
    this.tabs.overview = initOverviewTab(document.querySelector("#tabPaneOverview"));
    this.tabs.joints = initJointsTab(document.querySelector("#tabPaneJoints"));
    this.tabs.actions = initActionsTab(document.querySelector("#tabPaneActions"));
    this.tabs.navigation = initNavigationTab(document.querySelector("#tabPaneNavigation"));
    this.tabs.perception = initPerceptionTab(document.querySelector("#tabPanePerception"));
    this.tabs.policy = initPolicyTab(document.querySelector("#tabPanePolicy"));
    this.tabs.diagnostics = initDiagnosticsTab(document.querySelector("#tabPaneDiagnostics"));

    // 2. Setup tab navigation switching
    this._setupTabSwitching();
    const requestedTab = new URLSearchParams(window.location.search).get("tab");
    if (requestedTab && this.tabs[requestedTab]) this.switchTab(requestedTab);
    else this.switchTab("actions");

    // 3. Setup header controls (robot selector, auth pills, E-Stop)
    this._setupHeaderControls();

    // 4. Setup keyboard shortcuts (Space / Shift+Space for E-Stop)
    this._setupKeyboardShortcuts();

    // 5. Setup E-Stop modal & lease modal
    this._setupModals();

    // 6. Fetch robot registry and connect SSE
    await this._loadRobots();

    // 7. Subscribe to real-time telemetry stream
    api.subscribe((state) => this._onTelemetry(state));

    console.log("Motphys Robot Web initialized successfully.");
  }

  _setupTabSwitching() {
    const tabBtns = document.querySelectorAll(".tab-btn");
    tabBtns.forEach((btn) => {
      btn.addEventListener("click", () => {
        const tabName = btn.getAttribute("data-tab");
        this.switchTab(tabName);
      });
    });
  }

  switchTab(tabName) {
    if (!this.tabs[tabName]) return;
    if (this.featureState?.[tabName]?.enabled === false) {
      this._showToast(this.featureState[tabName].reason, "info");
      return;
    }
    this.activeTabName = tabName;

    // Update buttons
    document.querySelectorAll(".tab-btn").forEach((b) => {
      const match = b.getAttribute("data-tab") === tabName;
      b.setAttribute("aria-selected", match ? "true" : "false");
    });

    // Update panels
    document.querySelectorAll(".tab-panel").forEach((p) => {
      p.classList.remove("active");
    });
    const activePanel = document.querySelector(`#tabPane${capitalize(tabName)}`);
    if (activePanel) {
      activePanel.classList.add("active");
    }

    const crumb = document.querySelector("#currentSectionCrumb");
    if (crumb) crumb.textContent = document.querySelector(`[data-tab="${tabName}"]`)?.textContent.trim() || tabName;

    // Trigger update on active tab if state is cached
    if (this.cachedState && this.tabs[tabName] && typeof this.tabs[tabName].update === "function") {
      this.tabs[tabName].update(this.cachedState);
    }
  }

  async _loadRobots() {
    try {
      const res = await api.getRobots();
      this.allRobots = res.robots || [];
      const selector = document.querySelector("#robotSelector");
      selector.innerHTML = "";

      this.allRobots.forEach((r) => {
        const opt = document.createElement("option");
        opt.value = r.id;
        opt.textContent = `${r.name} (${r.robot_type} · ${r.dof} DOF)`;
        selector.appendChild(opt);
      });

      // Default select first or r1pro-mock
      const requestedRobot = new URLSearchParams(window.location.search).get("robot");
      const defaultId = this.allRobots.some(r => r.id === requestedRobot) ? requestedRobot : (this.allRobots.find((r) => r.id === "r1pro-mock") ? "r1pro-mock" : this.allRobots[0]?.id);
      if (defaultId) {
        selector.value = defaultId;
        await api.setRobotId(defaultId, true);
      }
    } catch (err) {
      console.error("Failed to load robots:", err);
      this._showToast(`加载机器人列表失败: ${err.message}`, "danger");
    }
  }

  _setupHeaderControls() {
    // Robot Selector change
    const selector = document.querySelector("#robotSelector");
    selector.addEventListener("change", async (e) => {
      const newId = e.target.value;
      const oldId = api.currentRobotId;
      selector.disabled = true;
      try {
        await api.setRobotId(newId);
        this.cachedState = null;
        drawer.close();
      } catch (err) {
        selector.value = oldId;
        this._showToast(`切换失败，旧机心跳已停止；未向新机授权: ${err.message}`, "danger");
        return;
      } finally {
        selector.disabled = false;
      }
      this._showToast(`已切换至机器人: ${newId}`, "info");

      // Verify capabilities & auto switch away from navigation tab if unsupported
      const robot = this.allRobots.find((r) => r.id === newId);
      if (robot) {
        this._updateCapabilitiesUI(robot.capabilities);
      }
    });

    // Task Permission Pill click -> quick switch
    const authPill = document.querySelector("#headerAuthPill");
    authPill.addEventListener("click", async () => {
      const current = authPill.getAttribute("data-perm") || "operator";
      const next = current === "viewer" ? "operator" : current === "operator" ? "admin" : "viewer";
      try {
        await api.setPermission(next, "operator@motphys");
        this._showToast(`已切换高层任务权限为: ${next}`, "info");
      } catch (err) {
        this._showToast(`权限切换失败: ${err.message}`, "danger");
      }
    });

    // Lease Pill click -> toggle lease
    const leasePill = document.querySelector("#headerLeasePill");
    leasePill.addEventListener("click", async () => {
      const isAcquired = leasePill.getAttribute("data-active") === "true";
      if (!isAcquired) {
        // Open lease acquire modal
        document.querySelector("#modalLeaseAcquire").classList.add("open");
      } else {
        const ok = confirm("确认主动释放底层控制 Lease 并停止所有任务？");
        if (ok) {
          try {
            await api.releaseLease();
            this._showToast("已释放底层 Lease 并安全停机", "info");
          } catch (err) {
            this._showToast(`释放 Lease 失败: ${err.message}`, "danger");
          }
        }
      }
    });

    // Header Global E-Stop Button
    const estopBtn = document.querySelector("#headerEstopBtn");
    estopBtn.addEventListener("click", () => {
      this.triggerEstop("Header E-Stop Button Clicked");
    });
  }

  _setupKeyboardShortcuts() {
    window.addEventListener("keydown", (e) => {
      // Space or Shift+Space for immediate E-Stop bypass
      if (e.code === "Space") {
        // If user is typing in input or textarea, only trigger if Shift/Ctrl/Alt is held
        const inInput = ["INPUT", "TEXTAREA", "SELECT"].includes(e.target.tagName);
        if (!inInput || e.shiftKey || e.ctrlKey || e.altKey) {
          e.preventDefault();
          this.triggerEstop("Keyboard Shortcut Space / Shift+Space");
        }
      }
    });
  }

  async triggerEstop(reason = "Manual Trigger") {
    try {
      const res = await api.triggerEstop(reason);
      this._showToast("本地软件急停已锁存；仅 Mock 停机，不代表硬件断电或远程停机送达", "danger");
      document.querySelector("#modalEstopLatched").classList.add("open");
      document.querySelector("#estopReasonDetail").textContent = `原因: ${reason} (时间: ${new Date().toLocaleTimeString()})`;
    } catch (err) {
      console.error("E-Stop trigger failed:", err);
      alert(`急停触发失败: ${err.message}`);
    }
  }

  _setupModals() {
    // E-Stop Latched Modal
    const estopModal = document.querySelector("#modalEstopLatched");
    document.querySelector("#btnClearEstopModalBtn").addEventListener("click", async () => {
      try {
        await api.clearEstop("operator@motphys");
        estopModal.classList.remove("open");
        this._showToast("急停状态已解除，系统恢复至只读待机模式", "success");
      } catch (err) {
        alert(`解除急停失败: ${err.message}`);
      }
    });

    // Lease Acquisition Modal
    const leaseModal = document.querySelector("#modalLeaseAcquire");
    document.querySelector("#btnCancelLeaseAcquire").addEventListener("click", () => {
      leaseModal.classList.remove("open");
    });

    document.querySelector("#btnConfirmLeaseAcquire").addEventListener("click", async () => {
      const operatorName = document.querySelector("#leaseOperatorInput").value.trim() || "operator@motphys";
      const taskPerm = document.querySelector("#leasePermSelect").value;
      const confirmCheck = document.querySelector("#chkConfirmLeaseSafety").checked;

      if (!confirmCheck) {
        alert("请先勾选确认安全事项：已确认机器人周边 1.5 米无障碍与人员干涉。");
        return;
      }

      try {
        const res = await api.acquireLease(operatorName, taskPerm);
        const leaseId = res.authority.lease_id;
        api.startHeartbeat(leaseId);
        leaseModal.classList.remove("open");
        this._showToast(`已获取控制 Lease [${leaseId}]，3s 心跳看门狗已激活`, "success");
      } catch (err) {
        alert(`申请 Lease 失败: ${err.message}`);
      }
    });
  }

  _updateCapabilitiesUI(caps) {
    if (!caps) return;
    this.featureState = featureAvailability(caps);
    applyFeatureNavigation(document.querySelectorAll(".primary-nav [data-tab]"), this.featureState);
    const note = document.querySelector("#featureAvailabilityNote");
    if (note) {
      const unavailable = Object.values(this.featureState).filter(feature => !feature.enabled);
      note.textContent = unavailable.length ? unavailable.map(feature => `${feature.label}：${feature.reason}`).join("；") : "全部功能入口可用 · 实际操作仍受 Mock 权限和 Lease 门禁约束";
    }
    if (this.featureState[this.activeTabName]?.enabled === false) this.switchTab("overview");
  }

  _onTelemetry(state) {
    if (!state) return;
    this.cachedState = state;
    const desc = state.descriptor || {};
    const obs = state.observation || {};
    const auth = state.authority || {};

    if (!obs.connected) api.stopHeartbeat();

    // 1. Update Header Mode Pill (MOCK / SIM / REAL)
    const modePill = document.querySelector("#headerModePill");
    const mode = desc.integration_status === "unimplemented" ? "UNAVAILABLE" : (desc.mode || "mock").toUpperCase();
    if (modePill) {
      modePill.textContent = mode;
      modePill.className = `pill-badge ${mode === "REAL" ? "pill-real" : mode === "SIM" ? "pill-sim" : "pill-mock"}`;
    }

    // 2. Update Header Auth Pill
    const authPill = document.querySelector("#headerAuthPill");
    const perm = auth.task_permission || "operator";
    if (authPill) {
      authPill.setAttribute("data-perm", perm);
      authPill.textContent = `Demo role: ${perm.toUpperCase()}`;
    }

    // 3. Update Header Lease Pill
    const leasePill = document.querySelector("#headerLeasePill");
    if (leasePill) {
      if (auth.lease_active) {
        leasePill.setAttribute("data-active", "true");
        leasePill.className = "pill-badge pill-auth active";
        leasePill.innerHTML = `<span style="display:inline-block; width:8px; height:8px; border-radius:50%; background:#10b981; animation:pulse-green 1.2s infinite;"></span> Mock Lease: ${auth.lease_id || "Active"}`;
      } else {
        leasePill.setAttribute("data-active", "false");
        leasePill.className = "pill-badge pill-auth";
        leasePill.innerHTML = `<span style="display:inline-block; width:8px; height:8px; border-radius:50%; background:#94a3b8;"></span> Mock Lease: 只读未授权`;
      }
    }

    // 4. Update Header E-Stop Button state
    const estopBtn = document.querySelector("#headerEstopBtn");
    if (estopBtn) {
      if (auth.estop_latched) {
        estopBtn.classList.add("latched");
        estopBtn.textContent = "急停锁死中 (已触发)";
      } else {
        estopBtn.classList.remove("latched");
        estopBtn.innerHTML = `
          <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round">
            <circle cx="12" cy="12" r="10"></circle>
            <line x1="12" y1="8" x2="12" y2="12"></line>
            <line x1="12" y1="16" x2="12.01" y2="16"></line>
          </svg>
          软件急停 (Space)
        `;
      }
    }

    // 5. Update Connection & Age Indicator
    const ageEl = document.querySelector("#headerConnAge");
    if (ageEl) {
      if (obs.connected) {
        ageEl.textContent = `${obs.feedback_age_ms || 18}ms · ${obs.measured_hz || 30}Hz`;
        ageEl.style.color = "var(--success)";
      } else {
        ageEl.textContent = `断联 · ${obs.feedback_age_ms || 2400}ms`;
        ageEl.style.color = "var(--danger)";
      }
    }

    // 6. Update Capabilities visibility
    this._updateCapabilitiesUI(desc.capabilities);

    // 7. Route update to active tab
    if (this.tabs[this.activeTabName] && typeof this.tabs[this.activeTabName].update === "function") {
      this.tabs[this.activeTabName].update(state);
    }
  }

  _showToast(msg, type = "info") {
    let container = document.querySelector("#toastContainer");
    if (!container) {
      container = document.createElement("div");
      container.id = "toastContainer";
      container.style.cssText = "position:fixed; bottom:20px; right:20px; z-index:9999; display:flex; flex-direction:column; gap:8px; pointer-events:none;";
      document.body.appendChild(container);
    }

    const toast = document.createElement("div");
    toast.style.cssText = `
      padding: 10px 16px;
      border-radius: var(--radius-md);
      font-size: 13px;
      font-weight: 500;
      color: #ffffff;
      background: ${type === "danger" ? "#dc2626" : type === "success" ? "#16a34a" : "#0f172a"};
      box-shadow: 0 4px 12px rgba(0,0,0,0.15);
      pointer-events: auto;
      transition: all 0.3s ease;
      opacity: 0;
      transform: translateY(10px);
    `;
    toast.textContent = msg;
    container.appendChild(toast);

    requestAnimationFrame(() => {
      toast.style.opacity = "1";
      toast.style.transform = "translateY(0)";
    });

    setTimeout(() => {
      toast.style.opacity = "0";
      toast.style.transform = "translateY(10px)";
      setTimeout(() => toast.remove(), 300);
    }, 3200);
  }
}

function capitalize(s) {
  if (!s) return "";
  return s.charAt(0).toUpperCase() + s.slice(1);
}

// Instantiate and boot on DOM ready
document.addEventListener("DOMContentLoaded", () => {
  const app = new MotphysRobotApp();
  window.__MOTPHYS_ROBOT_APP__ = app;
  app.init();
});
