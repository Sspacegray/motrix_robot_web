// Motphys Robot Web — Client API & Telemetry Manager

export class RobotApi {
  constructor() {
    this.currentRobotId = "r1pro-mock";
    this.listeners = new Set();
    this.sse = null;
    this.heartbeatTimer = null;
    this.cachedState = null;
  }

  setRobotId(id, force = false) {
    if (!force && this.currentRobotId === id && this.sse) return;
    this.currentRobotId = id;
    this.connectSSE();
    this.getState().then((s) => this._notify(s)).catch(() => {});
  }

  subscribe(callback) {
    this.listeners.add(callback);
    if (this.cachedState) callback(this.cachedState);
    return () => this.listeners.delete(callback);
  }

  _notify(state) {
    this.cachedState = state;
    for (const cb of this.listeners) {
      try {
        cb(state);
      } catch (err) {
        console.error("Telemetry listener error", err);
      }
    }
  }

  connectSSE() {
    if (this.sse) {
      this.sse.close();
      this.sse = null;
    }
    const url = `/api/events?robot_id=${encodeURIComponent(this.currentRobotId)}`;
    this.sse = new EventSource(url);

    this.sse.onmessage = (evt) => {
      try {
        const data = JSON.parse(evt.data);
        this._notify(data);
      } catch (err) {
        console.warn("SSE JSON parse failed", err);
      }
    };

    this.sse.onerror = (err) => {
      console.warn("SSE connection interrupted, retrying...", err);
    };
  }

  startHeartbeat(leaseId) {
    this.stopHeartbeat();
    this.heartbeatTimer = setInterval(async () => {
      try {
        await this.post(`/api/robots/${this.currentRobotId}/lease/heartbeat`, { lease_id: leaseId });
      } catch (err) {
        console.warn("Heartbeat failed", err);
      }
    }, 1000);
  }

  stopHeartbeat() {
    if (this.heartbeatTimer) {
      clearInterval(this.heartbeatTimer);
      this.heartbeatTimer = null;
    }
  }

  async get(path) {
    const res = await fetch(path);
    if (!res.ok) {
      const err = await res.json().catch(() => ({ detail: res.statusText }));
      throw new Error(err.detail || res.statusText);
    }
    return res.json();
  }

  async post(path, body = {}) {
    const res = await fetch(path, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });
    if (!res.ok) {
      const err = await res.json().catch(() => ({ detail: res.statusText }));
      throw new Error(err.detail || res.statusText);
    }
    return res.json();
  }

  async delete(path) {
    const res = await fetch(path, { method: "DELETE" });
    if (!res.ok) {
      const err = await res.json().catch(() => ({ detail: res.statusText }));
      throw new Error(err.detail || res.statusText);
    }
    return res.json();
  }

  // Robot management
  getRobots() {
    return this.get("/api/robots");
  }

  getState() {
    return this.get(`/api/robots/${this.currentRobotId}/state`);
  }

  // Lease & Safety Authority
  setPermission(permission, operatorName) {
    return this.post(`/api/robots/${this.currentRobotId}/permission`, {
      task_permission: permission,
      operator_name: operatorName,
    });
  }

  acquireLease(operatorName, taskPermission = "operator") {
    return this.post(`/api/robots/${this.currentRobotId}/lease/acquire`, {
      operator_name: operatorName,
      task_permission: taskPermission,
      confirm_safety: true,
    });
  }

  releaseLease() {
    this.stopHeartbeat();
    return this.post(`/api/robots/${this.currentRobotId}/lease/release`);
  }

  triggerEstop(reason = "Operator manual E-Stop") {
    return this.post(`/api/robots/${this.currentRobotId}/estop`, {
      source: "web_ui",
      reason,
    });
  }

  clearEstop(operatorName = "operator@motphys") {
    return this.post(`/api/robots/${this.currentRobotId}/estop/clear`, {
      operator_name: operatorName,
    });
  }

  // Motion
  sendJointTargets(targets, durationS = 1.2) {
    return this.post(`/api/robots/${this.currentRobotId}/joints/targets`, {
      targets,
      duration_s: durationS,
    });
  }

  sendEEStep(arm, dx, dy, dz, drollDeg = 0, dpitchDeg = 0, dyawDeg = 0) {
    return this.post(`/api/robots/${this.currentRobotId}/ee/step`, {
      arm,
      dx,
      dy,
      dz,
      droll_deg: drollDeg,
      dpitch_deg: dpitchDeg,
      dyaw_deg: dyawDeg,
    });
  }

  // Navigation
  setNavMode(mode) {
    return this.post(`/api/robots/${this.currentRobotId}/navigation/mode`, { mode });
  }

  sendNavGoal(x, y, yaw = 0) {
    return this.post(`/api/robots/${this.currentRobotId}/navigation/goal`, { x, y, yaw });
  }

  cancelNav() {
    return this.post(`/api/robots/${this.currentRobotId}/navigation/cancel`);
  }

  sendChassisVelocity(vx, vy, omega, durationS = 0.4) {
    return this.post(`/api/robots/${this.currentRobotId}/navigation/chassis`, {
      vx,
      vy,
      omega,
      duration_s: durationS,
    });
  }

  startLocalizationSpin(active = true) {
    return this.post(`/api/robots/${this.currentRobotId}/navigation/spin`, { active });
  }

  // Perception
  controlPerception(action, payload = {}) {
    return this.post(`/api/robots/${this.currentRobotId}/perception/${action}`, payload);
  }

  // Policy & Inference
  controlPolicy(action, payload = {}) {
    return this.post(`/api/robots/${this.currentRobotId}/policy/${action}`, payload);
  }

  // Fault injection drills
  injectFault(faultType) {
    return this.post(`/api/robots/${this.currentRobotId}/faults/${faultType}`);
  }

  // Actions & Workflows
  listActions() {
    return this.get(`/api/actions?robot_id=${encodeURIComponent(this.currentRobotId)}`);
  }

  saveAction(data) {
    return this.post("/api/actions", data);
  }

  deleteAction(actionId) {
    return this.delete(`/api/actions/${actionId}`);
  }

  importR1Actions(payload) {
    return this.post(`/api/actions/import-r1pro?robot_id=${encodeURIComponent(this.currentRobotId)}`, payload);
  }

  listWorkflows() {
    return this.get(`/api/workflows?robot_id=${encodeURIComponent(this.currentRobotId)}`);
  }

  saveWorkflow(data) {
    return this.post("/api/workflows", data);
  }

  deleteWorkflow(workflowId) {
    return this.delete(`/api/workflows/${workflowId}`);
  }

  runWorkflow(workflowId, dryRun = false) {
    return this.post(`/api/workflows/${workflowId}/run`, { dry_run: dryRun });
  }

  listJobs() {
    return this.get(`/api/jobs?robot_id=${encodeURIComponent(this.currentRobotId)}`);
  }

  getJob(jobId) {
    return this.get(`/api/jobs/${jobId}`);
  }

  retryJob(jobId, dryRun = false) {
    return this.post(`/api/jobs/${jobId}/retry`, { dry_run: dryRun });
  }

  cancelJob(jobId) {
    return this.post(`/api/jobs/${jobId}/cancel`);
  }

  // Loop Records
  listLoopRecords() {
    return this.get(`/api/loop/records?robot_id=${encodeURIComponent(this.currentRobotId)}`);
  }

  saveLoopRecord(data) {
    return this.post("/api/loop/records", data);
  }

  deleteLoopRecord(recordId) {
    return this.delete(`/api/loop/records/${recordId}`);
  }
}

export const api = new RobotApi();
