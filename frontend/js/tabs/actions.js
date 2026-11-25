// Tab 3: Atomic Actions, Workflow Sequences & Loop Execution Jobs
import { api } from "../api.js";
import { drawer } from "../drawer.js";
import { escapeHtml, formatLocalTime, filterJobs } from "../ui-format.js";
import { notify, enableDialogLightDismiss } from "../notify.js";

export function initActionsTab(container) {
  let jobsRequest = 0;
  let lastJobsSignature = "";
  let activeSub = "jobs";
  let lastRobotId = api.currentRobotId;
  let cachedActions = [];
  let cachedWorkflows = [];
  let editingActionId = null;
  let editingWorkflowId = null;
  let wfDraftSteps = [];

  container.innerHTML = `
    <section class="execution-heading-card">
      <div class="execution-title-row"><div><h1>Loop Actions · 动作与工作流</h1></div><span class="workspace-readonly">MOCK ONLY · 默认只读</span></div>
    <!-- Sub navigation for Actions Tab -->
    <div class="execution-subtabs">
      <div style="display:flex; gap:8px;">
        <button class="btn btn-sm sub-tab-btn" data-sub="jobs" aria-selected="true" style="background:var(--accent-light); color:var(--accent);">执行任务记录</button>
        <button class="btn btn-sm sub-tab-btn" data-sub="workflows">工作流编排</button>
        <button class="btn btn-sm sub-tab-btn" data-sub="actions">原子动作</button>
        <button class="btn btn-sm sub-tab-btn" data-sub="r1pro">R1 导入 / 导出</button>
      </div>
      <div style="display:flex; gap:8px;">
        <button class="btn btn-sm" id="headerNewWorkflowBtn">+ 新建工作流</button>
        <button class="btn btn-sm btn-primary" id="newActionOrWfBtn">+ 新建原子动作</button>
      </div>
    </div>

    </section>

    <!-- Section 1: Jobs Table (Motphys Loop Layout) -->
    <div id="subViewJobs" class="card jobs-card">
      <div class="card-header">
        <span class="card-title">Jobs <span class="job-count" id="jobCount">0</span></span>
        <div class="jobs-filters">
          <input type="search" id="jobSearch" class="job-search" aria-label="搜索任务" placeholder="搜索 Flow / Run / Operator…">
          <select class="select-clean" id="jobStateFilter" style="font-size:12px; padding:3px 8px;">
            <option value="ALL">全部状态 (All States)</option>
            <option value="Running">Running</option>
            <option value="Paused">Paused</option>
            <option value="Succeeded">Succeeded</option>
            <option value="Failed">Failed</option>
            <option value="Canceled">Canceled</option>
          </select>
          <button class="btn btn-sm" id="refreshJobsBtn">↻ 刷新</button>
          <button class="btn btn-sm btn-primary" id="previewDryRunBtn">+ Mock dry-run</button>
        </div>
      </div>
      <div class="table-wrapper">
        <table class="data-table">
          <thead>
            <tr>
              <th style="width:110px;">State</th>
              <th style="width:110px;">Kind</th>
              <th>Flow</th>
              <th>Run</th>
              <th>Pipeline</th>
              <th>Created</th>
              <th>Operator</th>
              <th style="text-align:right;">Action</th>
            </tr>
          </thead>
          <tbody id="jobsListBody">
            <tr><td colspan="8" style="text-align:center; color:#94a3b8;">加载中...</td></tr>
          </tbody>
        </table>
      </div>
    </div>

    <!-- Section 2: Workflows List -->
    <div id="subViewWorkflows" class="card" style="display:none;">
      <div class="card-header">
        <span class="card-title">工作流编排库 (Workflow Sequences)</span>
        <button class="btn btn-sm btn-primary" id="createWorkflowBtn">+ 新建工作流</button>
      </div>
      <div id="workflowsListContainer" style="display:grid; grid-template-columns:repeat(auto-fill, minmax(360px, 1fr)); gap:14px;">
        <!-- Workflows rendered dynamically -->
      </div>
    </div>

    <!-- Section 3: Atomic Actions -->
    <div id="subViewActions" class="card" style="display:none;">
      <div class="card-header">
        <span class="card-title">原子动作库 (Atomic Actions)</span>
        <button class="btn btn-sm btn-primary" id="createActionBtn">+ 新建动作</button>
      </div>
      <div class="table-wrapper">
        <table class="data-table">
          <thead>
            <tr>
              <th>Name</th>
              <th>Kind</th>
              <th>Description</th>
              <th>Params Summary</th>
              <th>Tags</th>
              <th style="text-align:right;">Actions</th>
            </tr>
          </thead>
          <tbody id="actionsListBody">
            <!-- Rendered dynamically -->
          </tbody>
        </table>
      </div>
    </div>

    <!-- Section 4: R1 Pro Import / Export Compatibility -->
    <div id="subViewR1Pro" class="card" style="display:none;">
      <div class="card-header">
        <span class="card-title">Galaxea R1 Pro 格式双向兼容 (actions.json &amp; sequences.json)</span>
      </div>
      <div class="grid-2">
        <div>
          <h4 style="font-size:13px; font-weight:700; margin-bottom:8px;">导入 R1 Pro actions.json 或 sequences.json</h4>
          <p style="font-size:12px; color:var(--text-muted); margin-bottom:8px;">
            自动将角度 (deg) 转换为弧度 (rad)、归一化夹爪并解析关节名（如 <code>torso[1] -&gt; torso_1</code>）。
          </p>
          <textarea class="form-textarea" id="r1ImportTextarea" rows="8" placeholder='粘贴 R1 Pro 的 actions.json 或 sequences.json 内容...' style="font-family:var(--font-mono); font-size:11px;"></textarea>
          <div style="margin-top:8px; display:flex; gap:8px;">
            <button class="btn btn-sm btn-primary" id="importR1ActionsBtn">导入为原子动作</button>
            <button class="btn btn-sm" id="importR1SeqBtn">导入为工作流序列</button>
          </div>
        </div>
        <div>
          <h4 style="font-size:13px; font-weight:700; margin-bottom:8px;">导出兼容 Bundle</h4>
          <p style="font-size:12px; color:var(--text-muted); margin-bottom:8px;">
            将当前机器人库导出为标准 R1 Pro 规范的 JSON 结构。
          </p>
          <div style="display:flex; gap:8px;">
            <button class="btn btn-sm btn-primary" id="exportR1BundleBtn">生成导出 JSON</button>
            <button class="btn btn-sm" id="copyR1ExportBtn">复制 JSON</button>
          </div>
          <pre class="terminal-logs" id="exportR1OutputPre" style="height:180px; margin-top:8px; font-size:11px;">点击上方按钮生成导出数据...</pre>
        </div>
      </div>
    </div>

    <!-- Native Dialog: Edit/Create Action -->
    <dialog id="actionEditDialog" closedby="any" aria-labelledby="actionDialogTitle">
      <div class="dialog-content">
        <div class="dialog-header">
          <h1 class="dialog-title" id="actionDialogTitle">新建原子动作</h1>
          <button type="button" class="drawer-close-btn" id="closeActionDialogBtn" aria-label="关闭对话框">✕</button>
        </div>
        <div class="form-group">
          <label class="form-label" for="actFormName">动作名称 (Name)</label>
          <input type="text" class="form-input" id="actFormName" autofocus placeholder="例如: r1_safe_home">
        </div>
        <div class="grid-2" style="gap:10px;">
          <div class="form-group">
            <label class="form-label" for="actFormKind">动作类型 (Kind)</label>
            <select class="form-select" id="actFormKind">
              <option value="joint_targets">joint_targets (目标关节位姿)</option>
              <option value="ee_relative">ee_relative (末端相对位移)</option>
              <option value="ee_pose">ee_pose (末端绝对位姿)</option>
              <option value="ee_axial">ee_axial (末端绕轴旋转)</option>
              <option value="chassis_velocity">chassis_velocity (底盘速度)</option>
              <option value="navigate_to_pose">navigate_to_pose (导航目标)</option>
              <option value="perception_step">perception_step (感知/微调步骤)</option>
              <option value="policy_intent">policy_intent (策略意图)</option>
            </select>
          </div>
          <div class="form-group">
            <label class="form-label" for="actFormTags">标签 (逗号分隔)</label>
            <input type="text" class="form-input" id="actFormTags" placeholder="safe, home">
          </div>
        </div>
        <div class="form-group">
          <label class="form-label" for="actFormDesc">说明描述 (Description)</label>
          <input type="text" class="form-input" id="actFormDesc" placeholder="简要说明">
        </div>
        <div class="form-group">
          <label class="form-label" for="actFormParams">参数定义 (JSON Params)</label>
          <textarea class="form-textarea" id="actFormParams" rows="5" style="font-family:var(--font-mono); font-size:12px;"></textarea>
        </div>
        <div class="dialog-actions">
          <form method="dialog" style="display:inline;"><button type="submit" value="cancel" class="btn btn-sm" id="cancelActionDialogBtn">取消</button></form>
          <button type="button" class="btn btn-sm btn-primary" id="saveActionDialogBtn">保存动作</button>
        </div>
      </div>
    </dialog>

    <!-- Native Dialog: Visual Workflow Composer -->
    <dialog id="workflowEditDialog" closedby="any" aria-labelledby="wfDialogTitle" style="max-width:660px;">
      <div class="dialog-content">
        <div class="dialog-header">
          <h1 class="dialog-title" id="wfDialogTitle">新建可视化工作流</h1>
          <button type="button" class="drawer-close-btn" id="closeWfDialogBtn" aria-label="关闭对话框">✕</button>
        </div>
        <div class="grid-2" style="gap:10px;">
          <div class="form-group">
            <label class="form-label" for="wfFormName">工作流名称 (Flow Name)</label>
            <input type="text" class="form-input" id="wfFormName" autofocus placeholder="例如: r1_pick_and_place_flow">
          </div>
          <div class="form-group">
            <label class="form-label" for="wfFormKind">任务类型 (Kind)</label>
            <select class="form-select" id="wfFormKind">
              <option value="Workflow">Workflow (编排流程)</option>
              <option value="Eval">Eval (策略评测)</option>
              <option value="Calibration">Calibration (标定流程)</option>
              <option value="Manual">Manual (手动序列)</option>
            </select>
          </div>
        </div>
        <div class="form-group">
          <label class="form-label" for="wfFormDesc">说明描述 (Description)</label>
          <input type="text" class="form-input" id="wfFormDesc" placeholder="描述本工作流的步骤目标与安全前提">
        </div>

        <div class="form-group">
          <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:6px; gap:8px; flex-wrap:wrap;">
            <label class="form-label" style="margin:0;">步骤编排序列与参数覆盖 (Workflow Steps &amp; Overrides)</label>
            <div style="display:flex; gap:6px;">
              <button type="button" class="btn btn-sm" id="wfValidateBtn">✓ 校验参数依赖</button>
              <button type="button" class="btn btn-sm" id="wfAddStepBtn">+ 添加步骤</button>
            </div>
          </div>
          <div id="wfValidationBanner" style="display:none; font-size:11px; padding:6px 10px; border-radius:4px; margin-bottom:6px;"></div>
          <div id="wfStepsEditorList" style="display:flex; flex-direction:column; gap:8px; max-height:260px; overflow-y:auto; padding:8px; background:var(--bg-subtle); border:1px solid var(--border); border-radius:var(--radius-md);"></div>
        </div>

        <div class="dialog-actions">
          <form method="dialog" style="display:inline;"><button type="submit" value="cancel" class="btn btn-sm">取消</button></form>
          <button type="button" class="btn btn-sm btn-primary" id="saveWfDialogBtn">保存工作流</button>
        </div>
      </div>
    </dialog>
  `;

  // Subtab switching
  const subBtns = container.querySelectorAll(".sub-tab-btn");
  const subViews = {
    jobs: container.querySelector("#subViewJobs"),
    workflows: container.querySelector("#subViewWorkflows"),
    actions: container.querySelector("#subViewActions"),
    r1pro: container.querySelector("#subViewR1Pro"),
  };

  function showSubView(sub) {
    if (!subViews[sub]) return;
    activeSub = sub;
    subBtns.forEach(button => {
      const selected = button.getAttribute("data-sub") === sub;
      button.setAttribute("aria-selected", String(selected));
      button.style.background = selected ? "var(--accent-light)" : "";
      button.style.color = selected ? "var(--accent)" : "";
    });
    Object.entries(subViews).forEach(([key, element]) => {
      element.style.display = key === sub ? "block" : "none";
    });
    if (sub === "jobs") _loadJobs();
    if (sub === "workflows") _loadWorkflows();
    if (sub === "actions") _loadActions();
  }
  subBtns.forEach(button => button.addEventListener("click", () => showSubView(button.getAttribute("data-sub"))));

  // ---- Atomic Action Dialog ----
  const actionDialog = container.querySelector("#actionEditDialog");
  enableDialogLightDismiss(actionDialog);
  container.querySelector("#newActionOrWfBtn").addEventListener("click", () => _openActionDialog());
  container.querySelector("#createActionBtn").addEventListener("click", () => _openActionDialog());
  container.querySelector("#closeActionDialogBtn").addEventListener("click", () => actionDialog.close("cancel"));

  container.querySelector("#saveActionDialogBtn").addEventListener("click", async () => {
    const name = container.querySelector("#actFormName").value.trim();
    const kind = container.querySelector("#actFormKind").value;
    const desc = container.querySelector("#actFormDesc").value.trim();
    const tags = container.querySelector("#actFormTags").value
      .split(",")
      .map(t => t.trim())
      .filter(Boolean);
    let params = {};
    try {
      params = JSON.parse(container.querySelector("#actFormParams").value || "{}");
    } catch (e) {
      notify("参数 JSON 格式非法: " + e.message, "error");
      return;
    }
    if (!name) {
      notify("请输入动作名称", "error");
      return;
    }
    try {
      const payload = {
        name,
        kind,
        description: desc,
        tags,
        robot_id: api.currentRobotId,
        params,
      };
      if (editingActionId) payload.id = editingActionId;
      await api.saveAction(payload);
      actionDialog.close("save");
      notify(editingActionId ? `已更新原子动作 '${name}'` : `已创建原子动作 '${name}'`, "success");
      await _loadActions();
    } catch (err) {
      notify("保存动作失败: " + err.message, "error");
    }
  });

  function _openActionDialog(action = null) {
    editingActionId = action ? action.id : null;
    container.querySelector("#actionDialogTitle").textContent = action ? "编辑原子动作" : "新建原子动作";
    container.querySelector("#actFormName").value = action ? action.name : "";
    container.querySelector("#actFormKind").value = action ? action.kind : "joint_targets";
    container.querySelector("#actFormTags").value = action ? (action.tags || []).join(", ") : "";
    container.querySelector("#actFormDesc").value = action ? (action.description || "") : "";
    container.querySelector("#actFormParams").value = action ? JSON.stringify(action.params, null, 2) : '{\n  "duration_s": 1.2\n}';
    actionDialog.showModal();
  }

  // ---- Visual Workflow Composer Dialog ----
  const wfDialog = container.querySelector("#workflowEditDialog");
  enableDialogLightDismiss(wfDialog);
  container.querySelector("#createWorkflowBtn").addEventListener("click", () => _openWorkflowDialog());
  container.querySelector("#headerNewWorkflowBtn").addEventListener("click", () => _openWorkflowDialog());
  container.querySelector("#closeWfDialogBtn").addEventListener("click", () => wfDialog.close("cancel"));

  function _renderWfStepRows() {
    const listEl = container.querySelector("#wfStepsEditorList");
    if (!wfDraftSteps.length) {
      listEl.innerHTML = `<div style="text-align:center; color:var(--text-muted); font-size:12px; padding:14px;">尚未添加步骤。点击右上「+ 添加步骤」编排动作序列。</div>`;
      return;
    }
    const actionNames = cachedActions.map(a => a.name);
    const actMap = new Map(cachedActions.map(a => [a.name, a]));
    listEl.innerHTML = wfDraftSteps.map((st, idx) => {
      const options = actionNames.includes(st.action_name) ? actionNames : [st.action_name, ...actionNames].filter(Boolean);
      const actMeta = actMap.get(st.action_name);
      const prodTags = (actMeta?.produces || []).map(k => `<span class="param-badge-produces">+${escapeHtml(k)}</span>`).join("");
      const reqTags = (actMeta?.requires || []).map(k => `<span class="param-badge-requires">需:${escapeHtml(k)}</span>`).join("");
      const ovText = st.overrides && Object.keys(st.overrides).length ? JSON.stringify(st.overrides) : "";
      return `
        <div class="wf-step-row" data-idx="${idx}" style="background:var(--bg-surface); padding:6px 8px; border:1px solid var(--border); border-radius:4px;">
          <div style="display:grid; grid-template-columns:28px 1fr 64px 64px 66px auto; gap:6px; align-items:center;">
            <span style="font-family:var(--font-mono); font-size:11px; color:var(--text-muted); text-align:center;">#${idx + 1}</span>
            <select class="form-select wf-step-action" style="padding:4px 6px; font-size:12px;">
              ${options.length
                ? options.map(n => `<option value="${escapeHtml(n)}" ${n === st.action_name ? "selected" : ""}>${escapeHtml(n)}</option>`).join("")
                : `<option value="">（当前机型无原子动作）</option>`}
            </select>
            <input type="number" class="form-input wf-step-delay" value="${Number(st.delay_s || 0)}" min="0" max="30" step="0.1" title="前置延时 (s)" placeholder="延时s" style="padding:4px 6px; font-size:11px; font-family:var(--font-mono);">
            <input type="number" class="form-input wf-step-dur" value="${Number(st.duration_s || 1.2)}" min="0.1" max="30" step="0.1" title="执行时长 (s)" placeholder="时长s" style="padding:4px 6px; font-size:11px; font-family:var(--font-mono);">
            <input type="number" class="form-input wf-step-timeout" value="${Number(st.timeout_s || 15)}" min="0.5" max="120" step="1" title="闭环到位超时熔断 (s)" placeholder="超时s" style="padding:4px 6px; font-size:11px; font-family:var(--font-mono);">
            <div style="display:flex; gap:4px;">
              <button type="button" class="btn btn-sm wf-step-up" ${idx === 0 ? "disabled" : ""} title="上移" style="padding:2px 6px;">↑</button>
              <button type="button" class="btn btn-sm wf-step-down" ${idx === wfDraftSteps.length - 1 ? "disabled" : ""} title="下移" style="padding:2px 6px;">↓</button>
              <button type="button" class="btn btn-sm wf-step-del" title="移除" style="padding:2px 6px; color:var(--danger);">✕</button>
            </div>
          </div>
          <div style="display:flex; gap:6px; align-items:center; margin-top:5px; padding-left:34px; flex-wrap:wrap;">
            ${prodTags}${reqTags}
            <label style="display:inline-flex; align-items:center; gap:4px; font-size:11px; color:#b45309; cursor:pointer; white-space:nowrap;" title="该步闭环到位后自动暂停，等待人工确认后继续">
              <input type="checkbox" class="wf-step-pause" ${st.pause_after ? "checked" : ""}> 步后断点
            </label>
            <input type="text" class="form-input wf-step-overrides" value="${escapeHtml(ovText)}" placeholder='可选 overrides JSON，如 {"xyz_source":"workflow_pose","offset_xyz_m":[0,0,0.1]}' style="flex:1; min-width:200px; padding:3px 6px; font-size:11px; font-family:var(--font-mono);">
          </div>
        </div>
      `;
    }).join("");

    listEl.querySelectorAll(".wf-step-row").forEach(row => {
      const idx = Number(row.dataset.idx);
      row.querySelector(".wf-step-action").addEventListener("change", e => {
        wfDraftSteps[idx].action_name = e.target.value;
        _renderWfStepRows();
      });
      row.querySelector(".wf-step-delay").addEventListener("change", e => { wfDraftSteps[idx].delay_s = Math.max(0, parseFloat(e.target.value) || 0); });
      row.querySelector(".wf-step-dur").addEventListener("change", e => { wfDraftSteps[idx].duration_s = Math.max(0.1, parseFloat(e.target.value) || 1.2); });
      row.querySelector(".wf-step-timeout").addEventListener("change", e => { wfDraftSteps[idx].timeout_s = Math.max(0.5, Math.min(120, parseFloat(e.target.value) || 15)); });
      row.querySelector(".wf-step-pause").addEventListener("change", e => { wfDraftSteps[idx].pause_after = Boolean(e.target.checked); });
      row.querySelector(".wf-step-overrides").addEventListener("change", e => {
        const raw = e.target.value.trim();
        if (!raw) {
          wfDraftSteps[idx].overrides = {};
          e.target.style.borderColor = "";
          return;
        }
        try {
          const parsed = JSON.parse(raw);
          if (typeof parsed !== "object" || Array.isArray(parsed) || !parsed) throw new Error("需为 JSON 对象");
          wfDraftSteps[idx].overrides = parsed;
          e.target.style.borderColor = "";
        } catch (err) {
          e.target.style.borderColor = "var(--danger)";
          notify(`步骤 #${idx + 1} overrides JSON 格式错误: ${err.message}`, "error");
        }
      });
      row.querySelector(".wf-step-up").addEventListener("click", () => {
        if (idx > 0) { [wfDraftSteps[idx - 1], wfDraftSteps[idx]] = [wfDraftSteps[idx], wfDraftSteps[idx - 1]]; _renderWfStepRows(); }
      });
      row.querySelector(".wf-step-down").addEventListener("click", () => {
        if (idx < wfDraftSteps.length - 1) { [wfDraftSteps[idx + 1], wfDraftSteps[idx]] = [wfDraftSteps[idx], wfDraftSteps[idx + 1]]; _renderWfStepRows(); }
      });
      row.querySelector(".wf-step-del").addEventListener("click", () => {
        wfDraftSteps.splice(idx, 1);
        _renderWfStepRows();
      });
    });
  }

  async function _openWorkflowDialog(wf = null) {
    try {
      const data = await api.listActions();
      cachedActions = data.actions || [];
    } catch {
      cachedActions = [];
    }
    editingWorkflowId = wf ? wf.id : null;
    container.querySelector("#wfDialogTitle").textContent = wf ? "编辑可视化工作流" : "新建可视化工作流";
    container.querySelector("#wfFormName").value = wf ? wf.name : "";
    container.querySelector("#wfFormKind").value = wf ? (wf.kind || "Workflow") : "Workflow";
    container.querySelector("#wfFormDesc").value = wf ? (wf.description || "") : "";
    const vb = container.querySelector("#wfValidationBanner");
    if (vb) vb.style.display = "none";
    wfDraftSteps = wf
      ? (wf.steps || []).map((s, i) => ({
          id: s.id || `st-${i + 1}`,
          action_name: s.action_name,
          delay_s: s.delay_s || 0,
          duration_s: s.duration_s || 1.2,
          timeout_s: s.timeout_s || 15.0,
          pause_after: Boolean(s.pause_after),
          overrides: { ...(s.overrides || {}) },
        }))
      : cachedActions.length ? [{ id: "st-1", action_name: cachedActions[0].name, delay_s: 0.0, duration_s: 1.2, timeout_s: 15.0, pause_after: false, overrides: {} }] : [];
    _renderWfStepRows();
    wfDialog.showModal();
  }

  container.querySelector("#wfValidateBtn").addEventListener("click", async () => {
    const vb = container.querySelector("#wfValidationBanner");
    try {
      const res = await api.validateWorkflow({
        robot_id: api.currentRobotId,
        name: container.querySelector("#wfFormName").value.trim() || "draft",
        steps: wfDraftSteps,
      });
      vb.style.display = "block";
      vb.style.background = res.ok ? "var(--success-bg)" : "var(--danger-bg)";
      vb.style.color = res.ok ? "var(--success)" : "var(--danger)";
      vb.textContent = (res.ok ? "✓ " : "✕ ") + res.message;
    } catch (err) {
      vb.style.display = "block";
      vb.style.background = "var(--danger-bg)";
      vb.style.color = "var(--danger)";
      vb.textContent = "✕ 校验失败: " + err.message;
    }
  });

  container.querySelector("#wfAddStepBtn").addEventListener("click", () => {
    const defaultAction = cachedActions[0]?.name || "";
    if (!defaultAction) {
      notify("当前机器人尚无原子动作，请先创建至少一个原子动作", "error");
      return;
    }
    wfDraftSteps.push({
      id: `st-${Date.now().toString(36)}`,
      action_name: defaultAction,
      delay_s: 0.0,
      duration_s: 1.2,
      timeout_s: 15.0,
      pause_after: false,
      overrides: {},
    });
    _renderWfStepRows();
  });

  container.querySelector("#saveWfDialogBtn").addEventListener("click", async () => {
    const name = container.querySelector("#wfFormName").value.trim();
    const kind = container.querySelector("#wfFormKind").value;
    const description = container.querySelector("#wfFormDesc").value.trim();
    if (!name) {
      notify("请输入工作流名称", "error");
      return;
    }
    if (!wfDraftSteps.length || wfDraftSteps.some(s => !s.action_name)) {
      notify("请至少编排 1 个有效的原子动作步骤", "error");
      return;
    }
    try {
      const payload = {
        name,
        kind,
        description,
        robot_id: api.currentRobotId,
        steps: wfDraftSteps,
      };
      if (editingWorkflowId) payload.id = editingWorkflowId;
      const saved = await api.saveWorkflow(payload);
      wfDialog.close("save");
      const valNote = saved.validation?.ok === false ? `（警告: ${saved.validation.message}）` : "";
      notify((editingWorkflowId ? `已更新工作流 '${name}'` : `已保存新工作流 '${name}'`) + valNote, saved.validation?.ok === false ? "info" : "success");
      showSubView("workflows");
    } catch (err) {
      notify("保存工作流失败: " + err.message, "error");
    }
  });

  // Jobs Loading
  async function _loadJobs() {
    const request = ++jobsRequest;
    const robotId = api.currentRobotId;
    const tbody = container.querySelector("#jobsListBody");
    const filter = container.querySelector("#jobStateFilter").value;
    try {
      const data = await api.listJobs();
      if (request !== jobsRequest || robotId !== api.currentRobotId) return;
      const jobs = filterJobs(data.jobs || [], filter, container.querySelector("#jobSearch").value);
      container.querySelector("#jobCount").textContent = jobs.length;
      if (jobs.length === 0) {
        tbody.innerHTML = `<tr><td colspan="8" style="text-align:center; color:#94a3b8;">暂无匹配的执行任务。点击「Mock dry-run」生成本地验证记录（不发送运动指令）。</td></tr>`;
        return;
      }
      tbody.innerHTML = jobs.map(j => `
        <tr data-job-id="${escapeHtml(j.job_id)}">
          <td><span class="badge badge-${escapeHtml(j.state.toLowerCase())}">${escapeHtml(j.state)}</span></td>
          <td><span style="font-weight:600; color:var(--text-muted);">${escapeHtml(j.kind)}</span></td>
          <td style="font-weight:600;">${escapeHtml(j.flow_name)}</td>
          <td style="font-family:var(--font-mono); font-size:12px; color:var(--text-muted);">${escapeHtml(j.run_id)}</td>
          <td style="font-family:var(--font-mono); font-size:11px; color:var(--text-muted);">${escapeHtml(j.pipeline_sha ? j.pipeline_sha.slice(0, 10) : "--")}</td>
          <td style="font-size:12px; color:var(--text-muted);">${formatLocalTime(j.created_at)}</td>
          <td style="font-size:12px;">${escapeHtml(j.operator || "--")}</td>
          <td style="text-align:right;">
            <button class="btn btn-sm open-drawer-btn" data-job="${escapeHtml(j.job_id)}">查看详情</button>
          </td>
        </tr>
      `).join("");

      tbody.querySelectorAll(".open-drawer-btn").forEach(btn => {
        btn.addEventListener("click", (e) => {
          e.stopPropagation();
          const jid = btn.getAttribute("data-job");
          drawer.open(jid);
        });
      });
      tbody.querySelectorAll("tr").forEach(tr => {
        tr.addEventListener("click", () => {
          const jid = tr.getAttribute("data-job-id");
          if (jid) drawer.open(jid);
        });
      });
    } catch (err) {
      tbody.innerHTML = `<tr><td colspan="8" style="color:var(--danger);">加载失败: ${escapeHtml(err.message)}</td></tr>`;
    }
  }

  container.querySelector("#refreshJobsBtn").addEventListener("click", _loadJobs);
  container.querySelector("#jobStateFilter").addEventListener("change", _loadJobs);
  container.querySelector("#jobSearch").addEventListener("input", _loadJobs);
  container.querySelector("#previewDryRunBtn").addEventListener("click", async (event) => {
    const button = event.currentTarget;
    button.disabled = true;
    try {
      const { workflows = [] } = await api.listWorkflows();
      if (!workflows.length) throw new Error("当前机器人尚无工作流，请先创建工作流");
      const { job } = await api.runWorkflow(workflows[0].id, true);
      await _loadJobs();
      await drawer.open(job.job_id);
    } catch (error) {
      notify("Mock dry-run 失败: " + error.message, "error");
    } finally {
      button.disabled = false;
    }
  });

  // Workflows Loading
  async function _loadWorkflows() {
    const wrap = container.querySelector("#workflowsListContainer");
    try {
      const data = await api.listWorkflows();
      const list = data.workflows || [];
      cachedWorkflows = list;
      if (list.length === 0) {
        wrap.innerHTML = `<div style="grid-column:1/-1; text-align:center; color:#94a3b8; padding:30px;">暂无编排工作流。点击右上「+ 新建工作流」开始可视化编排。</div>`;
        return;
      }
      wrap.innerHTML = list.map(wf => {
        const val = wf.validation || { ok: true, step_contracts: [] };
        const contracts = val.step_contracts || [];
        return `
        <div class="card" style="margin-bottom:0; display:flex; flex-direction:column; justify-content:space-between;">
          <div>
            <div style="display:flex; justify-content:space-between; align-items:flex-start; margin-bottom:8px; gap:8px;">
              <span style="font-weight:700; font-size:14px; color:var(--text-main);">${escapeHtml(wf.name)}</span>
              <div style="display:flex; gap:4px; flex-wrap:wrap; justify-content:flex-end;">
                <span class="badge ${val.ok ? "badge-passed" : "badge-failed"}" title="${escapeHtml(val.message || "")}">${val.ok ? "依赖✓" : "缺依赖"}</span>
                <span class="badge badge-pending">${escapeHtml(wf.kind)}</span>
              </div>
            </div>
            <p style="font-size:12px; color:var(--text-muted); margin-bottom:10px;">${escapeHtml(wf.description || "无描述")}</p>
            <div style="background:var(--bg-subtle); border-radius:var(--radius-sm); padding:8px 10px; margin-bottom:12px;">
              <div style="font-size:11px; font-weight:700; color:var(--text-muted); margin-bottom:4px;">步骤与参数传递链 (${wf.steps.length} 步)</div>
              ${wf.steps.map((st, i) => {
                const c = contracts[i] || {};
                const prods = (c.produces || []).map(k => `<span class="param-badge-produces">+${escapeHtml(k)}</span>`).join("");
                const reqs = (c.requires || []).map(k => `<span class="param-badge-requires">←${escapeHtml(k)}</span>`).join("");
                const ovs = st.overrides && Object.keys(st.overrides).length
                  ? `<span class="param-badge-override" title="${escapeHtml(JSON.stringify(st.overrides))}">覆盖:${escapeHtml(Object.keys(st.overrides).join(","))}</span>`
                  : "";
                const bpTag = st.pause_after ? `<span class="badge badge-paused" style="font-size:10px; padding:1px 5px;">断点</span>` : "";
                return `
                <div style="font-size:12px; padding:3px 0; border-bottom:1px dashed rgba(148,163,184,0.2);">
                  <div style="display:flex; justify-content:space-between; gap:6px;">
                    <span>${i + 1}. ${escapeHtml(st.action_name)} ${bpTag}</span>
                    <span style="font-family:var(--font-mono); color:var(--text-subtle); white-space:nowrap;">${st.delay_s ? `+${st.delay_s}s / ` : ""}${st.duration_s}s</span>
                  </div>
                  ${(prods || reqs || ovs) ? `<div style="display:flex; gap:4px; flex-wrap:wrap; margin-top:2px;">${reqs}${prods}${ovs}</div>` : ""}
                </div>`;
              }).join("")}
            </div>
          </div>
          <div style="display:flex; gap:6px; justify-content:space-between; align-items:center; border-top:1px solid var(--border); padding-top:10px; flex-wrap:wrap;">
            <div style="display:flex; gap:6px;">
              <button class="btn btn-sm edit-wf-btn" data-wfid="${escapeHtml(wf.id)}">编辑</button>
              <button class="btn btn-sm del-wf-btn" data-wfid="${escapeHtml(wf.id)}" style="color:var(--danger);">删除</button>
            </div>
            <div style="display:flex; gap:6px; flex-wrap:wrap;">
              <button class="btn btn-sm preview-wf-btn" data-wfid="${escapeHtml(wf.id)}" style="border-color:#8b5cf6; color:#6d28d9; background:#f5f3ff;" title="在 3D 视口以紫色幽灵+霓虹轨迹预演整个工作流及参数传递（免 Lease）">3D 轨迹预演</button>
              <button class="btn btn-sm run-dry-btn" data-wfid="${escapeHtml(wf.id)}">试运行 (Dry-run)</button>
              <button class="btn btn-sm btn-step-debug run-step-btn" data-wfid="${escapeHtml(wf.id)}" title="单步调试：每步闭环到位后自动暂停，人工确认后再点「下一步」（需 Lease）">⏭ 单步调试</button>
              <button class="btn btn-sm btn-primary run-live-btn" data-wfid="${escapeHtml(wf.id)}">授权执行 (Live)</button>
            </div>
          </div>
        </div>`;
      }).join("");

      wrap.querySelectorAll(".preview-wf-btn").forEach(b => {
        b.addEventListener("click", async () => {
          const wfid = b.getAttribute("data-wfid");
          try {
            const res = await api.previewWorkflow(wfid);
            const jointsTabBtn = document.querySelector('.tab-btn[data-tab="joints"]');
            if (jointsTabBtn) jointsTabBtn.click();
            window.dispatchEvent(new CustomEvent("motphys:preview-plan", { detail: res }));
            notify(`已在 3D 视口加载工作流轨迹预演（${res.steps.length} 步 · 未下发执行）`, "success");
          } catch (err) {
            notify("工作流轨迹预演失败: " + err.message, "error");
          }
        });
      });

      wrap.querySelectorAll(".edit-wf-btn").forEach(b => {
        b.addEventListener("click", () => {
          const wf = cachedWorkflows.find(w => w.id === b.getAttribute("data-wfid"));
          if (wf) _openWorkflowDialog(wf);
        });
      });

      wrap.querySelectorAll(".del-wf-btn").forEach(b => {
        b.addEventListener("click", async () => {
          const wfid = b.getAttribute("data-wfid");
          if (confirm("确定删除该工作流序列？")) {
            try {
              await api.deleteWorkflow(wfid);
              notify("已删除工作流", "info");
              _loadWorkflows();
            } catch (err) {
              notify("删除工作流失败: " + err.message, "error");
            }
          }
        });
      });

      wrap.querySelectorAll(".run-dry-btn").forEach(b => {
        b.addEventListener("click", async () => {
          const wfid = b.getAttribute("data-wfid");
          try {
            const res = await api.runWorkflow(wfid, true);
            drawer.open(res.job.job_id);
          } catch (err) {
            notify("启动试运行失败: " + err.message, "error");
          }
        });
      });

      wrap.querySelectorAll(".run-step-btn").forEach(b => {
        b.addEventListener("click", async () => {
          const wfid = b.getAttribute("data-wfid");
          try {
            const res = await api.runWorkflow(wfid, false, { stepMode: true });
            drawer.open(res.job.job_id);
          } catch (err) {
            notify("启动单步调试失败（需持有 Lease）: " + err.message, "error");
          }
        });
      });

      wrap.querySelectorAll(".run-live-btn").forEach(b => {
        b.addEventListener("click", async () => {
          const wfid = b.getAttribute("data-wfid");
          try {
            const res = await api.runWorkflow(wfid, false);
            drawer.open(res.job.job_id);
          } catch (err) {
            notify("启动执行失败（需持有 Lease）: " + err.message, "error");
          }
        });
      });
    } catch (err) {
      wrap.innerHTML = `<div style="color:var(--danger);">加载失败: ${escapeHtml(err.message)}</div>`;
    }
  }

  // Actions Loading
  async function _loadActions() {
    const tbody = container.querySelector("#actionsListBody");
    try {
      const data = await api.listActions();
      const list = data.actions || [];
      cachedActions = list;
      if (list.length === 0) {
        tbody.innerHTML = `<tr><td colspan="6" style="text-align:center; color:#94a3b8;">暂无原子动作</td></tr>`;
        return;
      }
      tbody.innerHTML = list.map(a => `
        <tr>
          <td style="font-weight:700;">${escapeHtml(a.name)}</td>
          <td><span class="badge badge-pending">${escapeHtml(a.kind)}</span></td>
          <td style="font-size:12px; color:var(--text-muted);">${escapeHtml(a.description || "--")}</td>
          <td style="font-family:var(--font-mono); font-size:11px; color:var(--text-muted);">${escapeHtml(JSON.stringify(a.params).slice(0, 56))}</td>
          <td>${(a.tags || []).map(t => `<span class="badge badge-no_result" style="margin-right:2px;">${escapeHtml(t)}</span>`).join("")}</td>
          <td style="text-align:right; white-space:nowrap;">
            ${a.kind === "joint_targets" ? `<button class="btn btn-sm preview-act-btn" data-actid="${escapeHtml(a.id)}" title="在 3D 视图中本地预览姿态（不下发控制）">3D 预览</button>` : ""}
            <button class="btn btn-sm dryrun-act-btn" data-actid="${escapeHtml(a.id)}" title="安全试跑（免 Lease，不下发运动）">试跑</button>
            <button class="btn btn-sm btn-primary exec-act-btn" data-actid="${escapeHtml(a.id)}" title="单步授权执行（需持有 Lease，记录到执行抽屉）">执行</button>
            <button class="btn btn-sm edit-act-btn" data-actid="${escapeHtml(a.id)}">编辑</button>
            <button class="btn btn-sm delete-act-btn" data-actid="${escapeHtml(a.id)}" style="color:var(--danger);">删除</button>
          </td>
        </tr>
      `).join("");

      tbody.querySelectorAll(".preview-act-btn").forEach(b => {
        b.addEventListener("click", (e) => {
          e.stopPropagation();
          const act = cachedActions.find(a => a.id === b.getAttribute("data-actid"));
          if (!act) return;
          const jointsTabBtn = document.querySelector('.tab-btn[data-tab="joints"]');
          if (jointsTabBtn) jointsTabBtn.click();
          window.dispatchEvent(new CustomEvent("motphys:preview-action", { detail: { action: act } }));
        });
      });

      tbody.querySelectorAll(".dryrun-act-btn").forEach(b => {
        b.addEventListener("click", async (e) => {
          e.stopPropagation();
          const actid = b.getAttribute("data-actid");
          try {
            const res = await api.executeAction(actid, true);
            await _loadJobs();
            drawer.open(res.job.job_id);
          } catch (err) {
            notify("试跑动作失败: " + err.message, "error");
          }
        });
      });

      tbody.querySelectorAll(".exec-act-btn").forEach(b => {
        b.addEventListener("click", async (e) => {
          e.stopPropagation();
          const actid = b.getAttribute("data-actid");
          try {
            const res = await api.executeAction(actid, false);
            await _loadJobs();
            drawer.open(res.job.job_id);
          } catch (err) {
            notify("执行动作失败（需持有 Lease）: " + err.message, "error");
          }
        });
      });

      tbody.querySelectorAll(".edit-act-btn").forEach(b => {
        b.addEventListener("click", (e) => {
          e.stopPropagation();
          const act = cachedActions.find(a => a.id === b.getAttribute("data-actid"));
          if (act) _openActionDialog(act);
        });
      });

      tbody.querySelectorAll(".delete-act-btn").forEach(b => {
        b.addEventListener("click", async (e) => {
          e.stopPropagation();
          const actid = b.getAttribute("data-actid");
          if (confirm("确定删除该原子动作？")) {
            try {
              await api.deleteAction(actid);
              notify("已删除原子动作", "info");
              _loadActions();
            } catch (err) {
              notify("删除动作失败: " + err.message, "error");
            }
          }
        });
      });
    } catch (err) {
      tbody.innerHTML = `<tr><td colspan="6" style="color:var(--danger);">加载失败: ${escapeHtml(err.message)}</td></tr>`;
    }
  }

  // R1 Pro Import / Export handlers
  container.querySelector("#importR1ActionsBtn").addEventListener("click", async () => {
    const raw = container.querySelector("#r1ImportTextarea").value.trim();
    if (!raw) return notify("请先粘贴 JSON 内容", "info");
    try {
      const payload = JSON.parse(raw);
      const res = await api.importR1Actions(payload);
      notify(`成功导入 ${res.imported_count} 个原子动作`, "success");
      _loadActions();
    } catch (e) {
      notify("导入动作失败: " + e.message, "error");
    }
  });

  container.querySelector("#importR1SeqBtn").addEventListener("click", async () => {
    const raw = container.querySelector("#r1ImportTextarea").value.trim();
    if (!raw) return notify("请先粘贴 JSON 内容", "info");
    try {
      const payload = JSON.parse(raw);
      const res = await api.post(`/api/workflows/import-r1pro?robot_id=${encodeURIComponent(api.currentRobotId)}`, payload);
      notify(`成功导入 ${res.imported_count} 个工作流序列`, "success");
      _loadWorkflows();
    } catch (e) {
      notify("导入工作流失败: " + e.message, "error");
    }
  });

  container.querySelector("#exportR1BundleBtn").addEventListener("click", async () => {
    try {
      const bundle = await api.get(`/api/actions/export-r1pro?robot_id=${encodeURIComponent(api.currentRobotId)}`);
      container.querySelector("#exportR1OutputPre").textContent = JSON.stringify(bundle, null, 2);
      notify("已生成 R1 Pro 兼容导出 Bundle", "success");
    } catch (e) {
      notify("导出失败: " + e.message, "error");
    }
  });

  container.querySelector("#copyR1ExportBtn").addEventListener("click", async () => {
    const text = container.querySelector("#exportR1OutputPre").textContent;
    if (!text || text.startsWith("点击上方")) return notify("请先点击「生成导出 JSON」", "info");
    try {
      await navigator.clipboard.writeText(text);
      notify("已复制 R1 Pro 导出 JSON", "success");
    } catch (e) {
      notify("复制失败: " + e.message, "error");
    }
  });

  // Direct links preserve the independent catalog/workflow/jobs pages.
  const requestedSection = new URLSearchParams(window.location.search).get("section");
  showSubView(subViews[requestedSection] ? requestedSection : "jobs");

  return {
    update(state) {
      if (lastRobotId !== api.currentRobotId) {
        lastRobotId = api.currentRobotId;
        showSubView(activeSub);
      }
      const signature = JSON.stringify([state?.descriptor?.id, (state?.recent_jobs || []).map(j => [j.job_id, j.state])]);
      if (signature !== lastJobsSignature) {
        lastJobsSignature = signature;
        _loadJobs();
      }
    }
  };
}
