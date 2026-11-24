// Tab 3: Atomic Actions, Workflow Sequences & Loop Execution Jobs
import { api } from "../api.js";
import { drawer } from "../drawer.js";

export function initActionsTab(container) {
  container.innerHTML = `
    <!-- Sub navigation for Actions Tab -->
    <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:16px;">
      <div style="display:flex; gap:8px;">
        <button class="btn btn-sm sub-tab-btn" data-sub="jobs" style="background:var(--accent-light); color:var(--accent);">执行任务 (Loop Jobs)</button>
        <button class="btn btn-sm sub-tab-btn" data-sub="workflows">工作流编排 (Sequences)</button>
        <button class="btn btn-sm sub-tab-btn" data-sub="actions">原子动作库 (Actions)</button>
        <button class="btn btn-sm sub-tab-btn" data-sub="r1pro">R1 Pro 格式导入/导出</button>
      </div>
      <div>
        <button class="btn btn-sm btn-primary" id="newActionOrWfBtn">+ 新建动作/流程</button>
      </div>
    </div>

    <!-- Section 1: Jobs Table (Motphys Loop Layout) -->
    <div id="subViewJobs" class="card">
      <div class="card-header">
        <span class="card-title">Loop Actions 任务执行历史 (Jobs)</span>
        <div style="display:flex; gap:8px;">
          <select class="select-clean" id="jobStateFilter" style="font-size:12px; padding:3px 8px;">
            <option value="ALL">全部状态 (All States)</option>
            <option value="Running">Running</option>
            <option value="Succeeded">Succeeded</option>
            <option value="Failed">Failed</option>
            <option value="Canceled">Canceled</option>
          </select>
          <button class="btn btn-sm" id="refreshJobsBtn">刷新</button>
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
        <span class="card-title">Galaxea R1 Pro 格式双向兼容 (actions.json & sequences.json)</span>
      </div>
      <div class="grid-2">
        <div>
          <h4 style="font-size:13px; font-weight:700; margin-bottom:8px;">导入 R1 Pro actions.json 或 sequences.json</h4>
          <p style="font-size:12px; color:var(--text-muted); margin-bottom:8px;">
            自动将角度 (deg) 转换为弧度 (rad)、归一化夹爪并解析关节名（如 <code>torso[1] -> torso_1</code>）。
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
          <button class="btn btn-sm" id="exportR1BundleBtn">生成导出 JSON</button>
          <pre class="terminal-logs" id="exportR1OutputPre" style="height:180px; margin-top:8px; font-size:11px;">点击上方按钮生成导出数据...</pre>
        </div>
      </div>
    </div>

    <!-- Native Dialog: Edit/Create Action -->
    <dialog id="actionEditDialog">
      <div class="dialog-content">
        <div class="dialog-header">
          <span class="dialog-title" id="actionDialogTitle">编辑原子动作</span>
          <button class="drawer-close-btn" id="closeActionDialogBtn">✕</button>
        </div>
        <div class="form-group">
          <label class="form-label">动作名称 (Name)</label>
          <input type="text" class="form-input" id="actFormName" placeholder="例如: r1_safe_home">
        </div>
        <div class="form-group">
          <label class="form-label">动作类型 (Kind)</label>
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
          <label class="form-label">说明描述 (Description)</label>
          <input type="text" class="form-input" id="actFormDesc" placeholder="简要说明">
        </div>
        <div class="form-group">
          <label class="form-label">参数定义 (JSON Params)</label>
          <textarea class="form-textarea" id="actFormParams" rows="5" style="font-family:var(--font-mono); font-size:12px;"></textarea>
        </div>
        <div class="dialog-actions">
          <button class="btn btn-sm" id="cancelActionDialogBtn">取消</button>
          <button class="btn btn-sm btn-primary" id="saveActionDialogBtn">保存动作</button>
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

  subBtns.forEach(btn => {
    btn.addEventListener("click", () => {
      const sub = btn.getAttribute("data-sub");
      subBtns.forEach(b => {
        b.style.background = "";
        b.style.color = "";
      });
      btn.style.background = "var(--accent-light)";
      btn.style.color = "var(--accent)";

      Object.entries(subViews).forEach(([k, el]) => {
        el.style.display = k === sub ? "block" : "none";
      });

      if (sub === "jobs") _loadJobs();
      if (sub === "workflows") _loadWorkflows();
      if (sub === "actions") _loadActions();
    });
  });

  // Action Dialog
  const actionDialog = container.querySelector("#actionEditDialog");
  container.querySelector("#newActionOrWfBtn").addEventListener("click", () => _openActionDialog());
  container.querySelector("#createActionBtn").addEventListener("click", () => _openActionDialog());
  container.querySelector("#closeActionDialogBtn").addEventListener("click", () => actionDialog.close());
  container.querySelector("#cancelActionDialogBtn").addEventListener("click", () => actionDialog.close());

  container.querySelector("#saveActionDialogBtn").addEventListener("click", async () => {
    const name = container.querySelector("#actFormName").value.trim();
    const kind = container.querySelector("#actFormKind").value;
    const desc = container.querySelector("#actFormDesc").value.trim();
    let params = {};
    try {
      params = JSON.parse(container.querySelector("#actFormParams").value || "{}");
    } catch (e) {
      alert("参数 JSON 格式非法: " + e.message);
      return;
    }
    if (!name) {
      alert("请输入动作名称");
      return;
    }
    try {
      await api.saveAction({
        name,
        kind,
        description: desc,
        robot_id: api.currentRobotId,
        params,
      });
      actionDialog.close();
      _loadActions();
    } catch (err) {
      alert("保存失败: " + err.message);
    }
  });

  function _openActionDialog(action = null) {
    container.querySelector("#actFormName").value = action ? action.name : "";
    container.querySelector("#actFormKind").value = action ? action.kind : "joint_targets";
    container.querySelector("#actFormDesc").value = action ? action.description : "";
    container.querySelector("#actFormParams").value = action ? JSON.stringify(action.params, null, 2) : '{\n  "duration_s": 1.2\n}';
    actionDialog.showModal();
  }

  // Jobs Loading
  async function _loadJobs() {
    const tbody = container.querySelector("#jobsListBody");
    const filter = container.querySelector("#jobStateFilter").value;
    try {
      const data = await api.listJobs();
      let jobs = data.jobs || [];
      if (filter !== "ALL") {
        jobs = jobs.filter(j => j.state === filter);
      }
      if (jobs.length === 0) {
        tbody.innerHTML = `<tr><td colspan="8" style="text-align:center; color:#94a3b8;">暂无匹配的执行任务</td></tr>`;
        return;
      }
      tbody.innerHTML = jobs.map(j => `
        <tr data-job-id="${j.job_id}">
          <td><span class="badge badge-${j.state.toLowerCase()}">${j.state}</span></td>
          <td><span style="font-weight:600; color:var(--text-muted);">${j.kind}</span></td>
          <td style="font-weight:600;">${j.flow_name}</td>
          <td style="font-family:var(--font-mono); font-size:12px; color:var(--text-muted);">${j.run_id}</td>
          <td style="font-family:var(--font-mono); font-size:11px; color:var(--text-muted);">${j.pipeline_sha ? j.pipeline_sha.slice(0, 10) : "--"}</td>
          <td style="font-size:12px; color:var(--text-muted);">${j.created_at ? j.created_at.slice(11, 19) : "--"}</td>
          <td style="font-size:12px;">${j.operator || "--"}</td>
          <td style="text-align:right;">
            <button class="btn btn-sm open-drawer-btn" data-job="${j.job_id}">查看详情 ➔</button>
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
      tbody.innerHTML = `<tr><td colspan="8" style="color:var(--danger);">加载失败: ${err.message}</td></tr>`;
    }
  }

  container.querySelector("#refreshJobsBtn").addEventListener("click", _loadJobs);
  container.querySelector("#jobStateFilter").addEventListener("change", _loadJobs);

  // Workflows Loading
  async function _loadWorkflows() {
    const wrap = container.querySelector("#workflowsListContainer");
    try {
      const data = await api.listWorkflows();
      const list = data.workflows || [];
      if (list.length === 0) {
        wrap.innerHTML = `<div style="grid-column:1/-1; text-align:center; color:#94a3b8; padding:30px;">暂无编排工作流</div>`;
        return;
      }
      wrap.innerHTML = list.map(wf => `
        <div class="card" style="margin-bottom:0; display:flex; flex-direction:column; justify-content:space-between;">
          <div>
            <div style="display:flex; justify-content:space-between; align-items:flex-start; margin-bottom:8px;">
              <span style="font-weight:700; font-size:14px; color:var(--text-main);">${wf.name}</span>
              <span class="badge badge-pending">${wf.kind}</span>
            </div>
            <p style="font-size:12px; color:var(--text-muted); margin-bottom:12px;">${wf.description || "无描述"}</p>
            <div style="background:var(--bg-subtle); border-radius:var(--radius-sm); padding:8px 10px; margin-bottom:12px;">
              <div style="font-size:11px; font-weight:700; color:var(--text-muted); margin-bottom:4px;">步骤序列 (${wf.steps.length} 步)</div>
              ${wf.steps.map((st, i) => `
                <div style="font-size:12px; display:flex; justify-content:space-between; padding:2px 0;">
                  <span>${i + 1}. ${st.action_name}</span>
                  <span style="font-family:var(--font-mono); color:var(--text-subtle);">${st.duration_s}s</span>
                </div>
              `).join("")}
            </div>
          </div>
          <div style="display:flex; gap:8px; justify-content:flex-end; border-top:1px solid var(--border); padding-top:10px;">
            <button class="btn btn-sm run-dry-btn" data-wfid="${wf.id}">试运行 (Dry-run)</button>
            <button class="btn btn-sm btn-primary run-live-btn" data-wfid="${wf.id}">授权执行 (Live)</button>
          </div>
        </div>
      `).join("");

      wrap.querySelectorAll(".run-dry-btn").forEach(b => {
        b.addEventListener("click", async () => {
          const wfid = b.getAttribute("data-wfid");
          try {
            const res = await api.runWorkflow(wfid, true);
            drawer.open(res.job.job_id);
          } catch (err) {
            alert("启动试运行失败: " + err.message);
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
            alert("启动执行失败（需持有 Lease）: " + err.message);
          }
        });
      });
    } catch (err) {
      wrap.innerHTML = `<div style="color:var(--danger);">加载失败: ${err.message}</div>`;
    }
  }

  // Actions Loading
  async function _loadActions() {
    const tbody = container.querySelector("#actionsListBody");
    try {
      const data = await api.listActions();
      const list = data.actions || [];
      if (list.length === 0) {
        tbody.innerHTML = `<tr><td colspan="6" style="text-align:center; color:#94a3b8;">暂无原子动作</td></tr>`;
        return;
      }
      tbody.innerHTML = list.map(a => `
        <tr>
          <td style="font-weight:700;">${a.name}</td>
          <td><span class="badge badge-pending">${a.kind}</span></td>
          <td style="font-size:12px; color:var(--text-muted);">${a.description || "--"}</td>
          <td style="font-family:var(--font-mono); font-size:11px; color:var(--text-muted);">${JSON.stringify(a.params).slice(0, 48)}...</td>
          <td>${(a.tags || []).map(t => `<span class="badge badge-no_result" style="margin-right:2px;">${t}</span>`).join("")}</td>
          <td style="text-align:right;">
            <button class="btn btn-sm delete-act-btn" data-actid="${a.id}" style="color:var(--danger);">删除</button>
          </td>
        </tr>
      `).join("");

      tbody.querySelectorAll(".delete-act-btn").forEach(b => {
        b.addEventListener("click", async (e) => {
          e.stopPropagation();
          const actid = b.getAttribute("data-actid");
          if (confirm("确定删除该动作？")) {
            await api.deleteAction(actid);
            _loadActions();
          }
        });
      });
    } catch (err) {
      tbody.innerHTML = `<tr><td colspan="6" style="color:var(--danger);">加载失败: ${err.message}</td></tr>`;
    }
  }

  // R1 Pro Import / Export handlers
  container.querySelector("#importR1ActionsBtn").addEventListener("click", async () => {
    const raw = container.querySelector("#r1ImportTextarea").value.trim();
    if (!raw) return alert("请先粘贴 JSON 内容");
    try {
      const payload = JSON.parse(raw);
      const res = await api.importR1Actions(payload);
      alert(`成功导入 ${res.imported_count} 个动作！`);
      _loadActions();
    } catch (e) {
      alert("导入失败: " + e.message);
    }
  });

  container.querySelector("#importR1SeqBtn").addEventListener("click", async () => {
    const raw = container.querySelector("#r1ImportTextarea").value.trim();
    if (!raw) return alert("请先粘贴 JSON 内容");
    try {
      const payload = JSON.parse(raw);
      const res = await api.post(`/api/workflows/import-r1pro?robot_id=${encodeURIComponent(api.currentRobotId)}`, payload);
      alert(`成功导入 ${res.imported_count} 个工作流序列！`);
      _loadWorkflows();
    } catch (e) {
      alert("导入失败: " + e.message);
    }
  });

  container.querySelector("#exportR1BundleBtn").addEventListener("click", async () => {
    try {
      const bundle = await api.get(`/api/actions/export-r1pro?robot_id=${encodeURIComponent(api.currentRobotId)}`);
      container.querySelector("#exportR1OutputPre").textContent = JSON.stringify(bundle, null, 2);
    } catch (e) {
      alert("导出失败: " + e.message);
    }
  });

  // Initial load
  _loadJobs();

  return {
    update(state) {
      // Background updates if needed
    }
  };
}
