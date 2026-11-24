// Tab 5: Policy Inference, Dynamic Checkpoints, Offline Replay & Loop Registry
import { api } from "../api.js";
import { policyLayout } from "../policy-layout.js";

export function initPolicyTab(container) {
  let selectedCheckpoint = "piper_cup_pickup_v3.safetensors";
  let checkSteps = {
    step1: true,  // Connection self-check
    step2: false, // Homing
    step3: false, // Low-speed test
    step4: false, // Offline dry-run
    step5: false, // Arm Live
  };
  let isInferring = false;
  let loopRecords = [];

  container.innerHTML = `
    <!-- Top Status Summary -->
    <div class="card" style="margin-bottom:16px;">
      <div style="display:flex; justify-content:space-between; align-items:center; flex-wrap:wrap; gap:12px;">
        <div style="display:flex; align-items:center; gap:12px;">
          <span class="card-title" style="margin:0;">策略推理与切模评估 (Policy & Evaluation)</span>
          <span class="badge badge-idle" id="policyStatusBadge">推理: STOPPED</span>
          <span class="badge badge-info" id="policyLayoutBadge">Action Layout: 未知 · 等待预检</span>
          <span class="badge badge-idle" id="policyRecordBadge">录制: 未开始</span>
        </div>
        <div style="display:flex; gap:8px;">
          <button class="btn btn-sm btn-danger" id="policyStopInferBtn" disabled>停止推理</button>
        </div>
      </div>
    </div>

    <!-- Main Grid: Left Checkpoints & 6-Step Safety Wizard; Right Offline Eval & Loop Chain Registry -->
    <div class="grid-2" style="grid-template-columns: 1fr 1fr; align-items:start;">
      <!-- Left Column: Checkpoint Selection & Piper 6-Step Safety Wizard -->
      <div style="display:flex; flex-direction:column; gap:16px;">
        <!-- Checkpoint Selector & Layout Pre-check -->
        <div class="card" style="margin-bottom:0;">
          <div class="card-header" style="margin-bottom:10px;">
            <span class="card-title">权重切模与 Action Layout 预检</span>
            <span class="badge badge-passed" id="checkpointReadyBadge">预检通过</span>
          </div>
          <div style="font-size:12px; color:var(--text-muted); margin-bottom:12px;">
            Mock 固定 Checkpoint 清单（未扫描磁盘或加载真实模型）。下发前自动校验输入 Observation 布局与输出 Action 维度，严禁越界或维度失配下发。
          </div>

          <div style="display:flex; gap:10px; margin-bottom:12px;">
            <select id="checkpointSelect" class="form-control" style="flex:1; padding:6px 10px; font-size:13px; border:1px solid var(--border); border-radius:var(--radius-sm); font-family:var(--font-mono);">
              <option value="piper_cup_pickup_v3.safetensors">piper_cup_pickup_v3.safetensors (ACT · 14-DOF · 30Hz)</option>
              <option value="piper_insert_gear_v1.safetensors">piper_insert_gear_v1.safetensors (Diffusion · 14-DOF · 50Hz)</option>
              <option value="r1pro_table_cleaning_v2.pt">r1pro_table_cleaning_v2.pt (OpenVLA · 20-DOF · 20Hz)</option>
              <option value="dex_w1_stand_balance_v1.onnx">dex_w1_stand_balance_v1.onnx (RL MLP · 20-DOF · 100Hz)</option>
            </select>
            <button class="btn btn-sm btn-primary" id="btnSwitchCheckpoint">加载并预检</button>
          </div>

          <div id="layoutCheckInfoBox" style="background:var(--bg-subtle); padding:10px 12px; border-radius:var(--radius-sm); font-size:11px; font-family:var(--font-mono); color:#334155;">
            <div>&check; 模型 Action 维度: <span id="modelActionDim">--</span> 维</div>
            <div>&check; 当前机器人关节数: <span id="robotActionDim">--</span> 关节</div>
            <div>&check; 观测空间: 以当前 Mock RobotDescriptor 为准；未接入真实传感器</div>
            <div style="color:var(--success); font-weight:600; margin-top:4px;">&rarr; 仅做 Mock Action 维度校验；不代表模型或传感器接入通过。</div>
          </div>
        </div>

        <!-- Piper-Style 6-Step Safety Checklist Wizard -->
        <div class="card" style="margin-bottom:0; border-left:4px solid var(--warning);">
          <div class="card-header" style="margin-bottom:8px;">
            <span class="card-title">Piper 风格 6 步安全放行向导 (Pre-Run Arming)</span>
            <span class="badge badge-warning" id="wizardReadyBadge">放行门禁保护中</span>
          </div>
          <div style="font-size:12px; color:var(--text-muted); margin-bottom:12px;">
            仅 Mock 顺序门禁演示：勾选不执行真实回零/自检，生成评测不读取 MCAP/LeRobot，不可作为硬件放行证据。
          </div>

          <!-- 6-Step Checklist Items -->
          <div style="display:flex; flex-direction:column; gap:8px; font-size:12px;">
            <!-- Step 1 -->
            <label style="display:flex; align-items:center; gap:8px; background:var(--bg-subtle); padding:8px 10px; border-radius:var(--radius-sm); cursor:pointer;">
              <input type="checkbox" id="chkStep1" checked disabled style="accent-color:var(--accent);">
              <span style="flex:1;"><strong>1. 通讯与反馈自检:</strong> Mock 仿真反馈可用（非 CAN/DDS 硬件自检）</span>
              <span class="badge badge-passed" style="font-size:10px;">已通过</span>
            </label>

            <!-- Step 2 -->
            <label style="display:flex; align-items:center; gap:8px; background:var(--bg-subtle); padding:8px 10px; border-radius:var(--radius-sm); cursor:pointer;">
              <input type="checkbox" id="chkStep2" style="accent-color:var(--accent);">
              <span style="flex:1;"><strong>2. 全关节回零校准 (Homing):</strong> 仅记录 Mock 操作者确认，不执行真实回零</span>
              <button class="btn btn-sm" id="btnRunHoming" style="font-size:10px; padding:2px 8px;">Mock 回零确认</button>
            </label>

            <!-- Step 3 -->
            <label style="display:flex; align-items:center; gap:8px; background:var(--bg-subtle); padding:8px 10px; border-radius:var(--radius-sm); cursor:pointer;">
              <input type="checkbox" id="chkStep3" style="accent-color:var(--accent);">
              <span style="flex:1;"><strong>3. 低速关节行程校验 (Low-Speed):</strong> 0.2 rad/s 点动确认无异响</span>
              <button class="btn btn-sm" id="btnRunLowSpeed" style="font-size:10px; padding:2px 8px;">点动校验</button>
            </label>

            <!-- Step 4 -->
            <label style="display:flex; align-items:center; gap:8px; background:var(--bg-subtle); padding:8px 10px; border-radius:var(--radius-sm); cursor:pointer;">
              <input type="checkbox" id="chkStep4" style="accent-color:var(--accent);">
              <span style="flex:1;"><strong>4. 离线空跑与防碰撞预检 (Dry-Run):</strong> 纯计算轨迹仿真碰撞检测</span>
              <button class="btn btn-sm" id="btnRunDryRun" style="font-size:10px; padding:2px 8px;">执行空跑</button>
            </label>

            <!-- Step 5 -->
            <label style="display:flex; align-items:center; gap:8px; background:var(--bg-subtle); padding:8px 10px; border-radius:var(--radius-sm); cursor:pointer;">
              <input type="checkbox" id="chkStep5" style="accent-color:var(--accent);">
              <span style="flex:1;"><strong>5. 武装进入 Live 状态 (Arm Policy Live):</strong> 确认现场无人员干涉</span>
              <span class="badge badge-idle" id="badgeStep5">待武装</span>
            </label>

            <!-- Step 6 Final Button -->
            <div style="margin-top:4px;">
              <button class="btn btn-primary" id="btnStartInference" disabled style="width:100%; padding:10px; font-weight:600; font-size:13px;">
                &gt;&gt; 6. 正式启动策略推理 (Live Inference) &lt;&lt;
              </button>
              <div id="armWarningText" style="font-size:11px; color:#b45309; text-align:center; margin-top:4px;">
                前置 5 项安全检查未就绪，启动按钮锁定中
              </div>
            </div>
          </div>
        </div>

        <!-- Live Inference Metrics -->
        <div class="card" style="margin-bottom:0;">
          <div class="card-header" style="margin-bottom:8px;">
            <span class="card-title">推理运行态指标 (Runtime Metrics)</span>
          </div>
          <div class="grid-3" style="font-family:var(--font-mono); font-size:12px; margin-bottom:8px;">
            <div style="background:var(--bg-subtle); padding:8px; border-radius:var(--radius-sm);">
              <div style="color:var(--text-muted); font-size:10px;">推理频率:</div>
              <strong id="inferHzVal">0.0 Hz</strong>
            </div>
            <div style="background:var(--bg-subtle); padding:8px; border-radius:var(--radius-sm);">
              <div style="color:var(--text-muted); font-size:10px;">Forward 耗时:</div>
              <strong id="inferLatencyVal">-- ms</strong>
            </div>
            <div style="background:var(--bg-subtle); padding:8px; border-radius:var(--radius-sm);">
              <div style="color:var(--text-muted); font-size:10px;">Action Chunk:</div>
              <strong id="inferChunkVal">16 步</strong>
            </div>
          </div>
        </div>
      </div>

      <!-- Right Column: Offline Replay, Episode Tagging & Loop Registry -->
      <div style="display:flex; flex-direction:column; gap:16px;">
        <!-- Offline Replay & Curve Comparison -->
        <div class="card" style="margin-bottom:0;">
          <div class="card-header" style="margin-bottom:10px;">
            <span class="card-title">LeRobot / MCAP 离线开环回放与对比</span>
            <button class="btn btn-sm btn-primary" id="btnRunOfflineReplay">启动离线对比回放</button>
          </div>
          <div style="font-size:12px; color:var(--text-muted); margin-bottom:10px;">
            显示后端生成的 Mock 演示曲线；未解析数据集，未运行真实模型。
          </div>
          <div style="display:flex; gap:10px; margin-bottom:10px;">
            <select id="datasetSelect" class="form-control" style="flex:1; padding:6px 10px; font-size:12px; border:1px solid var(--border); border-radius:var(--radius-sm); font-family:var(--font-mono);">
              <option value="dataset_piper_cup_ep01.mcap">dataset_piper_cup_ep01.mcap (专家示教 240 帧)</option>
              <option value="dataset_piper_cup_ep02.mcap">dataset_piper_cup_ep02.mcap (专家示教 210 帧)</option>
              <option value="dataset_r1pro_wipe_ep05.mcap">dataset_r1pro_wipe_ep05.mcap (桌面擦拭 350 帧)</option>
            </select>
          </div>

          <!-- Comparison Canvas Curve -->
          <div style="position:relative; width:100%; height:140px; background:#0f172a; border-radius:var(--radius-sm); overflow:hidden; border:1px solid var(--border);">
            <canvas id="replayCurveCanvas" width="450" height="140" style="width:100%; height:100%; display:block;"></canvas>
            <div style="position:absolute; top:6px; right:8px; font-size:10px; color:#94a3b8; font-family:var(--font-mono);">
              <span style="color:#38bdf8;">— 专家真实 (GT)</span> &nbsp; <span style="color:#0d9488;">-- 策略预测 (Pred)</span>
            </div>
            <div style="position:absolute; bottom:6px; left:8px; font-size:10px; color:#cbd5e1; font-family:var(--font-mono);" id="replayMetricsOverlay">
              尚无评测结果 · 不读取 MCAP/LeRobot
            </div>
          </div>
        </div>

        <!-- Episode Recording & Strict Outcome Tagging -->
        <div class="card" style="margin-bottom:0;">
          <div class="card-header" style="margin-bottom:10px;">
            <span class="card-title">Episode 试运行与打标 (Outcome Tagging)</span>
            <span class="badge badge-info" id="episodeTagBadge">结果严格打标</span>
          </div>
          <div style="font-size:12px; color:var(--text-muted); margin-bottom:10px;">
            遵循路线图规范：必须严格区分「零成功 (zero_success)」与「无结果 (no_result)」。
          </div>

          <div style="display:flex; gap:10px; align-items:center; margin-bottom:12px;">
            <button class="btn btn-sm btn-primary" id="btnToggleRecord" style="flex:1;">开始录制当前 Episode</button>
            <span id="recordTimerLabel" style="font-family:var(--font-mono); font-size:12px; color:var(--text-muted);">00:00.0</span>
          </div>

          <!-- Tagging Radio Group -->
          <div style="background:var(--bg-subtle); padding:10px; border-radius:var(--radius-sm); font-size:12px;">
            <div style="font-weight:600; margin-bottom:6px; color:#334155;">评测结论打标 (Outcome Tag):</div>
            <div style="display:grid; grid-template-columns: 1fr 1fr; gap:6px;">
              <label style="display:flex; align-items:center; gap:6px; cursor:pointer;">
                <input type="radio" name="outcomeTag" value="passed" checked style="accent-color:var(--success);">
                <span style="color:var(--success); font-weight:500;">&check; passed (完全成功)</span>
              </label>
              <label style="display:flex; align-items:center; gap:6px; cursor:pointer;">
                <input type="radio" name="outcomeTag" value="failed" style="accent-color:var(--danger);">
                <span style="color:var(--danger); font-weight:500;">&times; failed (动作失败)</span>
              </label>
              <label style="display:flex; align-items:center; gap:6px; cursor:pointer;">
                <input type="radio" name="outcomeTag" value="zero_success" style="accent-color:#e11d48;">
                <span style="color:#e11d48; font-weight:500;">&empty; zero_success (0成功率)</span>
              </label>
              <label style="display:flex; align-items:center; gap:6px; cursor:pointer;">
                <input type="radio" name="outcomeTag" value="no_result" style="accent-color:#64748b;">
                <span style="color:#64748b; font-weight:500;">? no_result (无结果/干预)</span>
              </label>
            </div>
          </div>
        </div>

        <!-- Motphys Loop Object Chain Manual Registry Form -->
        <div class="card" style="margin-bottom:0;">
          <div class="card-header" style="margin-bottom:10px;">
            <span class="card-title">Motphys Loop 对象链登记 (Loop Chain)</span>
            <button class="btn btn-sm btn-primary" id="btnSaveLoopRecord">登记到 Loop</button>
          </div>
          <div style="font-size:12px; color:var(--text-muted); margin-bottom:10px;">
            严格遵循 Loop 对象链结构：<code style="font-size:11px;">Task &rarr; TrainingRun &rarr; ModelArtifact &rarr; EvaluationPlan &rarr; Result + Evidence &rarr; NextAction</code>。
          </div>

          <div style="display:grid; grid-template-columns: 1fr 1fr; gap:8px; font-size:12px;">
            <div>
              <label style="color:var(--text-muted); font-size:11px;">Task Name</label>
              <input type="text" id="loopTaskName" value="volcano_cup_pickup" style="width:100%; padding:4px 8px; border:1px solid var(--border); border-radius:4px; font-family:var(--font-mono); font-size:12px;">
            </div>
            <div>
              <label style="color:var(--text-muted); font-size:11px;">Training Run / Run ID</label>
              <input type="text" id="loopTrainingRun" value="tr-202611-piper-04" style="width:100%; padding:4px 8px; border:1px solid var(--border); border-radius:4px; font-family:var(--font-mono); font-size:12px;">
            </div>
            <div>
              <label style="color:var(--text-muted); font-size:11px;">Model Artifact</label>
              <input type="text" id="loopModelArtifact" value="art-safetensors-sha256:7f3b" style="width:100%; padding:4px 8px; border:1px solid var(--border); border-radius:4px; font-family:var(--font-mono); font-size:12px;">
            </div>
            <div>
              <label style="color:var(--text-muted); font-size:11px;">Evaluation Plan</label>
              <input type="text" id="loopEvalPlan" value="plan-eval-50eps-t0-t2" style="width:100%; padding:4px 8px; border:1px solid var(--border); border-radius:4px; font-family:var(--font-mono); font-size:12px;">
            </div>
            <div style="grid-column: span 2;">
              <label style="color:var(--text-muted); font-size:11px;">Evidence MCAP / Video</label>
              <input type="text" id="loopEvidence" value="s3://loop/evidence/eval_ep01.mcap" style="width:100%; padding:4px 8px; border:1px solid var(--border); border-radius:4px; font-family:var(--font-mono); font-size:12px;">
            </div>
            <div style="grid-column: span 2;">
              <label style="color:var(--text-muted); font-size:11px;">Next Action (下一阶段建议)</label>
              <input type="text" id="loopNextAction" value="Promote to Staging / Continue Perturbation Evals" style="width:100%; padding:4px 8px; border:1px solid var(--border); border-radius:4px; font-family:var(--font-mono); font-size:12px;">
            </div>
          </div>

          <!-- Existing Loop Records Table -->
          <div style="margin-top:14px; pt-2; border-top:1px solid var(--border);">
            <div style="font-size:12px; font-weight:600; margin-bottom:6px; color:#334155;">已登记 Loop 记录 (Loop Records)</div>
            <div class="table-wrapper" style="max-height:160px; overflow-y:auto;">
              <table class="data-table" style="font-size:11px;">
                <thead>
                  <tr>
                    <th>Task</th>
                    <th>Model</th>
                    <th>Outcome</th>
                    <th>Next Action</th>
                  </tr>
                </thead>
                <tbody id="loopRecordsTableBody">
                  <tr><td colspan="4" style="text-align:center; color:#94a3b8;">加载中...</td></tr>
                </tbody>
              </table>
            </div>
          </div>
        </div>
      </div>
    </div>
  `;

  // Draw initial replay comparison curve on canvas
  const canvas = container.querySelector("#replayCurveCanvas");
  const ctx = canvas.getContext("2d");

  function drawReplayCurve(result = null) {
    if (!ctx) return;
    ctx.fillStyle = "#0b0b0e";
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    const series = result?.series;
    if (!series) {
      ctx.fillStyle = "#a8b0b8"; ctx.font = "12px monospace";
      ctx.fillText("No evaluation · synthetic Mock only", 15, 65); return;
    }
    for (const [key, color] of [["gt_primary", "#38bdf8"], ["pred_primary", "#45aeb2"]]) {
      const values = series[key] || [];
      ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.beginPath();
      values.forEach((v, i) => {const x = 12 + i / Math.max(1, values.length - 1) * (canvas.width - 24);
        const y = canvas.height / 2 - v * 80; if (i) ctx.lineTo(x, y); else ctx.moveTo(x, y);});
      ctx.stroke();
    }
  }
  drawReplayCurve();
  const ckSelect = container.querySelector("#checkpointSelect");
  const chk2 = container.querySelector("#chkStep2");
  const chk3 = container.querySelector("#chkStep3");
  const chk4 = container.querySelector("#chkStep4");
  const chk5 = container.querySelector("#chkStep5");
  const btnStartInfer = container.querySelector("#btnStartInference");
  const armWarning = container.querySelector("#armWarningText");
  let currentPolicy = null;
  let controlAllowed = false;
  let checkpointSignature = "";

  function checkWizardGate() {
    const steps = currentPolicy?.wizard_steps || {};
    chk2.checked = !!steps.homing; chk3.checked = !!steps.manual_slow_test;
    chk4.checked = !!steps.offline_replay; chk5.checked = !!steps.live_armed;
    const ready = !currentPolicy?.requires_wizard || Object.values(steps).every(Boolean);
    btnStartInfer.disabled = !controlAllowed || !ready || isInferring;
    const badge = container.querySelector("#wizardReadyBadge");
    badge.textContent = controlAllowed && ready ? "Mock 门禁就绪" : "Mock 门禁保护中";
    badge.className = `badge ${controlAllowed && ready ? "badge-passed" : "badge-warning"}`;
    armWarning.textContent = "顺序步骤由后端校验；所有确认与评测仅对 Mock 演示有效。";
    container.querySelector("#badgeStep5").textContent = steps.live_armed ? "Mock ARMED" : "待确认";
  }
  function renderPolicy(state) {
    const policy = state.observation?.policy || {};
    currentPolicy = {...policy, requires_wizard: !!state.descriptor?.capabilities?.piper_safety_wizard};
    controlAllowed = !!state.authority?.lease_active && !state.authority?.estop_latched;
    const options = policy.available_checkpoints || [];
    const signature = JSON.stringify([state.descriptor?.id, options]);
    if (signature !== checkpointSignature) {
      ckSelect.replaceChildren(...options.map((c) => {const opt = document.createElement("option");
        opt.value = c.id; opt.textContent = `${c.name} · ${c.action_dim} DOF${c.compatible ? "" : " · incompatible"}`; return opt;}));
      checkpointSignature = signature;
    }
    selectedCheckpoint = policy.active_checkpoint || ""; ckSelect.value = selectedCheckpoint;
    const active = options.find((c) => c.id === selectedCheckpoint);
    const layout = policyLayout(state.descriptor, active);
    const layoutBadge = container.querySelector("#policyLayoutBadge");
    layoutBadge.textContent = layout.text;
    layoutBadge.className = `badge ${layout.status === "matched" ? "badge-info" : "badge-warning"}`;
    container.querySelector("#modelActionDim").textContent = active?.action_dim ?? "--";
    isInferring = !!policy.infer_running; updateInferUI();
    const result = policy.last_offline_eval;
    container.querySelector("#replayMetricsOverlay").textContent = result ? `MOCK · RMSE ${result.rmse_rad} rad · 未解析数据集 / 非硬件证据` : "尚无评测结果 · 未解析数据集";
    drawReplayCurve(result);
  }
  async function mutatePolicy(action, payload = {}) {
    try {await api.controlPolicy(action, payload); renderPolicy(await api.getState());}
    catch (err) {checkWizardGate(); alert(`Mock 操作未完成: ${err.message}`);}
  }
  const evaluate = () => mutatePolicy("offline_eval", {dataset: "synthetic_mock_demo"});
  container.querySelector("#btnRunOfflineReplay").addEventListener("click", evaluate);
  container.querySelector("#btnSwitchCheckpoint").addEventListener("click", () => mutatePolicy("switch_checkpoint", {checkpoint_id: ckSelect.value}));
  container.querySelector("#btnRunHoming").addEventListener("click", () => mutatePolicy("wizard_step", {step: "homing", done: true}));
  container.querySelector("#btnRunLowSpeed").addEventListener("click", () => mutatePolicy("wizard_step", {step: "manual_slow_test", done: true}));
  container.querySelector("#btnRunDryRun").addEventListener("click", evaluate);
  for (const [checkbox, step] of [[chk2, "homing"], [chk3, "manual_slow_test"], [chk5, "live_armed"]]) {
    checkbox.addEventListener("change", () => mutatePolicy("wizard_step", {step, done: checkbox.checked}));
  }
  chk4.disabled = true; // Only evaluation can complete this step.

  // Start & Stop Inference
  btnStartInfer.addEventListener("click", async () => {
    try {
      await api.controlPolicy("start_infer", { checkpoint: selectedCheckpoint });
      renderPolicy(await api.getState());
    } catch (err) {
      alert(`启动推理失败: ${err.message}`);
    }
  });

  container.querySelector("#policyStopInferBtn").addEventListener("click", async () => {
    try {
      await api.controlPolicy("stop_infer");
      renderPolicy(await api.getState());
    } catch (err) {
      alert(`停止推理失败: ${err.message}`);
    }
  });

  function updateInferUI() {
    const badge = container.querySelector("#policyStatusBadge");
    const stopBtn = container.querySelector("#policyStopInferBtn");
    const hzEl = container.querySelector("#inferHzVal");
    const latEl = container.querySelector("#inferLatencyVal");

    if (isInferring) {
      badge.textContent = "推理: Mock 运行中";
      badge.className = "badge badge-running";
      stopBtn.disabled = false;
      btnStartInfer.disabled = true;
      hzEl.textContent = `${currentPolicy?.infer_hz ?? 0} Hz`;
      latEl.textContent = "Mock";
    } else {
      badge.textContent = "推理: STOPPED";
      badge.className = "badge badge-idle";
      stopBtn.disabled = true;
      checkWizardGate();
      hzEl.textContent = "0.0 Hz";
      latEl.textContent = "-- ms";
    }
  }

  // Episode Recording
  let recordTimer = null;
  const btnToggleRec = container.querySelector("#btnToggleRecord");
  const recBadge = container.querySelector("#policyRecordBadge");
  const recTimerLabel = container.querySelector("#recordTimerLabel");

  btnToggleRec.disabled = true;
  btnToggleRec.textContent = "真实 Episode 录制未实现";
  recBadge.textContent = "未接入采集底座";
  recTimerLabel.textContent = "--";

  // Loop Chain Registry
  async function reloadLoopRecords() {
    try {
      const res = await api.listLoopRecords();
      loopRecords = res.records || [];
      renderLoopRecords();
    } catch (err) {
      console.warn("Load loop records failed:", err);
    }
  }

  function renderLoopRecords() {
    const tbody = container.querySelector("#loopRecordsTableBody");
    if (!tbody) return;
    if (loopRecords.length === 0) {
      tbody.innerHTML = `<tr><td colspan="4" style="text-align:center; color:#94a3b8;">暂无 Loop 登记记录</td></tr>`;
      return;
    }
    tbody.innerHTML = loopRecords
      .map(
        (r) => `
        <tr>
          <td><strong>${r.task_name}</strong></td>
          <td><code style="font-size:10px;">${r.model_artifact}</code></td>
          <td><span class="badge ${r.eval_status === "passed" ? "badge-passed" : r.eval_status === "failed" ? "badge-failed" : "badge-warning"}">${r.eval_status}</span></td>
          <td style="color:#64748b;">${r.next_action}</td>
        </tr>
      `
      )
      .join("");
  }

  container.querySelector("#btnSaveLoopRecord").addEventListener("click", async () => {
    const outcome = container.querySelector('input[name="outcomeTag"]:checked').value;
    const payload = {
      task_name: container.querySelector("#loopTaskName").value,
      source_type: "training_run",
      source_ref: container.querySelector("#loopTrainingRun").value,
      model_artifact: container.querySelector("#loopModelArtifact").value,
      eval_plan: container.querySelector("#loopEvalPlan").value,
      eval_status: outcome,
      evidence_run_id: container.querySelector("#loopEvidence").value,
      next_action: container.querySelector("#loopNextAction").value,
      robot_id: api.currentRobotId,
    };
    try {
      await api.saveLoopRecord(payload);
      alert("已保存本地 Mock 台账；未同步 Loop 服务");
      reloadLoopRecords();
    } catch (err) {
      alert(`登记失败: ${err.message}`);
    }
  });

  reloadLoopRecords();

  return {
    update(state) {
      if (!state) return;
      const desc = state.descriptor || {};
      const obs = state.observation || {};
      renderPolicy(state);

      const robotDimEl = container.querySelector("#robotActionDim");
      if (robotDimEl && desc.joints) {
        robotDimEl.textContent = desc.joints.length;
      }
    },
  };
}
