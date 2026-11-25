// Tab 5: Policy Inference, Dynamic Checkpoints, Offline Replay & Loop Registry
import { api } from "../api.js";
import { policyLayout } from "../policy-layout.js";
import { escapeHtml, formatLocalTime } from "../ui-format.js";
import { notify } from "../notify.js";

export function initPolicyTab(container) {
  let selectedCheckpoint = "";
  let isInferring = false;
  let loopRecords = [];
  let lastRobotId = null;

  container.innerHTML = `
    <!-- Top Status Summary -->
    <div class="card" style="margin-bottom:16px;">
      <div style="display:flex; justify-content:space-between; align-items:center; flex-wrap:wrap; gap:12px;">
        <div style="display:flex; align-items:center; gap:10px; flex-wrap:wrap;">
          <span class="card-title" style="margin:0;">策略推理与切模评估 (Policy &amp; Evaluation)</span>
          <span class="badge badge-no_result" id="policyStatusBadge">推理: STOPPED</span>
          <span class="badge badge-info" id="policyLayoutBadge">Action Layout: 未知 · 等待预检</span>
          <span class="badge badge-no_result" id="policyRecordBadge">未接入采集底座</span>
        </div>
        <div style="display:flex; gap:8px;">
          <button class="btn btn-sm btn-danger" id="policyStopInferBtn" disabled>停止推理</button>
        </div>
      </div>
    </div>

    <!-- Main Grid: Left Checkpoints & 6-Step Safety Wizard; Right Offline Eval & Loop Chain Registry -->
    <div class="grid-2" style="grid-template-columns: 1fr 1fr; align-items:start;">
      <!-- Left Column: Checkpoint Selection, Config & Piper 6-Step Safety Wizard -->
      <div style="display:flex; flex-direction:column; gap:16px;">
        <!-- Checkpoint Selector & Layout Pre-check -->
        <div class="card" style="margin-bottom:0;">
          <div class="card-header" style="margin-bottom:10px;">
            <span class="card-title">权重切模与 Action Layout 预检</span>
            <span class="badge badge-passed" id="checkpointReadyBadge">等待预检</span>
          </div>
          <div style="font-size:12px; color:var(--text-muted); margin-bottom:12px;">
            Mock 固定 Checkpoint 清单（未扫描磁盘或加载真实模型）。下发前自动校验输入 Observation 布局与输出 Action 维度，严禁越界或维度失配下发。
          </div>

          <div style="display:flex; gap:10px; margin-bottom:12px;">
            <select id="checkpointSelect" class="form-select" style="flex:1; padding:6px 10px; font-size:12px; font-family:var(--font-mono);">
              <option value="">等待当前机器人 Checkpoint 清单</option>
            </select>
            <button class="btn btn-sm btn-primary" id="btnSwitchCheckpoint">加载并预检</button>
          </div>

          <!-- Task Intent & Speed Scale Config -->
          <div style="display:grid; grid-template-columns: 1fr 110px auto; gap:8px; align-items:end; margin-bottom:12px; background:var(--bg-subtle); padding:10px; border-radius:var(--radius-sm); border:1px solid var(--border);">
            <div>
              <label class="form-label" for="policyIntentInput" style="font-size:11px;">任务意图 (Task Intent)</label>
              <input type="text" id="policyIntentInput" class="form-input" value="set_the_plates" style="padding:4px 8px; font-size:12px; font-family:var(--font-mono);">
            </div>
            <div>
              <label class="form-label" for="policySpeedInput" style="font-size:11px;">速度比例 (0.1–1.0)</label>
              <input type="number" id="policySpeedInput" class="form-input" value="0.35" min="0.1" max="1.0" step="0.05" style="padding:4px 6px; font-size:12px; font-family:var(--font-mono);">
            </div>
            <button class="btn btn-sm" id="btnApplyPolicyConfig" title="修改配置会重置后端离线评测与武装放行状态">应用配置</button>
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
            <span class="badge badge-pending" id="wizardReadyBadge">放行门禁保护中</span>
          </div>
          <div style="font-size:12px; color:var(--text-muted); margin-bottom:12px;">
            仅 Mock 顺序门禁演示：勾选不执行真实回零/自检，生成评测不读取 MCAP/LeRobot，不可作为硬件放行证据。
          </div>

          <!-- 6-Step Checklist Items -->
          <div style="display:flex; flex-direction:column; gap:8px; font-size:12px;">
            <label style="display:flex; align-items:center; gap:8px; background:var(--bg-subtle); padding:8px 10px; border-radius:var(--radius-sm); cursor:pointer;">
              <input type="checkbox" id="chkStep1" checked disabled style="accent-color:var(--accent);">
              <span style="flex:1;"><strong>1. 通讯与反馈自检:</strong> Mock 仿真反馈可用（非 CAN/DDS 硬件自检）</span>
              <span class="badge badge-passed" style="font-size:10px;">已通过</span>
            </label>

            <label style="display:flex; align-items:center; gap:8px; background:var(--bg-subtle); padding:8px 10px; border-radius:var(--radius-sm); cursor:pointer;">
              <input type="checkbox" id="chkStep2" style="accent-color:var(--accent);">
              <span style="flex:1;"><strong>2. 全关节回零校准 (Homing):</strong> 仅记录 Mock 操作者确认，不执行真实回零</span>
              <button type="button" class="btn btn-sm" id="btnRunHoming" style="font-size:11px; padding:2px 8px;">Mock 回零确认</button>
            </label>

            <label style="display:flex; align-items:center; gap:8px; background:var(--bg-subtle); padding:8px 10px; border-radius:var(--radius-sm); cursor:pointer;">
              <input type="checkbox" id="chkStep3" style="accent-color:var(--accent);">
              <span style="flex:1;"><strong>3. 低速关节行程校验 (Low-Speed):</strong> 0.2 rad/s 点动确认无异响</span>
              <button type="button" class="btn btn-sm" id="btnRunLowSpeed" style="font-size:11px; padding:2px 8px;">点动校验</button>
            </label>

            <label style="display:flex; align-items:center; gap:8px; background:var(--bg-subtle); padding:8px 10px; border-radius:var(--radius-sm); cursor:pointer;">
              <input type="checkbox" id="chkStep4" style="accent-color:var(--accent);">
              <span style="flex:1;"><strong>4. 离线空跑与防碰撞预检 (Dry-Run):</strong> 仅可由右侧 Mock 离线对比回放完成</span>
              <button type="button" class="btn btn-sm" id="btnRunDryRun" style="font-size:11px; padding:2px 8px;">执行空跑</button>
            </label>

            <label style="display:flex; align-items:center; gap:8px; background:var(--bg-subtle); padding:8px 10px; border-radius:var(--radius-sm); cursor:pointer;">
              <input type="checkbox" id="chkStep5" style="accent-color:var(--accent);">
              <span style="flex:1;"><strong>5. 武装进入 Live 状态 (Arm Policy Live):</strong> 确认现场无人员干涉</span>
              <span class="badge badge-no_result" id="badgeStep5">待确认</span>
            </label>

            <div style="margin-top:4px;">
              <button class="btn btn-primary" id="btnStartInference" disabled style="width:100%; padding:10px; font-weight:600; font-size:13px;">
                &gt;&gt; 6. 正式启动策略推理 (Mock Live Inference) &lt;&lt;
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
          <div class="grid-3" style="font-family:var(--font-mono); font-size:12px; margin-bottom:4px;">
            <div style="background:var(--bg-subtle); padding:8px; border-radius:var(--radius-sm);">
              <div style="color:var(--text-muted); font-size:10px;">推理频率:</div>
              <strong id="inferHzVal">0.0 Hz</strong>
            </div>
            <div style="background:var(--bg-subtle); padding:8px; border-radius:var(--radius-sm);">
              <div style="color:var(--text-muted); font-size:10px;">推理来源:</div>
              <strong id="inferLatencyVal">--</strong>
            </div>
            <div style="background:var(--bg-subtle); padding:8px; border-radius:var(--radius-sm);">
              <div style="color:var(--text-muted); font-size:10px;">RTC / Schema:</div>
              <strong id="inferChunkVal">--</strong>
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
            显示后端生成的 Mock 演示曲线（含主/副两轴）；未解析真实 MCAP/LeRobot 数据集，不可作为硬件放行证据。
          </div>
          <div style="display:flex; gap:10px; margin-bottom:10px;">
            <select id="datasetSelect" class="form-select" style="flex:1; padding:6px 10px; font-size:12px; font-family:var(--font-mono);">
              <option value="synthetic_mock_demo">synthetic_mock_demo (Mock 合成演示曲线)</option>
              <option value="set_the_plates_real_all_mcap_20261119_v1">set_the_plates_real_all_mcap_20261119_v1 (名称占位 · 未解析)</option>
              <option value="r1pro_basket_mcap_sample">r1pro_basket_mcap_sample (名称占位 · 未解析)</option>
            </select>
          </div>

          <!-- Comparison Canvas Curve (Motphys Loop HiDPI Light Engineering Style) -->
          <div class="curve-kpi-strip" style="margin-bottom:6px;">
            <span class="curve-kpi-pill"><i class="curve-dot" style="background:#0284c7;"></i>GT1 (主轴)</span>
            <span class="curve-kpi-pill"><i class="curve-dot" style="background:#0d9488;"></i>Pred1 (主轴预测)</span>
            <span class="curve-kpi-pill"><i class="curve-dot" style="background:#94a3b8;"></i>GT2</span>
            <span class="curve-kpi-pill"><i class="curve-dot" style="background:#f59e0b;"></i>Pred2</span>
          </div>
          <div id="replayCurveStage" class="curve-stage" style="height:172px;">
            <canvas id="replayCurveCanvas" class="curve-canvas" style="height:172px; cursor:default;"></canvas>
          </div>
          <div id="replayMetricsOverlay" style="margin-top:6px; font-size:11px; color:var(--text-muted); font-family:var(--font-mono); background:var(--bg-subtle); border:1px solid var(--border); padding:5px 9px; border-radius:4px; overflow:hidden; text-overflow:ellipsis; white-space:nowrap;">
            尚无评测结果 · 不读取 MCAP/LeRobot
          </div>
          <div id="replayProvenanceBox" style="margin-top:8px; font-family:var(--font-mono); font-size:11px; color:var(--text-muted); background:var(--bg-subtle); padding:6px 10px; border-radius:var(--radius-sm); display:none;"></div>
        </div>

        <!-- Episode Recording & Strict Outcome Tagging -->
        <div class="card" style="margin-bottom:0;">
          <div class="card-header" style="margin-bottom:10px;">
            <span class="card-title">Episode 试运行与打标 (Outcome Tagging)</span>
            <span class="badge badge-info" id="episodeTagBadge">结果严格打标</span>
          </div>
          <div style="font-size:12px; color:var(--text-muted); margin-bottom:10px;">
            遵循路线图规范：必须严格区分「零成功 (zero_success)」与「无结果 (no_result)」。真实 MCAP 采集底座尚未接入。
          </div>

          <div style="display:flex; gap:10px; align-items:center; margin-bottom:12px;">
            <button class="btn btn-sm" id="btnToggleRecord" disabled style="flex:1;">真实 Episode 录制未实现</button>
            <button class="btn btn-sm btn-primary" id="btnTagMockEpisode">记录 Mock 打标</button>
            <span id="recordTimerLabel" style="font-family:var(--font-mono); font-size:12px; color:var(--text-muted);">--</span>
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

          <div id="mockEpisodesList" style="margin-top:8px; font-size:11px; color:var(--text-muted);"></div>
        </div>

        <!-- Motphys Loop Object Chain Manual Registry Form -->
        <div class="card" style="margin-bottom:0;">
          <div class="card-header" style="margin-bottom:10px;">
            <span class="card-title">Motphys Loop 对象链登记 (本地 Mock 台账)</span>
            <button class="btn btn-sm btn-primary" id="btnSaveLoopRecord">登记到本地台账</button>
          </div>
          <div style="font-size:12px; color:var(--text-muted); margin-bottom:10px;">
            严格遵循 Loop 对象链结构：<code style="font-size:11px;">Task &rarr; TrainingRun &rarr; ModelArtifact &rarr; EvaluationPlan &rarr; Result + Evidence &rarr; NextAction</code>（仅保存本地 JSON，不同步远端 Loop 服务）。
          </div>

          <div style="display:grid; grid-template-columns: 1fr 1fr; gap:8px; font-size:12px;">
            <div>
              <label class="form-label" style="font-size:11px;">Task Name</label>
              <input type="text" id="loopTaskName" class="form-input" value="volcano_cup_pickup" style="padding:4px 8px; font-family:var(--font-mono); font-size:12px;">
            </div>
            <div>
              <label class="form-label" style="font-size:11px;">Training Run / Run ID</label>
              <input type="text" id="loopTrainingRun" class="form-input" value="tr-202611-piper-04" style="padding:4px 8px; font-family:var(--font-mono); font-size:12px;">
            </div>
            <div>
              <label class="form-label" style="font-size:11px;">Model Artifact</label>
              <input type="text" id="loopModelArtifact" class="form-input" value="art-safetensors-sha256:7f3b" style="padding:4px 8px; font-family:var(--font-mono); font-size:12px;">
            </div>
            <div>
              <label class="form-label" style="font-size:11px;">Evaluation Plan</label>
              <input type="text" id="loopEvalPlan" class="form-input" value="plan-eval-50eps-t0-t2" style="padding:4px 8px; font-family:var(--font-mono); font-size:12px;">
            </div>
            <div style="grid-column: span 2;">
              <label class="form-label" style="font-size:11px;">Evidence MCAP / Video</label>
              <input type="text" id="loopEvidence" class="form-input" value="s3://loop/evidence/eval_ep01.mcap" style="padding:4px 8px; font-family:var(--font-mono); font-size:12px;">
            </div>
            <div style="grid-column: span 2;">
              <label class="form-label" style="font-size:11px;">Next Action (下一阶段建议)</label>
              <input type="text" id="loopNextAction" class="form-input" value="Promote to Staging / Continue Perturbation Evals" style="padding:4px 8px; font-family:var(--font-mono); font-size:12px;">
            </div>
          </div>

          <!-- Existing Loop Records Table -->
          <div style="margin-top:14px; padding-top:12px; border-top:1px solid var(--border);">
            <div style="font-size:12px; font-weight:600; margin-bottom:6px; color:#334155;">已登记 Loop 台账记录 (Local Loop Records)</div>
            <div class="table-wrapper" style="max-height:180px; overflow-y:auto;">
              <table class="data-table" style="font-size:11px;">
                <thead>
                  <tr>
                    <th>Task</th>
                    <th>Model</th>
                    <th>Outcome</th>
                    <th>Next Action</th>
                    <th style="text-align:right;">操作</th>
                  </tr>
                </thead>
                <tbody id="loopRecordsTableBody">
                  <tr><td colspan="5" style="text-align:center; color:#94a3b8;">加载中...</td></tr>
                </tbody>
              </table>
            </div>
          </div>
        </div>
      </div>
    </div>
  `;

  // Draw replay comparison curve on Retina HiDPI canvas (Motphys Loop light engineering style)
  const replayStage = container.querySelector("#replayCurveStage");
  const canvas = container.querySelector("#replayCurveCanvas");
  const ctx = canvas.getContext("2d");
  let replayCssW = 480;
  const replayCssH = 172;
  let lastReplayResult = null;

  function syncReplayCanvasSize() {
    if (!replayStage || !ctx) return;
    const rect = replayStage.getBoundingClientRect();
    const w = Math.max(240, Math.round(rect.width || 480));
    const dpr = Math.min(2, window.devicePixelRatio || 1);
    if (replayCssW !== w || canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(replayCssH * dpr)) {
      replayCssW = w;
      canvas.width = Math.round(w * dpr);
      canvas.height = Math.round(replayCssH * dpr);
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    }
  }
  const replayResizeObserver = new ResizeObserver(() => {
    syncReplayCanvasSize();
    drawReplayCurve(lastReplayResult);
  });
  if (replayStage) replayResizeObserver.observe(replayStage);
  syncReplayCanvasSize();

  function drawReplayCurve(result = null) {
    lastReplayResult = result;
    if (!ctx) return;
    const w = replayCssW;
    const h = replayCssH;
    const padL = 42, padR = 12, padT = 12, padB = 22;
    const plotW = Math.max(60, w - padL - padR);
    const plotH = Math.max(40, h - padT - padB);

    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = "#f8fafc";
    ctx.fillRect(0, 0, w, h);
    ctx.fillStyle = "#ffffff";
    ctx.fillRect(padL, padT, plotW, plotH);
    ctx.strokeStyle = "#e2e8f0";
    ctx.lineWidth = 1;
    ctx.strokeRect(padL + 0.5, padT + 0.5, plotW - 1, plotH - 1);

    const series = result?.series;
    let minV = -1.0;
    let maxV = 1.0;
    if (series) {
      const allVals = [
        ...(series.gt_primary || []),
        ...(series.pred_primary || []),
        ...(series.gt_secondary || []),
        ...(series.pred_secondary || []),
      ].filter(Number.isFinite);
      if (allVals.length) {
        const lo = Math.min(...allVals);
        const hi = Math.max(...allVals);
        const span = Math.max(0.4, (hi - lo) * 1.3);
        const mid = (lo + hi) * 0.5;
        minV = mid - span * 0.5;
        maxV = mid + span * 0.5;
      }
    }
    const toY = (v) => padT + ((maxV - v) / Math.max(1e-4, maxV - minV)) * plotH;

    // Gridlines + Y-axis labels
    ctx.font = "500 10px ui-monospace, SFMono-Regular, Menlo, monospace";
    for (let i = 0; i <= 4; i++) {
      const frac = i / 4;
      const y = padT + frac * plotH;
      if (i > 0 && i < 4) {
        ctx.strokeStyle = "#eef2f6";
        ctx.beginPath();
        ctx.moveTo(padL, Math.round(y) + 0.5);
        ctx.lineTo(padL + plotW, Math.round(y) + 0.5);
        ctx.stroke();
      }
      ctx.fillStyle = "#64748b";
      ctx.textAlign = "right";
      ctx.fillText((maxV - frac * (maxV - minV)).toFixed(2), padL - 6, y + 3.5);
    }
    if (minV < 0 && maxV > 0) {
      const y0 = Math.round(toY(0)) + 0.5;
      ctx.strokeStyle = "#cbd5e1";
      ctx.beginPath();
      ctx.moveTo(padL, y0);
      ctx.lineTo(padL + plotW, y0);
      ctx.stroke();
    }

    if (!series) {
      ctx.fillStyle = "#94a3b8";
      ctx.font = "500 12px ui-sans-serif, system-ui, sans-serif";
      ctx.textAlign = "center";
      ctx.fillText("点击「启动离线对比回放」生成合成 Mock 评测曲线", padL + plotW * 0.5, padT + plotH * 0.5 + 4);
      ctx.textAlign = "left";
      return;
    }

    const gt1 = series.gt_primary || [];
    const pr1 = series.pred_primary || [];
    const totalPts = Math.max(gt1.length, pr1.length, 2);
    const toX = (i, n) => padL + (i / Math.max(1, n - 1)) * plotW;

    // Translucent GT1-vs-Pred1 error fill
    if (gt1.length >= 2 && pr1.length === gt1.length) {
      ctx.beginPath();
      gt1.forEach((v, i) => (i === 0 ? ctx.moveTo(toX(i, gt1.length), toY(v)) : ctx.lineTo(toX(i, gt1.length), toY(v))));
      for (let i = pr1.length - 1; i >= 0; i--) ctx.lineTo(toX(i, pr1.length), toY(pr1[i]));
      ctx.closePath();
      ctx.fillStyle = "rgba(13, 148, 136, 0.11)";
      ctx.fill();
    }

    const lines = [
      ["gt_secondary", "#94a3b8", [], 1.3],
      ["pred_secondary", "#f59e0b", [4, 3], 1.5],
      ["gt_primary", "#0284c7", [], 2.1],
      ["pred_primary", "#0d9488", [5, 3], 2.0],
    ];
    for (const [key, color, dash, lw] of lines) {
      const values = series[key] || [];
      if (!values.length) continue;
      ctx.strokeStyle = color;
      ctx.lineWidth = lw;
      ctx.lineJoin = "round";
      ctx.lineCap = "round";
      ctx.setLineDash(dash);
      ctx.beginPath();
      values.forEach((v, i) => {
        const x = toX(i, values.length);
        const y = toY(v);
        if (i) ctx.lineTo(x, y);
        else ctx.moveTo(x, y);
      });
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // X-axis frame ticks
    ctx.fillStyle = "#94a3b8";
    ctx.font = "500 10px ui-monospace, SFMono-Regular, Menlo, monospace";
    ctx.textAlign = "center";
    for (const frac of [0, 0.25, 0.5, 0.75, 1]) {
      const x = padL + frac * plotW;
      const idx = Math.round(frac * (totalPts - 1));
      ctx.fillText(`f${idx}`, frac === 0 ? x + 8 : frac === 1 ? x - 10 : x, h - 6);
    }
    ctx.textAlign = "left";
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
  let configSyncedRobot = null;

  function checkWizardGate() {
    const steps = currentPolicy?.wizard_steps || {};
    chk2.checked = !!steps.homing;
    chk3.checked = !!steps.manual_slow_test;
    chk4.checked = !!steps.offline_replay;
    chk5.checked = !!steps.live_armed;
    const ready = !currentPolicy?.requires_wizard || Object.values(steps).every(Boolean);
    btnStartInfer.disabled = !controlAllowed || !ready || isInferring;
    const badge = container.querySelector("#wizardReadyBadge");
    badge.textContent = controlAllowed && ready ? "Mock 门禁就绪" : "Mock 门禁保护中";
    badge.className = `badge ${controlAllowed && ready ? "badge-passed" : "badge-pending"}`;
    armWarning.textContent = !controlAllowed
      ? "请先获取底层控制 Lease 并确保无急停锁存。"
      : ready
        ? "所有 Mock 前置门禁已通过，可启动 Mock 策略推理演示。"
        : "顺序步骤由后端校验；所有确认与评测仅对 Mock 演示有效。";
    container.querySelector("#badgeStep5").textContent = steps.live_armed ? "Mock ARMED" : "待确认";
  }

  function renderPolicy(state) {
    const policy = state.observation?.policy || {};
    currentPolicy = { ...policy, requires_wizard: !!state.descriptor?.capabilities?.piper_safety_wizard };
    controlAllowed = !!state.authority?.lease_active && !state.authority?.estop_latched;

    const options = policy.available_checkpoints || [];
    const signature = JSON.stringify([state.descriptor?.id, options]);
    if (signature !== checkpointSignature) {
      ckSelect.replaceChildren(...options.map((c) => {
        const opt = document.createElement("option");
        opt.value = c.id;
        opt.textContent = `${c.name} · ${c.action_dim} DOF${c.compatible ? "" : " · incompatible"}`;
        return opt;
      }));
      checkpointSignature = signature;
    }

    if (configSyncedRobot !== state.descriptor?.id) {
      configSyncedRobot = state.descriptor?.id;
      if (policy.task_intent) container.querySelector("#policyIntentInput").value = policy.task_intent;
      if (policy.speed_scale) container.querySelector("#policySpeedInput").value = policy.speed_scale;
    }

    selectedCheckpoint = policy.active_checkpoint || "";
    ckSelect.value = selectedCheckpoint;
    const active = options.find((c) => c.id === selectedCheckpoint);
    const layout = policyLayout(state.descriptor, active);
    const layoutBadge = container.querySelector("#policyLayoutBadge");
    layoutBadge.textContent = layout.text;
    layoutBadge.className = `badge ${layout.status === "matched" ? "badge-info" : "badge-pending"}`;
    const precheckBadge = container.querySelector("#checkpointReadyBadge");
    precheckBadge.textContent = layout.status === "matched" ? "Mock 维度预检通过" : "未知或不匹配";
    precheckBadge.className = `badge ${layout.status === "matched" ? "badge-passed" : "badge-pending"}`;
    container.querySelector("#modelActionDim").textContent = active?.action_dim ?? "--";
    container.querySelector("#inferChunkVal").textContent = `${policy.rtc_enabled ? "RTC ON" : "RTC OFF"} · ${policy.execution_schema || "--"}`;

    isInferring = !!policy.infer_running;
    updateInferUI();

    const result = policy.last_offline_eval;
    container.querySelector("#replayMetricsOverlay").textContent = result
      ? `MOCK · RMSE ${result.rmse_rad} rad · max_jump ${result.max_jump_rad} rad · 未解析数据集 / 非硬件证据`
      : "尚无评测结果 · 未解析数据集";
    const provBox = container.querySelector("#replayProvenanceBox");
    if (provBox) {
      if (result) {
        provBox.style.display = "block";
        provBox.textContent = `${result.provenance || ""} | ${result.safety_contract || ""}`;
      } else {
        provBox.style.display = "none";
      }
    }
    drawReplayCurve(result);

    // Render tagged mock episodes if present
    const epEl = container.querySelector("#mockEpisodesList");
    const eps = policy.recorded_episodes || [];
    if (epEl) {
      epEl.innerHTML = eps.length
        ? `<div style="margin-top:4px; font-weight:600; color:var(--text-main);">最近 Mock 打标 (${eps.length})</div>` +
          eps.slice(0, 4).map(ep =>
            `<div style="display:flex; justify-content:space-between; padding:3px 0; border-bottom:1px solid var(--border); font-family:var(--font-mono);">
              <span>${escapeHtml(ep.id)} · ${escapeHtml(ep.checkpoint || "--")}</span>
              <span class="badge badge-${escapeHtml(ep.outcome === "passed" ? "passed" : ep.outcome === "failed" || ep.outcome === "zero_success" ? "failed" : "no_result")}">${escapeHtml(ep.outcome)}</span>
            </div>`
          ).join("")
        : "";
    }
  }

  async function mutatePolicy(action, payload = {}, successMsg = "") {
    try {
      await api.controlPolicy(action, payload);
      renderPolicy(await api.getState());
      if (successMsg) notify(successMsg, "success");
    } catch (err) {
      checkWizardGate();
      notify(`策略操作未完成: ${err.message}`, "error");
    }
  }

  const evaluate = () => {
    const dataset = container.querySelector("#datasetSelect")?.value || "synthetic_mock_demo";
    mutatePolicy("offline_eval", { dataset }, "Mock 合成离线对比回放已完成（非真实数据集评测）");
  };
  container.querySelector("#btnRunOfflineReplay").addEventListener("click", evaluate);
  container.querySelector("#btnSwitchCheckpoint").addEventListener("click", () =>
    mutatePolicy("switch_checkpoint", { checkpoint_id: ckSelect.value }, `已切换检查点至 ${ckSelect.value}（向导状态已安全复位）`)
  );
  container.querySelector("#btnApplyPolicyConfig").addEventListener("click", () => {
    const task_intent = container.querySelector("#policyIntentInput").value.trim() || "set_the_plates";
    const speed_scale = parseFloat(container.querySelector("#policySpeedInput").value) || 0.35;
    mutatePolicy("set_config", { task_intent, speed_scale }, `已应用策略配置 (intent=${task_intent}, speed=${speed_scale})`);
  });
  container.querySelector("#btnRunHoming").addEventListener("click", () => mutatePolicy("wizard_step", { step: "homing", done: true }, "步骤② Mock 回零已确认"));
  container.querySelector("#btnRunLowSpeed").addEventListener("click", () => mutatePolicy("wizard_step", { step: "manual_slow_test", done: true }, "步骤③ 低速行程已确认"));
  container.querySelector("#btnRunDryRun").addEventListener("click", evaluate);
  for (const [checkbox, step] of [[chk2, "homing"], [chk3, "manual_slow_test"], [chk5, "live_armed"]]) {
    checkbox.addEventListener("change", () => mutatePolicy("wizard_step", { step, done: checkbox.checked }));
  }
  chk4.disabled = true; // Only evaluation can complete this step.

  // Start & Stop Inference
  btnStartInfer.addEventListener("click", async () => {
    try {
      await api.controlPolicy("start_infer", { checkpoint: selectedCheckpoint });
      renderPolicy(await api.getState());
      notify("Mock 策略推理已启动", "success");
    } catch (err) {
      notify(`启动推理失败: ${err.message}`, "error");
    }
  });

  container.querySelector("#policyStopInferBtn").addEventListener("click", async () => {
    try {
      await api.controlPolicy("stop_infer");
      renderPolicy(await api.getState());
      notify("策略推理已停止", "info");
    } catch (err) {
      notify(`停止推理失败: ${err.message}`, "error");
    }
  });

  // Tag Mock Episode
  container.querySelector("#btnTagMockEpisode").addEventListener("click", () => {
    const outcome = container.querySelector('input[name="outcomeTag"]:checked').value;
    mutatePolicy("tag_episode", { outcome, frames: 240, notes: "UI Mock tag" }, `已记录 Mock Episode 打标: ${outcome}`);
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
      latEl.textContent = "Synthetic Mock";
    } else {
      badge.textContent = "推理: STOPPED";
      badge.className = "badge badge-no_result";
      stopBtn.disabled = true;
      checkWizardGate();
      hzEl.textContent = "0.0 Hz";
      latEl.textContent = "--";
    }
  }

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
      tbody.innerHTML = `<tr><td colspan="5" style="text-align:center; color:#94a3b8;">当前机型暂无 Loop 登记记录</td></tr>`;
      return;
    }
    tbody.innerHTML = loopRecords
      .map((r) => {
        const cls = r.eval_status === "passed" ? "badge-passed" : (r.eval_status === "failed" || r.eval_status === "zero_success") ? "badge-failed" : "badge-pending";
        return `
        <tr>
          <td><strong>${escapeHtml(r.task_name)}</strong><div style="font-size:10px; color:var(--text-subtle);">${formatLocalTime(r.created_at)}</div></td>
          <td><code style="font-size:10px;">${escapeHtml(r.model_artifact)}</code></td>
          <td><span class="badge ${cls}">${escapeHtml(r.eval_status)}</span></td>
          <td style="color:#64748b;">${escapeHtml(r.next_action)}</td>
          <td style="text-align:right;"><button class="btn btn-sm del-loop-btn" data-recid="${escapeHtml(r.record_id)}" style="padding:2px 6px; font-size:10px; color:var(--danger);">删除</button></td>
        </tr>
      `;
      })
      .join("");

    tbody.querySelectorAll(".del-loop-btn").forEach(btn => {
      btn.addEventListener("click", async () => {
        const recId = btn.getAttribute("data-recid");
        try {
          await api.deleteLoopRecord(recId);
          notify("已删除本地 Loop 台账记录", "info");
          reloadLoopRecords();
        } catch (err) {
          notify(`删除记录失败: ${err.message}`, "error");
        }
      });
    });
  }

  container.querySelector("#btnSaveLoopRecord").addEventListener("click", async () => {
    const outcome = container.querySelector('input[name="outcomeTag"]:checked').value;
    const payload = {
      task_name: container.querySelector("#loopTaskName").value.trim() || "unnamed_task",
      source_type: "training_run",
      source_ref: container.querySelector("#loopTrainingRun").value.trim(),
      model_artifact: container.querySelector("#loopModelArtifact").value.trim(),
      eval_plan: container.querySelector("#loopEvalPlan").value.trim(),
      eval_status: outcome,
      evidence_run_id: container.querySelector("#loopEvidence").value.trim(),
      next_action: container.querySelector("#loopNextAction").value.trim(),
      robot_id: api.currentRobotId,
    };
    try {
      await api.saveLoopRecord(payload);
      notify("已保存本地 Mock 台账；未同步远端 Loop 服务", "success");
      reloadLoopRecords();
    } catch (err) {
      notify(`登记失败: ${err.message}`, "error");
    }
  });

  reloadLoopRecords();

  return {
    update(state) {
      if (!state) return;
      const desc = state.descriptor || {};
      if (lastRobotId !== desc.id) {
        lastRobotId = desc.id;
        reloadLoopRecords();
      }
      renderPolicy(state);

      const robotDimEl = container.querySelector("#robotActionDim");
      if (robotDimEl && desc.joints) {
        robotDimEl.textContent = desc.joints.length;
      }
    },
  };
}
