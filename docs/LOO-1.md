# LOO-1 — 首批安全与 Loop 核心 UI

2026-11-24，Asia/Shanghai。工作分支：`loop/loo-1-safety-ui`。

## 已实现（仅本地 Mock）

- 参照提供的 Loop 源码，而非视觉猜测：`frontend/src/css/app.scss`、`quasar.variables.sass` 和 `execution-task-detail-drawer.vue`。主色 `#167E83`、画布 `#F5F7F8`、文字 `#172126`、64px 顶栏、工作区面包屑、二级标签、紧凑表格、760px 抽屉、元信息/Steps/`#0b0b0e` Logs/Follow/Retry/Cancel。保持原生 ESM、离线静态资源、无 CDN、无前端构建；不是 Vue/Quasar 的 1:1 移植，没有迁移认证。
- 四类未实现真实连接器强制只读并返回断联/空关节遥测、`integration_status=unimplemented`、`telemetry_source=unavailable`。无 Mock 回退、无假 `forwarded_to`，不发网络或硬件请求。
- 默认无活动租约；活动租约不可被静默接管。3 秒心跳/300 秒闲置在同步调用和后台都检查；超时撤销并停机，心跳不能复活过期租约；动作不充当心跳。
- 切机先释放旧机器人并停止心跳；新机授权也撤销其他本地租约；降级 viewer 停机并中止工作流。
- 感知/策略/故障变更增加后端权限门禁；停机/取消与只读拾点保留可达。工作流在分派前重新验证，中断/失败停机。
- Piper 的 Mock 前置步骤必须顺序完成；评测步骤不能用勾选伪造；切换检查点、配置变更或停机会失效后续放行。生成评测明确 `synthetic_mock`、`dataset_parsed=false`、`qualified_for_hardware=false`。
- 策略 UI 接实际后端状态/检查点清单/生成曲线，删除固定 RMSE 成功弹窗；不再以浏览器复选框替代后端状态。未实现的 Episode 采集按钮禁用；Loop 台账明确仅本地保存。
- 急停无需租约并锁存，解除后不自动授权；回包明确不确认真实硬件停机，界面不再宣称网关/硬件已阻断。运动请求拒绝 NaN/Inf。

## 验证

- `uv run pytest -q`：62 passed；1 条 FastAPI/Starlette 对 httpx 的上游弃用警告（不是零警告）。覆盖三构型、适配器 fail-closed、权限旁路、过期心跳/闲置、切机、急停、策略顺序、有限数值、只读镜像与工作流。
- `node --test tests/test_client_api.mjs`：3 passed，验证旧机释放顺序、失败不切换和旧遥测丢弃。
- 所有 `frontend/js/*.js`、`frontend/js/tabs/*.js` 经 Node ESM 语法检查。
- 仅通过本机 TestClient 与 loopback HTTP 验证，无真实机器人/CAN/远程生产连接。执行代理未进行浏览器视觉验收；视觉与交互应由父代理另验，不能把静态 token 测试称为视觉验收。

## 预览与运行

```sh
uv sync --extra dev
uv run python -m motrix_robot_web --host 127.0.0.1 --port 8097 --data-dir /workspace/loo1-preview-data
```

父代理可在 box 浏览器打开 `http://127.0.0.1:8097/`，查看总览、关节只读程序骨架、动作/工作流 Dry-run 抽屉；切到 Piper 查看顺序门禁。预览使用默认三台 Mock，不加载配置文件。Mac 原目录没有修改、安装、执行、部署或同步；只做清单与源文件复制，另产生了一个临时过滤传输副本。

## 明确未完成 / 禁用

- 没有真实 Edge 鉴权、Token/Lease、传输、状态读取或远程停机送达；Demo role 和 Mock Lease 不是生产认证。真实适配器即使配置 `readonly_only: false` 也不会打开写控。
- W1 URDF 网格未加载；`urdf_3d=false`，3D 仍是标注清楚的只读程序骨架。没有 Motrix WASM 仿真器，只保留只读消息桥。
- 未解析 MCAP/LeRobot、未加载真实策略模型、未实现真实 Episode 采集；生成评测不可作为硬件放行证据。
- Piper 防第二 CAN 写者仅为本进程配置防线，不是进程外资源仲裁；本批根本不提供真实写通路。未来真实集成仍需 Edge 的认证/Lease/资源互斥与独立验收。
- 首次源码导入为最小可运行选集：原项目源码、地图、当前运行所需 Three.js/OrbitControls、URDF 文本/加载器存档；没有上传完整 W1 mesh/STL/GLB 包、完整未使用 vendor loader 集、参考 ZIP 或其余参考仓。URDF 加载器未接到运行视图，其离线依赖与网格需后续补齐；不能声称完整资产包已交付。

## 文件变更目的、验证和回退

| 文件组 | 用途 / 原因 | 验证 | 回退 |
|---|---|---|---|
| `app.py`, `safety.py`, `workflow_engine.py`, `schemas.py` | 堵写入口、同步 watchdog、停机联动、拒绝非有限值、诚实元数据 | Python API/安全/工作流测试 | 整批回退，不应独立回退安全模块 |
| `connectors/{unavailable,edge,r1pro,piper_lelab,dex_w1,mock}.py` | 真实骨架 fail-closed；Mock 顺序和来源 | 适配器/策略回归 | 同上；不得恢复假转发用于实机 |
| `frontend/css/style.css`, `index.html`, `js/{app,api,drawer,policy-layout}.js` | 真实 Loop 核心布局、无租约切机、抽屉 | token/API/Node 语法及客户端状态测试；视觉待验 | 可单独恢复 UI，保留后端安全 |
| `frontend/js/tabs/{policy,joints,overview}.js` | 策略接后端；禁用假录制；只读模型/仿真与遥测来源说明 | Mock/Node；视觉待验 | 同上 |
| `tests/test_loo1_regression.py`, `tests/test_loo1_regressions.py`, `tests/test_client_api.mjs`, 原测试调整 | 新约束的负面用例与顺序流程 | 上述命令 | 随实现回退，勿仅删除失败测试 |
| `.gitignore`, `pyproject.toml`, `README.md`, 本文与许可证 | 排除 data/密钥/缓存、明确异步 fixture、真实状态/端口、保留许可 | git 文件清单/秘密扫描/测试 | 保留忽略规则与许可 |

未合并时直接关闭 PR 即可回退，不影响 Mac 原项目；已合并后用新的 revert 提交整批回退，不强推或改写已推历史。

## 许可与导入安全

Loop 使用 MIT，原声明在 `docs/licenses/LOOP-MIT.txt`；Three.js 原 MIT 保留在 `frontend/vendor/three/LICENSE`；URDF Loaders 上游 Apache-2.0 声明在 `docs/licenses/URDF-LOADERS-APACHE-2.0.txt`（来源 `https://github.com/gkjohnson/urdf-loaders`）。本批没有给用户项目重新指定整体许可证。

导入排除了 `.env`/密钥/token/credentials 文件、旧 `.git` 历史、虚拟环境、缓存、日志、运行数据、symlink、生成 egg-info。未读取 `.env`/密钥内容。源码扫描只输出文件路径/计数，不输出疑似秘密值。临时归档因设备允许根路径限制无法传入 box；未重试该受限路径，后续只用 sanctioned CopyToBox 对授权项目内的选定普通文件逐个复制。

`gv` 按本机现有帮助登记到 `/workspace/loo1-tracking/LOO-1.{json,log}`；没有宣称同步远端 grok_viewer hub。当前技能目录查询无 `huashu` 匹配，未因此阻塞主线。

## 用户截图纠正后的 UI 增量（同一 PR，不重建）

- 这批不是仅改颜色：顶栏改为品牌 + Overview / Actions / Policies / Diagnostics（全部映射已有功能），机器人选择与权限/急停移入独立状态条。默认进入 Actions，使用灰画布、面包屑、独立白色标题卡与底部二级 tabs、独立紧凑 Jobs 表卡。
- Jobs 增加真实数据搜索、状态筛选、计数与空态说明；`Mock dry-run` 调用当前机器人第一个现有工作流的真实本地验证 API，不造任务行、不发送运动指令，完成后打开同一任务的详情。搜索/切机丢弃旧请求；遥测任务状态变化会更新任务表。
- 760px 抽屉改成 Execution task 标题、独立状态/ID 行、两列元信息、Steps 表、黑底 Logs 与 Follow；遮罩、Esc、焦点约束/恢复和滚动锁。Retry 仅对有源工作流的终态任务启用，并明确 `Retry dry-run`；不会用重试按钮意外发起 Live。Cancel 仅运行中显示，走原有停机/取消 API。
- 新增 `frontend/js/ui-format.js` 用于 HTML 转义、状态/文本筛选与 Asia/Shanghai 时间；新增 `tests/test_ui_format.mjs` 对这些纯函数及接线做回归。禁用未接线的可视编排新建按钮，原子动作创建及 JSON 导入/导出仍保留。
- 新增/修改目的：完整修正信息层级与执行详情，保留原生 ESM/FastAPI 离线架构、全部原安全边界及许可证。回退：对本 UI 增量提交创建新的 revert（不强推），不回退之前的后端安全修复。
- 验证：`uv run pytest -q` 为 62 passed、1 条原上游弃用警告；`node --test tests/test_client_api.mjs tests/test_ui_format.mjs` 为 7 passed；全部 ESM 语法与 `git diff --check` 通过。R1Pro 20DOF 与 Piper 14DOF 的 Action Layout 动态回归已包含在 Node 测试中。
- 视觉状态：子代理尝试 Read 用户图片时工具明确拒绝为父代理私有附件；未绕过或读取。仅按用户明确描述和只读 Loop 源码重构。这里不宣称已与图片逐像素匹配，也没有把单元测试当作浏览器验收。父代理需亲自读取原图，并在 box `http://127.0.0.1:8096/` 强制刷新查看默认 Actions → Jobs，点击 `Mock dry-run` 查看遮罩抽屉；截图建议保存 `/workspace/loo1-tracking/loop-jobs-v2.png` 与 `loop-drawer-v2.png`（尚未生成）。Mac 原目录仍未同步。
