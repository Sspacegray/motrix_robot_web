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
- 此处最初 W1 网格未加载的限制已由下文 14:03 模型增量取代：W1 接入本地 URDF/21 GLB；浏览器视觉验收仍待父代理。R1/Piper 真实模型仍缺失；没有 Motrix WASM 仿真器，只保留只读消息桥。
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

## 13:43 Asia/Shanghai 用户澄清：默认主页面、按需二级详情

- 已逐项复读只读 Loop `layouts/main-layout/main-layout.vue`、header 组件/导航、`pages/action-page.vue`、`components/title-section.vue`、`components/execution/execution-tasks.vue`。
- 真实源码证据：main-layout 只有 Header + 自适应 page-container/router-view；action-page 使用白色 title-section + 二级 tabs，内容区是 tab-panel；execution-tasks 的 `detailOpen = ref(false)`，仅 `openDetail` 点击行改 true、`closeDetail` 改 false。因此不是默认左右分栏。
- 本项目抽屉原本就默认关闭：构造只创建 `.drawer-panel` / `.drawer-backdrop`，不带 `.open`，panel 初始 inert；CSS 默认 visibility:hidden 且 fixed 右侧移出，遮罩 display:none；仅显式 `open(jobId)` 展示，`close()` 删除 open 和页面滚动锁。没有预留 760px 列宽，不需要重写/进一步强化抽屉。
- 找到真正多余层级：品牌全局导航后，又重复一整行七个机器人功能 tabs。此次删除该重复条，将现有七个真实功能统一映射到唯一全局导航；保留标题卡内的页面二级 tabs。标题/字号/留白按 title-section 改为 Loop Actions / 24px / 20px×24px，不再加额外 eyebrow 与介绍段。
- 与原版有意保留差异：全局菜单映射机器人已有功能，不造 Projects/Datatable/Access Control 空链接；标题 tabs 是 Jobs/Workflows/Atomic actions/Import-Export，不冒充未实现 Templates/Triggers；机器人状态/权限/急停条为本应用安全所需，保留。源码 Loop 默认 Templates，本项目首屏 Jobs 是当前验收目标。原生 ESM，不迁 Quasar/Vue。
- 验证：新增 `tests/test_drawer_lifecycle.mjs`，运行实际抽屉 controller 的最小 DOM double，验证启动无请求/无 open、选中任务才展示、关闭恢复；另验证每功能只有唯一导航、无固定分栏。Node 合计 9 passed；这不是浏览器或截图验收。
- 新增/修改文件：index.html 去重复导航、actions.js 标题、style.css 源码对应尺寸、drawer lifecycle 测试及本文。原因是减少层级并证实默认全宽。回退只需新建本增量 revert，保留之前安全修复。
- 给父代理默认首页截图路径：打开/强刷 `http://127.0.0.1:8096/`，不点击任务、不点击 Mock dry-run，即为全宽默认主页面；若已有抽屉打开，按 Esc 或右上 × 或点击遮罩关闭。建议输出 `/workspace/loo1-tracking/loop-default-main-v3.png`，尚未生成，待父代理派浏览器验收。Mac 未同步。

## Motphys 原品牌资产修正（用户 13:46 Asia/Shanghai 要求）

- 真实 header 来源为 `frontend/src/components/images/kleinkram-logo.vue`（由真实 header-component 引用），不是手画三角。它使用 `/motphys-logo.svg`，20px 高、`filter: brightness(0)`，右侧 16px 间距放置 20px/600/18px 行高的 Loop 字样，小屏隐藏 Loop 字样。
- 仅逐字节复制参考仓库 `frontend/public/motphys-logo.svg` → 本项目 `frontend/assets/motphys-logo.svg`，`motphys-icon.svg` → `frontend/assets/motphys-icon.svg`；前者 viewBox 398.2×64，保留比例，后者用作 favicon。未生成、近似描摹、修改 SVG 路径或整包复制参考。
- 修改 index.html 和 style.css 替换旧三角/近似文字，按真实组件尺寸与滤镜显示；新增 `tests/test_brand_assets.mjs` 验证引用、比例样式与原始文件 SHA-256 固定值，并排除 SVG 主动脚本/外链。两资产的原字节完全保留。
- 许可来源：参考仓库根 LICENSE 为 MIT，原版权与全文已保留在 `docs/licenses/LOOP-MIT.txt`；本次未发现资产单独许可声明。此记录仅说明源码包许可来源，不把 MIT 解释为额外商标权或对外背书；使用范围限用户明确要求的本项目品牌视觉。
- 回退：为本资产修正提交创建新的 revert，恢复之前 UI 即可；不改后端、不影响安全边界。浏览器验收 URL 仍为 `http://127.0.0.1:8096/`，刷新查看左上 Motphys 原图及浏览器 favicon；Mac 仍未同步。

品牌增量验证：11 项 Node 测试通过，git diff --check 通过，源码秘密/运行数据扫描无命中；loopback HTTP 返回的 logo 与原资产字节一致。未宣称完成浏览器截图验收。

## 14:03 Asia/Shanghai 两项缺陷：真实模型与本地三角品牌

### 诊断证据（不是先归因缓存）
- 8096 和 8097 的进程 cwd 均为 box `/workspace/motrix_robot_web`；StaticFiles 明确挂载该项目 frontend。两端根页面最初均返回当前 Motphys SVG，而不是旧三角；当时 HTTP 有 Last-Modified/ETag，无显式 Cache-Control。
- 对 Mac 授权项目只读检查发现 `frontend/index.html` 第19行仍为三角 data favicon，第27–30行仍为手画三角和旧 Motphys 文字。因此 Mac 页面三角有明确旧源码证据，不能把它称为浏览器缓存或宣称 box 更新已部署 Mac。未修改 Mac 文件。
- box 全品牌位置用原 Motphys header SVG/favicon；增加可见 build ID `LOO1-20261124-model-v1`，入口 JS/CSS/品牌 URL 带相同版本。服务新增 HTML/JS/CSS `no-cache, must-revalidate`（Python 服务重启后生效）。没有增加假兜底三角。

### 最小真实模型资产与实现
- 原项目 Mac `frontend/assets/urdf/dexforce_w1_v030/robot.urdf` 已有21个 visual GLB引用；box 此前只带URDF文本，运行代码根本没有实例化URDFLoader，只画程序骨架。现只读逐文件安全复制该URDF实际引用的21个GLB（约26MiB），未复制collision、手部STL库或整包参考。源均来自用户授权现有项目，不访问旧参考库、不读.env。
- GLB原始解析证明：21个几何场景，含原有PBR材质，其中10个GLB各内嵌1张纹理，共10张，无外部buffer/image URL、无必需压缩扩展。其余11个是几何/纯色材质，未给它们绘制假贴图。源模型不含额外BrainCo手部装配；`robot_with_ee.urdf`/手部库本批不导入。
- 新增 `model-registry.js` 严格同时核对 `dex-w1-mock` 与 `dexforce_w1`；W1绝不映射到Piper/R1。R1和Piper缺各自模型，不再显示程序骨架冒充，明确空态提示。当前授权项目清单只找到W1模型；需要用户之后提供R1Pro、Piper各自URDF及引用mesh/材质纹理，才可补齐。
- `robot-model-viewer.js` 使用现有URDFLoader，补齐项目已有Three r160的GLTF/STL/Collada/TGA loader和BufferGeometryUtils（5个JS，MIT同原Three LICENSE），按URDF层级/origin/axis/scale装配，Z-up转Y-up、相机自动包围取景、窗口/标签切换ResizeObserver；只把当前Mock joint_positions映射同名urdf_joint。保留GLTF scene Group/PBR材质贴图，不用统一颜色覆盖。等全部21视觉与纹理解码后统计；少于10张实际已解码图像会标partial，失败不假报成功。
- 新模型模块不导入api/命令通道，不写CAN/ROS，资源限制同源模型目录；切机丢弃旧异步结果并释放资源。默认仍无Lease。真实Edge和Motrix WASM未实现状态不变。
- 资产清单及每文件字节数/SHA256/材质/纹理数在 `visual-manifest.json`。资产属于用户现有项目，原URDF DexForce署名保留；未找到单独模型许可，不把模型改标MIT，也不宣称拥有对外再分发权。仅授权私有项目使用；Three和Loop原许可不变。

### 验证和交接
- `tests/test_model_assets.py` 校验21引用全覆盖、文件hash/GLB结构、内嵌纹理10、无外部资源、20关节映射、W1独有能力，以及所有模型/loader实际HTTP路径。`test_model_registry.mjs` 覆盖跨机型拒绝和只读加载接线。品牌回归允许显式版本URL且保留原SVG hash。
- 最新验证：64 Python passed、1条原上游弃用warning；13 Node passed；ESM语法、diff检查通过。用Node vm链接并执行10个实际离线ESM依赖模块通过（实验性vm warning；不是浏览器）。这些不能替代WebGL视觉加载验收，尚未声称浏览器确已显示模型和10张贴图。
- 父代理入口：`http://127.0.0.1:8096/?tab=joints&robot=dex-w1-mock`（URL会选择实际注册机型且不申请Lease）。检查页面build ID、HUD `21/21 GLB`、贴图解码统计、W1外观、Orbit旋转缩放；切Piper/R1应明确模型缺失而不是仍显示W1。建议截图 `/workspace/loo1-tracking/w1-real-model-v1.png`（尚未生成）。
- 修改文件用途：app.py缓存验证头；mock.py如实声明W1静态模型；index/app/build CSS可见版本和直接验收入口；joints/model modules真实只读渲染并删除假骨架；5个vendor依赖、21资产与manifest构成最小离线依赖；测试与本文记录验证/来源。回退新建本批revert，不强推；保留之前后端安全修复。Mac没有写入、部署或同步。

预览状态补充：已仅重启 box 的8096 Mock服务使新模型能力与 no-cache 验证头生效，loopback实测正确；8097未重启，虽静态文件同根，Python描述符仍可能旧，统一用8096验收。不涉及Mac服务。

## 14:37 Asia/Shanghai 功能入口恢复（navigation-v2）

- 根因已定位：`app.js::_updateCapabilitiesUI` 不仅按能力 `hidden/display:none`，还额外重复隐藏 Navigation。用户选择 W1 时 navigation/policy 都为 false，故整项消失；感知入口误仅判断 head_depth/YOLO/ArUco，忽略 W1/Piper 明明具有 `cameras=true`。不是模块文件丢失。原子动作库仍在 Actions 内，但英文 Actions/Atomic actions/Jobs 的层级容易被误解为 Jobs 替代了动作库。
- 修复为始终保留唯一一套七项全局入口：总览、动作与工作流、关节/3D、地图导航、视觉感知、策略评测、安全诊断。无能力的入口 disabled 并提供具体原因、建议切换的机型，页面也显示原因；不隐藏、不造假能力。R1 可进导航，W1/Piper 导航禁用；W1 策略禁用，Piper/R1 策略可用。相机存在即可进入视觉感知，内部未支持的深度/YOLO/6D/ArUco启动按钮禁用。
- Actions 清晰保留三个互不替代的真实页面：原子动作库（真实 listActions/创建/导入/删除）、工作流编排（listWorkflows/已有 Dry-run/受控 Mock 执行）、执行任务记录（listJobs/详情）。新增 `section=actions|workflows|jobs|r1pro` 直达链接，切机刷新当前二级页，原地图/感知/策略模块代码继续保留。未新造原子动作直接执行API或真实硬件能力。
- 新增 `navigation-state.js` 统一能力可达性，`test_navigation_access.mjs` 验证七入口不消失、三机型矩阵、相机-only感知、禁用原因、切机恢复及原子/流程/任务分立。W1真实模型、品牌及安全后端无改动。build 标识更新为 `LOO1-20261124-navigation-v2`。
- 验收URL：`/?tab=navigation&robot=r1pro-mock` 地图导航；`/?tab=actions&section=actions&robot=r1pro-mock` 原子动作；section=workflows 为流程、section=jobs 为记录；`/?tab=perception&robot=dex-w1-mock` W1相机入口；`/?tab=policy&robot=piper-dual-mock` Piper策略。浏览器视觉验收仍由父代理进行。用户图片为父代理私有附件，子代理 Read 被拒，未绕过；以上依据实际源码定位。
- 文件目的：index/app/nav-state恢复完整可见入口和守卫；actions明确二级页及直达/切机更新；perception区分相机能力与算法启动；CSS禁用样式/说明；测试和本文记录回归。回退仅revert本修复，不回退安全/模型/品牌修复。若同步Mac，只备份并部署这些已测增量，保留全部配置和已有Mock数据；不在Mac开发。
