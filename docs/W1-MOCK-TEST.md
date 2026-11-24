# W1 全流程 Mock 测试记录

2026-11-24 14:53 Asia/Shanghai；build `LOO1-20261124-pose-hands-v3`。样本 `dex-w1-mock`。

范围：真实接口的进程内 TestClient + 实际前端预览函数；隔离临时数据；socket.connect 禁止。不是浏览器/WebGL视觉验收。

|步骤|输入|预期|实际/证据|结果|
|---|---|---|---|---|
|机型能力|GET W1 state|20身体关节、Mock、无Lease|20 / mock / lease=false；导航、策略能力false|PASS|
|模型资源|逐一GET并校验SHA-256|21GLB + 2手URDF/24STL|47文件HTTP200且hash相同；身体10张内嵌图，手无纹理|PASS|
|本地身体/手预览|ANKLE_PITCH=-0.65；左T_MCP=0.6|input立即更新模型，不伪造反馈/发请求|模型关节=-0.65，反馈=0.00；手目标0.6；请求0；身体命令20键且无手键|PASS|
|原子动作保存/选用|POST /api/actions，joint_targets|保存后列表能选同一ID，无执行|动作库确有同一ID/名称，运动分派0|PASS|
|工作流编排|保存两个action_name引用|列表包含工作流，两步骤|引用/列表/步骤数2均一致|PASS|
|本地dry-run/详情/Steps/Logs|run dry_run=true|Succeeded/exit0，步骤/日志可读|Succeeded/0；四阶段全succeeded（含起止）；日志8条；运动分派0|PASS|
|故障停止|首动作引用故意不存在；后续正常动作|Failed/exit1，后续不执行|succeeded/failed/canceled/canceled；分派0；没有硬件停机调用|PASS|
|取消|对运行中dry-run POST cancel|Canceled|Canceled/exit130|PASS|
|重试|对取消任务retry dry_run=true|新ID且成功，不转live|新ID，Succeeded，lease_id=null，分派0|PASS|
|安全边界|无Lease发目标；只读桥反向command|403 / 只读拒绝|lease_required / readonly_mirror_violation；分派0|PASS|
|切机/最终复核|W1→Piper；再送旧W1观察|旧目标/手不串值，反馈独立|旧值清空；W1反馈仍0.00；全部Lease inactive；外部连接0|PASS|

原始证据（box，不入库）：`/workspace/loo1-tracking/w1-flow-v3/w1-mock-flow.json`、同目录Markdown。测试生成ID已从本审阅记录省略，避免提交运行数据。

复现：`uv run python scripts/verify_w1_mock_flow.py --report-dir /tmp/w1-mock-report`；另由pytest自动回归该流程。

未覆盖/待实现：浏览器鼠标交互和WebGL视觉装配/纹理解码验收；真实机械手反馈与控制；CAN/Edge/ROS/真机；真实Lease/硬件急停；采集上传、真实数据评测、推理和Motrix WASM。W1地图导航/策略明确能力缺失，保留禁用入口与原因。
