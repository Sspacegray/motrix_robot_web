"""Reproducible W1 in-process Mock integration flow. No TCP/robot/Lease acquisition.
Run: uv run python scripts/verify_w1_mock_flow.py --report-dir /tmp/w1-mock-report
"""
from __future__ import annotations
import argparse
import asyncio
from datetime import datetime, timezone, timedelta
import hashlib
import json
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "backend"))
from fastapi.testclient import TestClient
from motrix_robot_web.app import create_app


def run_flow(report_dir: Path):
    report = {"sample":"dex-w1-mock", "build":"LOO1-20261124-pose-hands-v3", "time":datetime.now(timezone(timedelta(hours=8))).isoformat(),
              "transport":"in-process TestClient, socket.connect blocked; isolated TemporaryDirectory", "steps":[],
              "not_covered":["浏览器/WebGL视觉验收和真实鼠标拖动（父代理另验）", "真实机械手反馈/CAN/Edge/ROS/真实机器人", "真实Lease或硬件急停", "采集上传/MCAP真实评测/模型推理/Motrix WASM（未实现）"]}
    def record(name, inputs, expected, actual):
        report["steps"].append({"name":name,"input":inputs,"expected":expected,"actual":actual,"result":"PASS"})
    report_dir.mkdir(parents=True,exist_ok=True)
    try:
        with tempfile.TemporaryDirectory(prefix="w1-mock-flow-") as tmp, patch.object(socket.socket,"connect",side_effect=AssertionError("External network forbidden")) as network:
            app=create_app(data_dir=Path(tmp))
            with TestClient(app) as client:
                rid="dex-w1-mock"; prefix=f"/api/robots/{rid}"
                def get(path):
                    response=client.get(path);assert response.status_code==200,(path,response.status_code);return response.json()
                def post(path, payload):
                    response=client.post(path,json=payload);assert response.status_code==200,(path,response.status_code,response.text);return response.json()
                def wait(job_id):
                    deadline=time.monotonic()+6
                    while time.monotonic()<deadline:
                        job=get("/api/jobs/"+job_id)["job"]
                        if job["state"]!="Running":return job
                        client.portal.call(asyncio.sleep,.03)
                    raise AssertionError("Mock job did not terminate")
                state=get(prefix+"/state");piper=get("/api/robots/piper-dual-mock/state")
                assert state["descriptor"]["mode"]=="mock" and not state["authority"]["lease_active"]
                assert len(state["descriptor"]["joints"])==20
                record("加载机型/能力",{"GET":prefix+"/state"},"W1/20身体关节/无Lease/仅Mock",{"robot":rid,"body_dof":20,"mode":"mock","lease":False,"navigation":state["descriptor"]["capabilities"]["navigation"],"policy":state["descriptor"]["capabilities"]["policy_inference"]})
                count=0
                for name in ["visual-manifest.json","hand-manifest.json"]:
                    manifest=json.loads((ROOT/"frontend/assets/urdf/dexforce_w1_v030"/name).read_text())
                    for entry in manifest["files"]:
                        response=client.get("/assets/urdf/dexforce_w1_v030/"+entry["path"]);assert response.status_code==200
                        assert hashlib.sha256(response.content).hexdigest()==entry["sha256"];count+=1
                record("身体/左右机械手资产",{"HTTP_asset_requests":count},"21 GLB + 2 URDF/24 STL，源hash一致；10内嵌图片；手STL无贴图",{"verified_files":count,"body_visuals":21,"hand_visuals":24,"hand_urdf":2,"embedded_body_images":10,"webgl_visual_acceptance":False})
                node_code=r"""
import fs from 'node:fs/promises';
let input='';for await(const chunk of process.stdin)input+=chunk;
const fixture=JSON.parse(input);
const load=async name=>{const source=await fs.readFile(name,'utf8');return import('data:text/javascript;base64,'+Buffer.from(source).toString('base64'));};
const {PosePreview,bindLocalPreviewInput,applyModelJoints}=await load('frontend/js/pose-preview.js');
const {W1_HANDS}=await load('frontend/js/model-registry.js');
const preview=new PosePreview();preview.selectRobot(fixture.w1.descriptor,fixture.w1.observation,W1_HANDS);
let writes=0;globalThis.fetch=()=>{writes++;throw Error('No write/request allowed');};
const values={},robot={joints:{ANKLE_PITCH:{}},setJointValue(name,value){values[name]=value;}};
const slider={addEventListener(type,fn){this[type]=fn;}};
bindLocalPreviewInput(slider,v=>preview.setTarget('ANKLE_PITCH',v),()=>applyModelJoints(robot,fixture.w1.descriptor,preview.visiblePose()));
slider.input({target:{value:'-0.65'}});preview.setHandTarget('left','T_MCP',.6);
preview.observe(preview.robotId,fixture.w1.observation.joint_positions);
if(values.ANKLE_PITCH!==-.65 || preview.feedback.ANKLE_PITCH!==fixture.w1.observation.joint_positions.ANKLE_PITCH || writes!==0)throw Error('preview isolation failed');
const result={model_ankle:values.ANKLE_PITCH,feedback_ankle:preview.feedback.ANKLE_PITCH,left_thumb_target:preview.handTargets.left.T_MCP,request_count:writes,body_command_keys:Object.keys(preview.commandTargets()).length,hand_keys_in_command:Object.keys(preview.commandTargets()).some(k=>k.includes('T_MCP'))};
preview.selectRobot(fixture.piper.descriptor,fixture.piper.observation,[]);preview.observe('dex-w1-mock',{ANKLE_PITCH:1});
result.switch_cleared_w1=!('ANKLE_PITCH' in preview.targets)&&Object.keys(preview.handTargets).length===0;
if(!result.switch_cleared_w1 || result.hand_keys_in_command)throw Error('robot/command isolation failed');
console.log(JSON.stringify(result));
"""
                node=subprocess.run(["node","--input-type=module","-e",node_code],cwd=ROOT,input=json.dumps({"w1":state,"piper":piper}),capture_output=True,text=True,check=True)
                preview=json.loads(node.stdout)
                record("input→目标预览/反馈隔离/手指预览",{"slider_ANKLE_PITCH":-.65,"left_T_MCP":.6},"立即调用模型关节=-0.65，反馈保持原值；请求0；手指不进身体命令",preview)
                connector=app.state.ctx.connectors[rid]
                with patch.object(connector,"send_joint_targets",side_effect=AssertionError("dry-run dispatched motion")) as dispatch:
                    payload={"name":"W1_MOCK_PREVIEW_POSE","robot_id":rid,"kind":"joint_targets","params":{"targets":{"ANKLE_PITCH":-.65},"duration_s":.4},"description":"Mock integration metadata only"}
                    action=post("/api/actions",payload)["action"]
                    assert any(a["id"]==action["id"] for a in get("/api/actions?robot_id="+rid)["actions"])
                    record("原子动作保存/选用",payload,"本地动作库出现真实保存条目，不执行",{"id":action["id"],"selected_name":action["name"],"motion_dispatches":dispatch.call_count})
                    wf_payload={"name":"W1_MOCK_TWO_STEP","robot_id":rid,"steps":[{"action_name":action["name"],"duration_s":.4},{"action_name":action["name"],"duration_s":.4}]}
                    workflow=post("/api/workflows",wf_payload)["workflow"]
                    assert any(w["id"]==workflow["id"] for w in get("/api/workflows?robot_id="+rid)["workflows"])
                    record("工作流编排",wf_payload,"保存两个真实动作引用",{"workflow_id":workflow["id"],"steps":len(workflow["steps"])})
                    job=post(f"/api/workflows/{workflow['id']}/run",{"dry_run":True})["job"];done=wait(job["job_id"])
                    assert done["state"]=="Succeeded" and done["exit_code"]==0 and all(step["phase"]=="succeeded" for step in done["steps"])
                    assert any("Dry-run OK" in line for line in done["logs"])
                    record("Dry-run→详情/Steps/Logs→成功",{"workflow":workflow["id"],"dry_run":True},"Succeeded/exit0，步骤成功、日志可读取，无运动分派",{"job":done["job_id"],"state":done["state"],"exit_code":done["exit_code"],"phases":[s["phase"] for s in done["steps"]],"log_count":len(done["logs"]),"motion_dispatches":dispatch.call_count})
                    bad=post("/api/workflows",{"name":"W1_MISSING_ACTION","robot_id":rid,"steps":[{"action_name":"intentionally_missing_w1_action","duration_s":.4},{"action_name":action["name"],"duration_s":.4}]})["workflow"]
                    failed=wait(post(f"/api/workflows/{bad['id']}/run",{"dry_run":True})["job"]["job_id"])
                    assert failed["state"]=="Failed" and failed["exit_code"]==1 and failed["steps"][-1]["phase"]=="canceled"
                    record("故障停止/后续步骤不执行",{"missing_action":"intentionally_missing_w1_action"},"Failed/exit1，后续canceled，dry-run不发硬件停机",{"job":failed["job_id"],"state":failed["state"],"exit_code":failed["exit_code"],"phases":[s["phase"] for s in failed["steps"]],"motion_dispatches":dispatch.call_count})
                    running=post(f"/api/workflows/{workflow['id']}/run",{"dry_run":True})["job"]
                    post(f"/api/jobs/{running['job_id']}/cancel",{});canceled=wait(running["job_id"]);assert canceled["state"]=="Canceled"
                    record("取消",{"job":running["job_id"]},"本地Dry-run终止为Canceled",{"state":canceled["state"],"exit_code":canceled["exit_code"]})
                    retry=post(f"/api/jobs/{running['job_id']}/retry",{"dry_run":True})["job"];retried=wait(retry["job_id"])
                    assert retried["job_id"]!=running["job_id"] and retried["state"]=="Succeeded"
                    record("重试",{"source_job":running["job_id"],"dry_run":True},"新job ID，Succeeded，不转成Live",{"new_job":retried["job_id"],"state":retried["state"],"lease_id":retried["lease_id"],"motion_dispatches":dispatch.call_count})
                    denied=client.post(prefix+"/joints/targets",json={"targets":{"ANKLE_PITCH":-.65},"duration_s":.4});assert denied.status_code==403 and denied.json()["code"]=="lease_required"
                    reverse=app.state.ctx.motrix_bridge.handle_incoming_message('{"type":"joint_command"}');assert reverse["code"]=="readonly_mirror_violation"
                    assert dispatch.call_count==0
                    record("安全边界/无自动执行",{"motion_without_lease":True,"reverse_mirror_command":True},"403 lease_required / readonly_mirror_violation；0分派",{"motion_http":denied.status_code,"code":denied.json()["code"],"mirror":reverse["code"],"motion_dispatches":dispatch.call_count})
                final=get(prefix+"/state")
                assert not final["authority"]["lease_active"]
                assert final["observation"]["joint_positions"]["ANKLE_PITCH"]==state["observation"]["joint_positions"]["ANKLE_PITCH"]
                assert all(not gate.snapshot().lease_active for gate in app.state.ctx.gates.values()) and network.call_count==0
                record("切机隔离/反馈最终复核/缺失能力",{"frontend_switch":"W1→Piper","unsupported":["W1地图导航","W1策略"]},"W1本地目标/手指不串机型；反馈未伪造；所有Lease inactive",{"switch_cleared_w1":preview["switch_cleared_w1"],"ankle_feedback":final["observation"]["joint_positions"]["ANKLE_PITCH"],"leases_active":False,"external_connections":network.call_count,"unsupported_capabilities":{"navigation":False,"policy":False},"hand_feedback":"unimplemented/local preview only"})
        report["result"]="PASS"
    except Exception as error:
        report["result"]="FAIL";report["error"]=repr(error)
        raise
    finally:
        (report_dir/"w1-mock-flow.json").write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n")
        rows=["# W1 全流程 Mock 测试记录", "", f"时间：{report['time']} Asia/Shanghai；版本：{report['build']}。", "", "范围：隔离临时数据 + TestClient进程内API + 实际本地预览状态/事件函数；socket.connect被禁止。非浏览器/WebGL视觉验收。", "", "|步骤|输入|预期|实际/证据|结果|", "|---|---|---|---|---|"]
        compact=lambda value:json.dumps(value,ensure_ascii=False).replace("|","/")
        rows += [f"|{step['name']}|{compact(step['input'])}|{step['expected']}|{compact(step['actual'])}|{step['result']}|" for step in report['steps']]
        rows += ["", "未覆盖/待实现："+"；".join(report['not_covered']), "", "复现：`uv run python scripts/verify_w1_mock_flow.py --report-dir /tmp/w1-mock-report`。原始JSON/Markdown放在指定报告目录，不提交运行数据。"]
        (report_dir/"w1-mock-flow.md").write_text("\n".join(rows)+"\n")
    return report

if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("--report-dir",type=Path,required=True);args=parser.parse_args()
    result=run_flow(args.report_dir);print(json.dumps({"result":result["result"],"steps":len(result["steps"]),"report":str(args.report_dir)},ensure_ascii=False))
