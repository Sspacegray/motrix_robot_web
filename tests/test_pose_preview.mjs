import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
const load=async file=>{const text=await readFile(new URL(file,import.meta.url),'utf8');return import(`data:text/javascript;base64,${Buffer.from(text).toString('base64')}`);};
const {PosePreview,bindLocalPreviewInput,applyModelJoints}=await load('../frontend/js/pose-preview.js');
const {W1_HANDS,modelForDescriptor}=await load('../frontend/js/model-registry.js');
const w1={id:'dex-w1-mock',robot_type:'dexforce_w1',joints:[{name:'ANKLE_PITCH',urdf_joint:'ANKLE_PITCH',lower:-1.57,upper:1.57,default:0}]};
function create(){const state=new PosePreview();state.selectRobot(w1,{joint_positions:{ANKLE_PITCH:0}},W1_HANDS);return state;}
test('slider input immediately updates URDF target preview, never feedback or API',()=>{
 const state=create(),seen=[];let writes=0;const input={addEventListener(type,fn){this[type]=fn;}};
 const model={joints:{ANKLE_PITCH:{}},setJointValue(name,value){seen.push([name,value]);}};
 const original=globalThis.fetch;globalThis.fetch=()=>{writes++;throw Error('No requests allowed');};
 try{
 bindLocalPreviewInput(input,value=>state.setTarget('ANKLE_PITCH',value),()=>applyModelJoints(model,w1,state.visiblePose()));
 input.input({target:{value:'-0.65'}});
 assert.deepEqual(seen,[['ANKLE_PITCH',-.65]]);assert.equal(state.feedback.ANKLE_PITCH,0);assert.equal(writes,0);
 state.observe(w1.id,{ANKLE_PITCH:0});assert.equal(state.visiblePose().ANKLE_PITCH,-.65);
 }finally{globalThis.fetch=original;}
});
test('feedback visual mode stays independent and new target input returns to preview',()=>{
 const state=create();state.setTarget('ANKLE_PITCH',-.65);state.mode='feedback';assert.equal(state.visiblePose().ANKLE_PITCH,0);
 state.setTarget('ANKLE_PITCH',-.4);assert.equal(state.mode,'target');assert.equal(state.visiblePose().ANKLE_PITCH,-.4);assert.equal(state.feedback.ANKLE_PITCH,0);
});
test('real W1 hands expose six source joints each; local finger targets never enter commands',()=>{
 const state=create();assert.equal(W1_HANDS.length,2);assert.ok(W1_HANDS.every(hand=>hand.joints.length===6));
 assert.equal(state.setHandTarget('left','T_MCP',.6),true);assert.equal(state.handTargets.left.T_MCP,.6);assert.deepEqual(state.commandTargets(),{ANKLE_PITCH:0});assert.deepEqual(state.feedback,{ANKLE_PITCH:0});
 assert.equal(state.setHandTarget('left','invented_joint',1),false);
});
test('switch robot clears previous body/hand targets and discards old feedback',()=>{
 const state=create();state.setTarget('ANKLE_PITCH',-.65);state.setHandTarget('right','T_MCP',.5);
 const piper={id:'piper-dual-mock',robot_type:'piper_dual',joints:[{name:'left_j1',lower:-2,upper:2,default:0}]};
 state.selectRobot(piper,{joint_positions:{left_j1:.1}},modelForDescriptor(piper)?.hands||[]);state.observe(w1.id,{ANKLE_PITCH:1});
 assert.deepEqual(state.targets,{left_j1:.1});assert.deepEqual(state.handTargets,{});assert.deepEqual(state.feedback,{left_j1:.1});
});
test('non-finite input rejected and limits enforced locally',()=>{
 const state=create();assert.equal(state.setTarget('ANKLE_PITCH',NaN),false);state.setTarget('ANKLE_PITCH',99);assert.equal(state.targets.ANKLE_PITCH,1.57);assert.equal(state.feedback.ANKLE_PITCH,0);
});
