// Capability/entry regression without browser, network or robot writes.
import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
const source=await readFile(new URL('../frontend/js/navigation-state.js',import.meta.url),'utf8');
const {FEATURE_LABELS,featureAvailability,applyFeatureNavigation}=await import(`data:text/javascript;base64,${Buffer.from(source).toString('base64')}`);
const W1={navigation:false,cameras:true,head_depth:false,yolo_foundation_pose:false,aruco:false,policy_inference:false,joints:true,atomic_actions:true,workflows:true};
const R1={...W1,navigation:true,head_depth:true,yolo_foundation_pose:true,aruco:true,policy_inference:true};
function button(name){return {name,hidden:true,style:{display:'none'},dataset:{},attrs:{'aria-hidden':'true'},getAttribute(key){return key==='data-tab'?name:this.attrs[key];},setAttribute(key,val){this.attrs[key]=val;},removeAttribute(key){delete this.attrs[key];}};}
test('all seven feature entries remain visible for W1; unavailable Navigation/Policy disabled with reason',()=>{
 const buttons=Object.keys(FEATURE_LABELS).map(button),state=featureAvailability(W1);
 applyFeatureNavigation(buttons,state);
 assert.equal(buttons.length,7);
 for(const entry of buttons){assert.equal(entry.hidden,false);assert.equal(entry.style.display,'inline-flex');assert.equal(entry.attrs['aria-hidden'],undefined);}
 assert.equal(state.perception.enabled,true,'camera-only W1 perception page must remain reachable');
 for(const key of ['navigation','policy']){const entry=buttons.find(b=>b.name===key);assert.equal(entry.disabled,true);assert.ok(entry.title.includes('当前机型'));}
 assert.equal(buttons.find(b=>b.name==='actions').disabled,false);
});
test('switching to R1 restores Navigation/Perception/Policies without recreating entries',()=>{
 const buttons=Object.keys(FEATURE_LABELS).map(button);
 applyFeatureNavigation(buttons,featureAvailability(W1));
 applyFeatureNavigation(buttons,featureAvailability(R1));
 for(const entry of buttons)assert.equal(entry.disabled,false,entry.name);
});
test('Piper retains camera access and policies; navigation explains missing chassis capability',()=>{
 const state=featureAvailability({...W1,policy_inference:true});
 assert.equal(state.perception.enabled,true);assert.equal(state.policy.enabled,true);assert.equal(state.navigation.enabled,false);assert.match(state.navigation.reason,/R1 Pro/);
});
test('catalog, workflows and jobs stay separate real subpages with direct links and API loaders',async()=>{
 const html=await readFile(new URL('../frontend/index.html',import.meta.url),'utf8');
 const app=await readFile(new URL('../frontend/js/app.js',import.meta.url),'utf8');
 const actions=await readFile(new URL('../frontend/js/tabs/actions.js',import.meta.url),'utf8');
 for(const label of ['地图导航','视觉感知','策略评测','动作与工作流'])assert.ok(html.includes(label));
 assert.doesNotMatch(app,/button\.hidden = !enabled|navTabBtn\.style\.display = "none"/);
 for(const [sub,label] of [['actions','原子动作'],['workflows','工作流编排'],['jobs','执行任务记录']])assert.match(actions,new RegExp(`data-sub="${sub}"[^>]*>[^<]*${label}`));
 assert.match(actions,/get\("section"\)/);assert.match(actions,/api\.listActions\(\)/);assert.match(actions,/api\.listWorkflows\(\)/);assert.match(actions,/api\.listJobs\(\)/);
 assert.match(actions,/lastRobotId !== api\.currentRobotId/);
});
