// Native EventTarget/Event listener tests with a minimal DOM bubbling harness.
// This exercises the production handlers, not source regexes; no WebGL/browser claim.
import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
const source=await readFile(new URL('../frontend/js/map-interactions.js',import.meta.url),'utf8');
const {bindMapInteractions}=await import(`data:text/javascript;base64,${Buffer.from(source).toString('base64')}`);
class Node extends EventTarget {
 constructor(parent=null){super();this.parent=parent;}
 dispatchEvent(event){
  const path=[];for(let node=this;node;node=node.parent)path.push(node);
  Object.defineProperty(event,'target',{value:this,configurable:true});
  Object.defineProperty(event,'composedPath',{value:()=>path,configurable:true});
  for(const node of (event.bubbles?path:[this])) EventTarget.prototype.dispatchEvent.call(node,event);
  return !event.defaultPrevented;
 }
}
function event(target,type,options={}){
 const value=new Event(type,{bubbles:true,cancelable:true});
 for(const [name,field] of Object.entries({button:0,buttons:1,clientX:1,clientY:2,...options}))Object.defineProperty(value,name,{value:field});
 target.dispatchEvent(value);return value;
}
function setup(){
 const root=new Node(),canvas=new Node(root),cloud=new Node(root);
 let view='occupancy2d',supported=true,dragging=false;
 const values={x:1,y:2,yaw:3},marker={x:1,y:2,yaw:3};
 const calls={goal:0,pan:0,wheel2d:0,camera:0};
 const edit=e=>{values.x=e.clientX;values.y=e.clientY;values.yaw=0;Object.assign(marker,values);calls.goal++;};
 const guard=bindMapInteractions({canvas,cloudCanvas:cloud,windowTarget:root,
  isEnabled:()=>view==='occupancy2d'&&supported,
  onDown:e=>{dragging=true;edit(e);},onMove:e=>{if(dragging)edit(e);},onUp:()=>{dragging=false;},onWheel:()=>calls.wheel2d++,
 });
 // A 3D camera listener remains independent and operational.
 cloud.addEventListener('mousemove',()=>calls.camera++);cloud.addEventListener('wheel',()=>calls.camera++);
 return {root,canvas,cloud,guard,values,marker,calls,switchView(next){guard.reset();view=next;},unsupported(){supported=false;},snapshot:()=>JSON.stringify({values,marker})};
}

test('3D orbit, wheel and bubbled post-drag click preserve goal inputs AND goal marker',()=>{
 const s=setup();event(s.canvas,'mousedown',{clientX:4,clientY:5});event(s.canvas,'mousemove',{clientX:6,clientY:7});
 // Switch without a mouseup: this was the leaked drag state.
 s.switchView('cloud3d');const before=s.snapshot(),edits=s.calls.goal;
 let requests=0;const old=globalThis.fetch;globalThis.fetch=()=>{requests++;throw Error('read-only camera sent a request');};
 try{
  event(s.cloud,'mousedown');event(s.cloud,'mousemove',{clientX:13.77,clientY:.11});event(s.cloud,'wheel');event(s.cloud,'mouseup');event(s.cloud,'click');
  // Even a delayed 2D event cannot edit a hidden 2D canvas while in 3D.
  event(s.canvas,'mousemove',{clientX:99,clientY:99});event(s.canvas,'wheel');event(s.canvas,'mousedown');
  assert.equal(s.snapshot(),before);assert.equal(s.calls.goal,edits);assert.equal(s.calls.wheel2d,0);
  assert.equal(s.calls.camera,2);assert.equal(requests,0);
 }finally{globalThis.fetch=old;s.guard.destroy();}
});

test('3D input itself clears unfinished 2D mousedown even before a view reset',()=>{
 const s=setup();event(s.canvas,'mousedown',{clientX:4,clientY:5});const before=s.snapshot();
 event(s.cloud,'pointerdown');event(s.cloud,'mousemove',{clientX:99,clientY:99});event(s.cloud,'click');
 event(s.canvas,'mousemove',{clientX:99,clientY:99});
 assert.equal(s.snapshot(),before);s.guard.destroy();
});

test('switching 2D→3D→2D requires a fresh 2D press; no residual drag or click',()=>{
 const s=setup();event(s.canvas,'mousedown');s.switchView('cloud3d');event(s.cloud,'mousemove');event(s.cloud,'click');s.switchView('occupancy2d');
 const before=s.snapshot();event(s.canvas,'mousemove',{clientX:99,clientY:99});event(s.canvas,'click');assert.equal(s.snapshot(),before);
 event(s.canvas,'mousedown',{clientX:8,clientY:9});event(s.canvas,'mousemove',{clientX:10,clientY:11});event(s.canvas,'mouseup');
 assert.deepEqual(s.values,{x:10,y:11,yaw:0});assert.deepEqual(s.marker,s.values);s.guard.destroy();
});

test('lost mouseup, unsupported robot and disposed controller cannot keep editing',()=>{
 const s=setup();event(s.canvas,'mousedown');const before=s.snapshot();event(s.canvas,'mousemove',{buttons:0,clientX:20});event(s.canvas,'mousemove',{clientX:21});assert.equal(s.snapshot(),before);
 s.unsupported();event(s.canvas,'mousedown',{clientX:22});event(s.canvas,'wheel');assert.equal(s.snapshot(),before);assert.equal(s.calls.wheel2d,0);
 s.guard.destroy();event(s.canvas,'mousedown',{clientX:23});event(s.canvas,'mousemove',{clientX:24});assert.equal(s.snapshot(),before);
});


test('navigation wires the boundary and resets it when switching views',async()=>{
 const navigation=await readFile(new URL('../frontend/js/tabs/navigation.js',import.meta.url),'utf8');
 assert.match(navigation,/const mapInteractions = bindMapInteractions/);
 assert.match(navigation,/function setMapView\(next\) \{\s*mapInteractions.reset\(\)/);
 assert.match(navigation,/isEnabled:\(\) => mapSupported && mapView === "occupancy2d" && !canvas.hidden/);
 assert.doesNotMatch(navigation,/window.addEventListener\("mousemove"/);
 const css=await readFile(new URL('../frontend/css/style.css',import.meta.url),'utf8');
 assert.match(css,/#navMapCanvas\[hidden\][^}]*display:none!important/);
});

test('point-cloud HUD is separate from the 2D description and legends are view-specific',async()=>{
 const navigation=await readFile(new URL('../frontend/js/tabs/navigation.js',import.meta.url),'utf8');
 assert.match(navigation,/hud: container.querySelector\("#navCloudHud"\)/);
 assert.match(navigation,/querySelector\("#navCloudHud"\).hidden = !threeD/);
 assert.match(navigation,/querySelector\("#navMapLegend"\).innerHTML = threeD/);
 assert.match(navigation,/querySelector\("#mapCloudNote"\).textContent = threeD/);
});
