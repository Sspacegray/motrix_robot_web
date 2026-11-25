// Actual drawer controller lifecycle in a minimal DOM double, not browser/visual testing.
import test from 'node:test';
import assert from 'node:assert/strict';
import vm from 'node:vm';
import {readFile} from 'node:fs/promises';
const source = await readFile(new URL('../frontend/js/drawer.js',import.meta.url),'utf8');
class Element {
  constructor() {
    const classes = new Set();
    this.classList = {add:v=>classes.add(v),remove:v=>classes.delete(v),contains:v=>classes.has(v)};
    this.children = new Map(); this.events = {}; this.style = {};
  }
  setAttribute() {}
  addEventListener(type,fn) {this.events[type]=fn;}
  appendChild() {}
  querySelector(selector) {if (!this.children.has(selector)) this.children.set(selector,new Element()); return this.children.get(selector);}
  focus() {}
}
test('initial page has no open drawer; explicit selection opens overlay; close restores unreserved page', async () => {
  let reads=0;
  const body=new Element();
  const document={body,createElement:()=>new Element(),activeElement:new Element()};
  const job={job_id:'local-job',state:'Succeeded',workflow_id:'wf',steps:[],logs:[],flow_name:'Mock',run_id:'r'};
  const context={document,window:{addEventListener(){}},api:{getJob:async()=>{reads++; return {job};}},escapeHtml:String,formatLocalTime:()=>'',setInterval:()=>{throw Error('terminal task must not poll');},clearInterval(){},console};
  vm.createContext(context);
  vm.runInContext(source.replace(/^import .*;$/gm,'').replace('export const drawer =','globalThis.drawer ='),context);
  const drawer=context.drawer;
  assert.equal(reads,0);
  assert.equal(drawer.currentJobId,null);
  assert.equal(drawer.panel.classList.contains('open'),false);
  assert.equal(drawer.backdrop.classList.contains('open'),false);
  assert.equal(drawer.panel.inert,true);
  await drawer.open('local-job');
  assert.equal(reads,1);
  assert.equal(drawer.panel.classList.contains('open'),true);
  assert.equal(body.classList.contains('execution-open'),true);
  drawer.close();
  assert.equal(drawer.panel.classList.contains('open'),false);
  assert.equal(drawer.backdrop.classList.contains('open'),false);
  assert.equal(body.classList.contains('execution-open'),false);
  assert.equal(drawer.currentJobId,null);
  assert.equal(drawer.panel.inert,true);
});
test('main layout has exactly one navigation entry per feature, no extra seven-tab band', async () => {
  const html=await readFile(new URL('../frontend/index.html',import.meta.url),'utf8');
  for (const tab of ['overview','actions','joints','navigation','perception','policy','diagnostics']) {
    assert.equal((html.match(new RegExp(`data-tab="${tab}"`,'g'))||[]).length,1,tab);
  }
  assert.doesNotMatch(html,/class="sub-nav-bar"/);
  const css=await readFile(new URL('../frontend/css/style.css',import.meta.url),'utf8');
  assert.match(css,/\.drawer-panel[^{]*\{[^}]*position: fixed;/);
  assert.doesNotMatch(css,/margin-right:\s*760px|grid-template-columns:\s*[^;]*760px/);
});
