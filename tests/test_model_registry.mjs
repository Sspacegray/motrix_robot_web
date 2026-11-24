import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
const source=await readFile(new URL('../frontend/js/model-registry.js',import.meta.url),'utf8');
const {modelForDescriptor}=await import(`data:text/javascript;base64,${Buffer.from(source).toString('base64')}`);
test('W1 model cannot be substituted for another robot identity',()=>{
 assert.ok(modelForDescriptor({id:'dex-w1-mock',robot_type:'dexforce_w1'}));
 for (const descriptor of [{id:'r1pro-mock',robot_type:'r1pro'},{id:'piper-dual-mock',robot_type:'piper_dual'},{id:'piper-dual-mock',robot_type:'dexforce_w1'},{id:'dex-w1-mock',robot_type:'piper_dual'}]) assert.equal(modelForDescriptor(descriptor),null);
});
test('visual loader has no control API and preserves GLTF scene materials',async()=>{
 const text=await readFile(new URL('../frontend/js/robot-model-viewer.js',import.meta.url),'utf8');
 assert.doesNotMatch(text,/import.*api\.js|fetch\(.*\/api\//);
 assert.match(text,/done\(gltf\.scene\)/);
 assert.match(text,/loader\.parseCollision = false/);
 assert.match(text,/await Promise\.all\(pending\)/);
 assert.match(text,/ownGeneration !== generation/);
 assert.match(text,/url\.origin !== window\.location\.origin/);
});
