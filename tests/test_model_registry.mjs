import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
const source=await readFile(new URL('../frontend/js/model-registry.js',import.meta.url),'utf8');
const {modelForDescriptor}=await import(`data:text/javascript;base64,${Buffer.from(source).toString('base64')}`);

test('each of the 3 robots maps strictly to its own authentic URDF and never cross-substitutes',()=>{
 const w1 = modelForDescriptor({id:'dex-w1-mock',robot_type:'dexforce_w1'});
 const r1 = modelForDescriptor({id:'r1pro-mock',robot_type:'r1pro'});
 const piper = modelForDescriptor({id:'piper-dual-mock',robot_type:'piper_dual'});
 assert.ok(w1 && r1 && piper);
 assert.equal(w1.url, '/assets/urdf/dexforce_w1_v030/robot.urdf');
 assert.equal(r1.url, '/assets/urdf/galaxea_r1pro/robot.urdf');
 assert.equal(piper.url, '/assets/urdf/piper_dual_d405/robot.urdf');
 assert.equal(new Set([w1.url, r1.url, piper.url]).size, 3);
 for (const descriptor of [
   {id:'piper-dual-mock',robot_type:'dexforce_w1'},
   {id:'dex-w1-mock',robot_type:'piper_dual'},
   {id:'r1pro-mock',robot_type:'dexforce_w1'},
   {id:'dex-w1-mock',robot_type:'r1pro'},
   {id:'r1pro-mock',robot_type:'piper_dual'},
   {id:'unknown-robot',robot_type:'r1pro'},
   null,
 ]) {
   assert.equal(modelForDescriptor(descriptor), null);
 }
});

test('custom imported robots resolve dynamically from descriptor.model_profile without code edits', () => {
  const custom = modelForDescriptor({
    id: 'cobot-arm6-demo',
    name: 'Arm6 Cobot',
    robot_type: 'motphys_arm6',
    model_profile: {
      label: 'Arm6 URDF',
      format_label: 'STL',
      url: '/custom-robots/cobot-arm6-demo/urdf/robot.urdf',
      visual_count: 9,
      embedded_images: 0,
      trail_links: [{ name: 'ee_link', color: '#00f0ff', light_color: '#0284c7', label: '末端' }],
    },
  });
  assert.ok(custom);
  assert.equal(custom.url, '/custom-robots/cobot-arm6-demo/urdf/robot.urdf');
  assert.equal(custom.visualCount, 9);
  assert.equal(custom.trailLinks[0].color, 0x00f0ff);
  assert.equal(custom.trailLinks[0].lightColor, 0x0284c7);
  assert.equal(modelForDescriptor({
    id: 'evil-bot',
    robot_type: 'evil',
    model_profile: { url: 'https://evil.example/robot.urdf' },
  }), null);
});

test('visual loader has no control API, supports OBJ/STL/GLB, and preserves GLTF scene materials',async()=>{
 const text=await readFile(new URL('../frontend/js/robot-model-viewer.js',import.meta.url),'utf8');
 assert.doesNotMatch(text,/import.*api\.js|fetch\(.*\/api\//);
 assert.match(text,/OBJLoader/);
 assert.match(text,/done\(gltf\.scene\)/);
 assert.match(text,/loader\.parseCollision = false/);
 assert.match(text,/await Promise\.all\(pending\)/);
 assert.match(text,/ownGeneration !== generation/);
 assert.match(text,/url\.origin !== window\.location\.origin/);
});
