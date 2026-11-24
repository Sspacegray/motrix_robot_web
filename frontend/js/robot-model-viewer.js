// Local, read-only visual loading. No robot API or command transport imports.
import * as THREE from "three";
import URDFLoader from "urdf-loader";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";
import { modelForDescriptor } from "./model-registry.js";
import { applyModelJoints } from "./pose-preview.js";

export function disposeModel(root) {
  root?.traverse(node => {
    node.geometry?.dispose();
    for (const material of (Array.isArray(node.material) ? node.material : [node.material])) {
      if (!material) continue;
      for (const value of Object.values(material)) if (value?.isTexture) value.dispose();
      material.dispose();
    }
  });
}

export async function loadLocalRobotModel(config, onProgress = () => {}) {
  const manager = new THREE.LoadingManager();
  const base = new URL(config.url, window.location.href);
  const allowedPrefix = new URL("./", base).pathname;
  manager.setURLModifier(raw => {
    if (raw.startsWith("blob:")) return raw; // GLB embedded image, never a remote texture.
    const url = new URL(raw, window.location.href);
    if (url.origin !== window.location.origin || !url.pathname.startsWith(allowedPrefix)) throw new Error("Blocked nonlocal model resource");
    return url.href;
  });
  const loader = new URDFLoader(manager);
  loader.parseVisual = true;
  loader.parseCollision = false;
  const pending = [];
  let loaded = 0;
  loader.loadMeshCb = (path, _manager, done) => {
    const stl = new URL(path, window.location.href).pathname.toLowerCase().endsWith(".stl");
    const promise = (stl ? new STLLoader(manager).loadAsync(path) : new GLTFLoader(manager).loadAsync(path)).then(result => {
      if (stl) done(new THREE.Mesh(result)); // Original hand URDF provides material and mesh scale.
      else { const gltf = result; done(gltf.scene); } // Preserve original GLB PBR maps.
      loaded++;
      onProgress(loaded, config.visualCount);
    }).catch(error => { done(null, error); throw error; });
    // Handle now, await all below; avoid unhandled rejection while URDF is parsing.
    promise.catch(() => {});
    pending.push(promise);
  };
  let robot;
  try {
    robot = await loader.loadAsync(base.href);
    await Promise.all(pending);
    if (loaded !== config.visualCount) throw new Error(`Expected ${config.visualCount} visuals, got ${loaded}`);
    const stats = {visuals:loaded, meshes:0, textureMaps:0};
    robot.traverse(node => {
      if (!node.isMesh) return;
      stats.meshes++;
      for (const mat of (Array.isArray(node.material) ? node.material : [node.material])) if (mat?.map?.image?.width > 0 && mat.map.image.height > 0) stats.textureMaps++;
    });
    return {robot, stats};
  } catch (error) {
    await Promise.allSettled(pending);
    disposeModel(robot);
    throw error;
  }
}

export function createRobotModelViewer({scene, camera, controls, hud}) {
  const root = new THREE.Group();
  root.rotation.x = -Math.PI / 2; // URDF Z-up -> Three Y-up.
  scene.add(root);
  let generation = 0, currentRobot = null, descriptor = null, positions = {}, handModels = {}, handTargets = {};
  function syncHands() {
    for(const [side,targets] of Object.entries(handTargets)) for(const [name,value] of Object.entries(targets)) {
      if(Number.isFinite(value) && handModels[side]?.joints[name]) handModels[side].setJointValue(name,value);
    }
  }
  function syncPose() { applyModelJoints(currentRobot,descriptor,positions); }

  return {
    async select(nextDescriptor) {
      const ownGeneration = ++generation;
      if (currentRobot) {root.remove(currentRobot); disposeModel(currentRobot); currentRobot = null;}
      handModels = {};
      descriptor = nextDescriptor;
      hud.dataset.modelState = "loading";
      hud.dataset.textureMaps = "0";
      const config = modelForDescriptor(descriptor);
      if (!config) {
        hud.dataset.modelState = "unavailable";
        hud.textContent = `${descriptor?.name || "当前机型"}：缺少该机型真实模型资产，未显示替代骨架或 W1 模型。只读视图。`;
        return;
      }
      hud.textContent = "W1 本地 URDF / GLB 加载中… 0 / 21（只读）";
      try {
        const {robot, stats} = await loadLocalRobotModel(config, (loaded,total) => {
          if (ownGeneration === generation) hud.textContent = `W1 本地网格加载 ${loaded} / ${total} · 只读`;
        });
        if (ownGeneration !== generation) {disposeModel(robot); return;}
        currentRobot = robot;
        root.add(robot);
        syncPose();
        const hands = await Promise.allSettled((config.hands || []).map(hand => loadLocalRobotModel(hand)));
        if (ownGeneration !== generation) { for(const result of hands) if(result.status === "fulfilled") disposeModel(result.value.robot); return; }
        let handCount = 0;
        hands.forEach((result,index) => {
          const hand = config.hands[index];
          if(result.status === "fulfilled" && robot.links[hand.anchor]) {
            robot.links[hand.anchor].add(result.value.robot); handModels[hand.side] = result.value.robot; handCount++;
          } else if(result.status === "fulfilled") disposeModel(result.value.robot);
        });
        syncHands();
        root.updateMatrixWorld(true);
        const bounds = new THREE.Box3().setFromObject(root);
        if (!bounds.isEmpty()) {
          const center = bounds.getCenter(new THREE.Vector3());
          const size = bounds.getSize(new THREE.Vector3());
          const radius = Math.max(size.x,size.y,size.z,0.5);
          controls.target.copy(center);
          camera.position.copy(center).add(new THREE.Vector3(radius*1.2,radius*.65,radius*1.4));
          camera.near = 0.01; camera.far = radius*30; camera.updateProjectionMatrix(); controls.update();
        }
        hud.textContent = `W1 原始 URDF · ${stats.visuals}/21 GLB · ${stats.meshes} 网格 · ${stats.textureMaps} 材质贴图已解码 · BrainCo Revo1 手 ${handCount}/2（STL原色/无纹理） · 本地预览 / 反馈分离`;
        if (stats.textureMaps < config.embeddedImages) hud.textContent += ` · 警告：预期 ${config.embeddedImages} 张原始贴图，尚未完整解码` ;
        hud.dataset.modelState = stats.textureMaps >= config.embeddedImages ? "loaded" : "partial";
        hud.dataset.textureMaps = String(stats.textureMaps);
        hud.dataset.handsLoaded = String(handCount);
        if(handCount !== 2) { hud.textContent += " · 手部资产加载不完整（未造假替代）"; hud.dataset.modelState = "partial"; }
      } catch (error) {
        if (ownGeneration !== generation) return;
        hud.textContent = "W1 模型加载失败（未用骨架冒充）：" + error.message;
        hud.dataset.modelState = "error";
      }
    },
    updatePose(nextPositions) { positions = {...(nextPositions || {})}; syncPose(); },
    updateHandTargets(targets) { handTargets = targets || {}; syncHands(); },
    destroy() {generation++; if (currentRobot) disposeModel(currentRobot); scene.remove(root);}
  };
}
