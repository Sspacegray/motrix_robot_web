// Local, read-only visual loading for W1 (GLB), R1 Pro (textured OBJ), and Piper Dual (STL/OBJ + D405 texture).
// Includes Dexforce W1 style translucent purple Ghost overlay + glowing neon 3D trajectory trails + IK target marker.
// Strictly read-only: NO robot API or command transport imports.
import * as THREE from "three";
import URDFLoader from "urdf-loader";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import { OBJLoader } from "three/addons/loaders/OBJLoader.js";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";
import { modelForDescriptor } from "./model-registry.js";
import { applyModelJoints } from "./pose-preview.js";

const PIPER_PBR_OVERRIDES = Object.freeze({
  piper_d405_silver: { roughness: 0.28, metalness: 0.85 },
  piper_visual_c8c8c8ff: { roughness: 0.26, metalness: 0.88 },
  piper_mount_aluminium: { roughness: 0.36, metalness: 0.68 },
  piper_mount_black: { roughness: 0.55, metalness: 0.18 },
  piper_d405_front_glass: { roughness: 0.12, metalness: 0.1 },
  piper_d405_label: { roughness: 0.78, metalness: 0.0 },
  piper_gripper_pad_black: { roughness: 0.85, metalness: 0.0 },
  piper_visual_da1f1fff: { roughness: 0.34, metalness: 0.08 },
  piper_visual_c72b2bff: { roughness: 0.34, metalness: 0.08 },
});

const KEYFRAME_COUNT = 7;
const UP_VEC = new THREE.Vector3(0, 1, 0);

function applyGhostTheme(fillMat, edgeMat, isLight) {
  if (isLight) {
    fillMat.color.setHex(0x7c3aed);
    fillMat.emissive.setHex(0x6d28d9);
    fillMat.emissiveIntensity = 0.85;
    fillMat.opacity = 0.50;
    if (edgeMat) {
      edgeMat.color.setHex(0x4c1d95);
      edgeMat.opacity = 0.62;
    }
  } else {
    fillMat.color.setHex(0x38bdf8);
    fillMat.emissive.setHex(0x0ea5e9);
    fillMat.emissiveIntensity = 1.35;
    fillMat.opacity = 0.56;
    if (edgeMat) {
      edgeMat.color.setHex(0xbae6fd);
      edgeMat.opacity = 0.78;
    }
  }
}

function makeGhostMaterials(isLight = false) {
  const fill = new THREE.MeshStandardMaterial({
    color: 0x38bdf8,
    transparent: true,
    opacity: 0.56,
    depthWrite: false,
    polygonOffset: true,
    polygonOffsetFactor: -1,
    polygonOffsetUnits: -1,
    emissive: 0x0ea5e9,
    emissiveIntensity: 1.35,
    roughness: 0.28,
    metalness: 0.10,
  });
  const edge = new THREE.LineBasicMaterial({
    color: 0xbae6fd,
    transparent: true,
    opacity: 0.78,
    depthWrite: false,
    toneMapped: false,
  });
  applyGhostTheme(fill, edge, isLight);
  return { fill, edge };
}

function upgradeMaterialPbr(mat, profile) {
  if (!mat || mat.isMeshStandardMaterial || mat.isMeshPhysicalMaterial) return mat;
  let roughness = 0.44;
  let metalness = 0.14;
  const name = (mat.name || "").toLowerCase();
  if (profile === "piper_dual") {
    const ov = PIPER_PBR_OVERRIDES[mat.name];
    if (ov) {
      roughness = ov.roughness;
      metalness = ov.metalness;
    } else {
      roughness = 0.35;
      metalness = 0.12;
    }
  } else if (profile === "r1pro") {
    if (mat.map) {
      roughness = 0.44;
      metalness = 0.16;
      mat.map.anisotropy = 4;
    } else if (name.includes("wheel") || name.includes("finger")) {
      roughness = 0.72;
      metalness = 0.05;
    } else if (name.includes("steer") || name.includes("lidar") || name.includes("camera") || name.includes("d405") || name.includes("arm_base")) {
      roughness = 0.32;
      metalness = 0.52;
    } else if (name.includes("head") || name.includes("torso") || name.includes("base_link")) {
      roughness = 0.38;
      metalness = 0.12;
    } else {
      roughness = 0.36;
      metalness = 0.25;
    }
  } else if (profile === "custom") {
    roughness = 0.38;
    metalness = 0.22;
  }
  const std = new THREE.MeshStandardMaterial({
    name: mat.name,
    color: mat.color ? mat.color.clone() : new THREE.Color(0xcbd5e1),
    map: mat.map || null,
    roughness,
    metalness,
    opacity: mat.opacity,
    transparent: mat.transparent,
    depthWrite: mat.depthWrite,
  });
  mat.dispose?.();
  return std;
}

export function disposeModel(root) {
  const seenGeo = new Set();
  const seenMat = new Set();
  const seenTex = new Set();
  root?.traverse(node => {
    if (node.geometry && !seenGeo.has(node.geometry)) {
      seenGeo.add(node.geometry);
      node.geometry.dispose();
    }
    for (const material of (Array.isArray(node.material) ? node.material : [node.material])) {
      if (!material || seenMat.has(material)) continue;
      seenMat.add(material);
      for (const value of Object.values(material)) {
        if (value?.isTexture && !seenTex.has(value)) {
          seenTex.add(value);
          value.dispose();
        }
      }
      material.dispose();
    }
  });
}

function extractObjGeometry(group) {
  let firstGeo = null;
  group.traverse(child => {
    if (!firstGeo && child.isMesh && child.geometry) firstGeo = child.geometry;
  });
  if (!firstGeo) return null;
  if (!firstGeo.attributes.normal) firstGeo.computeVertexNormals();
  return firstGeo;
}

function waitForTextureDecode(tex) {
  if (!tex) return Promise.resolve();
  return new Promise(resolve => {
    const start = performance.now();
    const check = () => {
      const img = tex.image;
      if (img && ((img.complete && img.naturalWidth > 0) || (img.width > 0 && img.height > 0))) {
        resolve();
        return;
      }
      if (performance.now() - start > 2500) {
        resolve();
        return;
      }
      setTimeout(check, 40);
    };
    check();
  });
}

export async function loadLocalRobotModel(config, onProgress = () => {}, sharedCache = null) {
  const manager = new THREE.LoadingManager();
  const base = new URL(config.url, window.location.href);
  let allowedPrefix = new URL("./", base).pathname;
  if (base.pathname.startsWith("/custom-robots/")) {
    const segs = base.pathname.split("/").filter(Boolean);
    if (segs.length >= 2) allowedPrefix = `/${segs[0]}/${segs[1]}/`;
  }
  manager.setURLModifier(raw => {
    if (raw.startsWith("blob:")) return raw; // GLB embedded image, never a remote texture.
    const url = new URL(raw, window.location.href);
    if (url.origin !== window.location.origin || !url.pathname.startsWith(allowedPrefix)) {
      throw new Error("Blocked nonlocal model resource");
    }
    return url.href;
  });

  const loader = new URDFLoader(manager);
  loader.parseVisual = true;
  loader.parseCollision = false;

  const stlLoader = new STLLoader(manager);
  const objLoader = new OBJLoader(manager);
  const gltfLoader = new GLTFLoader(manager);
  const geoCache = sharedCache?.geoCache || new Map();
  const gltfCache = sharedCache?.gltfCache || new Map();
  const isGhost = Boolean(sharedCache?.asGhost);
  const ghostMat = isGhost ? sharedCache.ghostMaterial : null;

  const pending = [];
  let loaded = 0;

  loader.loadMeshCb = (path, _manager, done) => {
    const lower = new URL(path, window.location.href).pathname.toLowerCase();
    let promise;
    if (lower.endsWith(".stl")) {
      let cached = geoCache.get(path);
      if (!cached) {
        cached = stlLoader.loadAsync(path).then(geo => {
          if (!geo.attributes.normal) geo.computeVertexNormals();
          return geo;
        });
        cached.catch(() => {});
        geoCache.set(path, cached);
      }
      promise = cached.then(geo => {
        done(new THREE.Mesh(geo.clone(), ghostMat || undefined));
        loaded++;
        onProgress(loaded, config.visualCount);
      }).catch(error => { done(null, error); throw error; });
    } else if (lower.endsWith(".obj")) {
      let cached = geoCache.get(path);
      if (!cached) {
        cached = objLoader.loadAsync(path).then(group => {
          const geo = extractObjGeometry(group);
          if (!geo) throw new Error(`No mesh geometry in OBJ: ${path}`);
          return geo;
        });
        cached.catch(() => {});
        geoCache.set(path, cached);
      }
      promise = cached.then(geo => {
        done(new THREE.Mesh(geo.clone(), ghostMat || undefined));
        loaded++;
        onProgress(loaded, config.visualCount);
      }).catch(error => { done(null, error); throw error; });
    } else {
      let cached = gltfCache.get(path);
      if (!cached) {
        cached = gltfLoader.loadAsync(path);
        cached.catch(() => {});
        gltfCache.set(path, cached);
      }
      promise = cached.then(gltf => {
        if (isGhost) {
          const cloned = gltf.scene.clone(true);
          cloned.traverse(o => { if (o.isMesh) { o.material = ghostMat; o.renderOrder = 2; } });
          done(cloned);
        } else {
          done(gltf.scene); // Preserve original GLB PBR maps untouched.
        }
        loaded++;
        onProgress(loaded, config.visualCount);
      }).catch(error => { done(null, error); throw error; });
    }
    promise.catch(() => {});
    pending.push(promise);
  };

  let robot;
  try {
    if (sharedCache?.urdfText) {
      loader.workingPath = new URL("./", base).href;
      robot = loader.parse(sharedCache.urdfText);
    } else {
      const resp = await fetch(base.href);
      if (!resp.ok) throw new Error(`URDF HTTP ${resp.status}`);
      const urdfText = await resp.text();
      if (sharedCache) sharedCache.urdfText = urdfText;
      loader.workingPath = new URL("./", base).href;
      robot = loader.parse(urdfText);
    }
    await Promise.all(pending);
    if (loaded !== config.visualCount) {
      throw new Error(`Expected ${config.visualCount} visuals, got ${loaded}`);
    }

    if (isGhost) {
      const edgeMat = sharedCache.ghostEdgeMaterial;
      const ghostMeshes = [];
      robot.traverse(node => {
        if (node.isMesh) {
          node.material = ghostMat;
          node.renderOrder = 4;
          ghostMeshes.push(node);
        }
      });
      if (edgeMat) {
        for (const mesh of ghostMeshes) {
          const posCount = mesh.geometry?.attributes?.position?.count || 0;
          if (posCount > 0 && posCount <= 30000) {
            const edges = new THREE.LineSegments(new THREE.EdgesGeometry(mesh.geometry, 28), edgeMat);
            edges.renderOrder = 5;
            mesh.add(edges);
          }
        }
      }
      robot.userData.ghostMeshes = ghostMeshes;
      return { robot, stats: { visuals: loaded, meshes: 0, textureMaps: 0 } };
    }

    const texturesToWait = [];
    const upgradedMap = new Map();
    if (!config.preserveGltfPbr && config.pbrProfile) {
      robot.traverse(node => {
        if (!node.isMesh || !node.material) return;
        const orig = node.material;
        if (!Array.isArray(orig)) {
          let next = upgradedMap.get(orig);
          if (!next) {
            next = upgradeMaterialPbr(orig, config.pbrProfile);
            upgradedMap.set(orig, next);
          }
          node.material = next;
        }
      });
    }

    robot.traverse(node => {
      if (!node.isMesh) return;
      for (const mat of (Array.isArray(node.material) ? node.material : [node.material])) {
        if (mat?.map) texturesToWait.push(waitForTextureDecode(mat.map));
      }
    });
    if (texturesToWait.length) await Promise.all(texturesToWait);

    const stats = { visuals: loaded, meshes: 0, textureMaps: 0 };
    const seenTextures = new Set();
    robot.traverse(node => {
      if (!node.isMesh) return;
      stats.meshes++;
      for (const mat of (Array.isArray(node.material) ? node.material : [node.material])) {
        if (mat?.map?.image?.width > 0 && mat.map.image.height > 0 && !seenTextures.has(mat.map)) {
          seenTextures.add(mat.map);
          stats.textureMaps++;
        }
      }
    });
    return { robot, stats };
  } catch (error) {
    await Promise.allSettled(pending);
    disposeModel(robot);
    throw error;
  }
}

// ---- High-Visibility X-Ray Neon 3D Trajectory Helpers ----

function makeXrayAxes(size = 0.14) {
  const axes = new THREE.AxesHelper(size);
  axes.material.depthTest = false;
  axes.material.depthWrite = false;
  axes.material.toneMapped = false;
  axes.renderOrder = 20;
  return axes;
}

function makeGlowLine(color) {
  const g = new THREE.Group();
  g.userData = {
    color,
    tube: null,
    tubeGlow: null,
    spine: null,
    arrows: [],
    floorGroup: null,
  };
  return g;
}

function disposeLineMeshes(lineGroup) {
  const ud = lineGroup.userData;
  for (const k of ["tube", "tubeGlow", "spine"]) {
    if (ud[k]) {
      lineGroup.remove(ud[k]);
      ud[k].geometry.dispose();
      ud[k].material.dispose();
      ud[k] = null;
    }
  }
  if (ud.arrows?.length) {
    for (const cone of ud.arrows) {
      lineGroup.remove(cone);
      cone.geometry.dispose();
      cone.material.dispose();
    }
    ud.arrows = [];
  }
  if (ud.floorGroup) {
    lineGroup.remove(ud.floorGroup);
    ud.floorGroup.traverse(o => {
      if (o.geometry) o.geometry.dispose();
      if (o.material) o.material.dispose();
    });
    ud.floorGroup = null;
  }
}

function setLinePoints(lineGroup, pts, isLight = false) {
  const ud = lineGroup.userData;
  disposeLineMeshes(lineGroup);
  if (!pts || pts.length < 2) {
    lineGroup.visible = false;
    return;
  }
  const clean = [pts[0]];
  for (let i = 1; i < pts.length; i++) {
    if (pts[i].distanceTo(clean[clean.length - 1]) > 0.002) clean.push(pts[i]);
  }
  if (clean.length < 2) {
    lineGroup.visible = false;
    return;
  }
  lineGroup.visible = true;
  const curve = new THREE.CatmullRomCurve3(clean);
  const segs = Math.max(42, clean.length * 4);
  const arcLen = curve.getLength();

  // Outer wide X-ray neon aura (always readable even behind robot body)
  ud.tubeGlow = new THREE.Mesh(
    new THREE.TubeGeometry(curve, segs, 0.025, 10, false),
    new THREE.MeshBasicMaterial({
      color: ud.color,
      transparent: true,
      opacity: isLight ? 0.24 : 0.30,
      depthTest: false,
      depthWrite: false,
      toneMapped: false,
    })
  );
  ud.tubeGlow.renderOrder = 14;

  // Bold vivid core neon ribbon
  ud.tube = new THREE.Mesh(
    new THREE.TubeGeometry(curve, segs, 0.0115, 10, false),
    new THREE.MeshBasicMaterial({
      color: ud.color,
      transparent: true,
      opacity: 0.96,
      depthTest: false,
      depthWrite: false,
      toneMapped: false,
    })
  );
  ud.tube.renderOrder = 15;

  // White-hot centerline spine (pops crisply on both dark & light backgrounds)
  ud.spine = new THREE.Mesh(
    new THREE.TubeGeometry(curve, segs, 0.0036, 6, false),
    new THREE.MeshBasicMaterial({
      color: 0xffffff,
      transparent: true,
      opacity: isLight ? 0.92 : 0.95,
      depthTest: false,
      depthWrite: false,
      toneMapped: false,
    })
  );
  ud.spine.renderOrder = 16;
  lineGroup.add(ud.tubeGlow, ud.tube, ud.spine);

  // Directional flow arrow cones pointing Start -> End along the curve tangent
  const arrowCount = arcLen < 0.14 ? 2 : arcLen < 0.35 ? 3 : 5;
  for (let i = 0; i < arrowCount; i++) {
    const u = (i + 1) / (arrowCount + 1);
    const pos = curve.getPointAt(u);
    const tan = curve.getTangentAt(u).normalize();
    const cone = new THREE.Mesh(
      new THREE.ConeGeometry(0.018, 0.046, 12),
      new THREE.MeshBasicMaterial({
        color: isLight ? ud.color : 0xffffff,
        transparent: true,
        opacity: 0.96,
        depthTest: false,
        depthWrite: false,
        toneMapped: false,
      })
    );
    cone.position.copy(pos);
    cone.quaternion.setFromUnitVectors(UP_VEC, tan);
    cone.renderOrder = 17;
    ud.arrows.push(cone);
    lineGroup.add(cone);
  }

  // Floor projection dashed shadow + Start/End vertical height drop lines (3D spatial depth cues)
  const floorGroup = new THREE.Group();
  const floorY = 0.006;
  const floorPts = clean.map(p => new THREE.Vector3(p.x, floorY, p.z));
  const floorLineGeo = new THREE.BufferGeometry().setFromPoints(floorPts);
  const floorLine = new THREE.Line(
    floorLineGeo,
    new THREE.LineDashedMaterial({
      color: ud.color,
      transparent: true,
      opacity: isLight ? 0.65 : 0.55,
      dashSize: 0.028,
      gapSize: 0.018,
      depthWrite: false,
    })
  );
  floorLine.computeLineDistances();
  floorLine.renderOrder = 8;
  floorGroup.add(floorLine);

  const makeDrop = (pt, hex) => {
    const dropGeo = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(pt.x, floorY, pt.z),
      pt.clone(),
    ]);
    const drop = new THREE.Line(
      dropGeo,
      new THREE.LineDashedMaterial({
        color: hex,
        transparent: true,
        opacity: isLight ? 0.60 : 0.50,
        dashSize: 0.022,
        gapSize: 0.018,
        depthWrite: false,
      })
    );
    drop.computeLineDistances();
    drop.renderOrder = 8;
    return drop;
  };
  floorGroup.add(makeDrop(clean[0], 0x22c55e));
  floorGroup.add(makeDrop(clean[clean.length - 1], 0xff2e55));
  ud.floorGroup = floorGroup;
  lineGroup.add(floorGroup);
}

function makeMarkerBall(color, radius, opacity = 1.0, xray = false) {
  const m = new THREE.Mesh(
    new THREE.SphereGeometry(radius, 18, 14),
    xray
      ? new THREE.MeshBasicMaterial({
          color,
          transparent: opacity < 1,
          opacity,
          depthTest: false,
          depthWrite: false,
          toneMapped: false,
        })
      : new THREE.MeshStandardMaterial({
          color,
          emissive: color,
          emissiveIntensity: 1.35,
          transparent: opacity < 1,
          opacity,
          depthWrite: opacity >= 1,
          roughness: 0.25,
          metalness: 0.1,
        })
  );
  m.renderOrder = xray ? 18 : 12;
  return m;
}

function makeBeaconRing(color, radius, tubeRadius, opacity = 0.92, renderOrder = 18) {
  const ring = new THREE.Mesh(
    new THREE.TorusGeometry(radius, tubeRadius, 8, 28),
    new THREE.MeshBasicMaterial({
      color,
      transparent: true,
      opacity,
      depthTest: false,
      depthWrite: false,
      toneMapped: false,
    })
  );
  ring.rotation.x = Math.PI / 2;
  ring.renderOrder = renderOrder;
  return ring;
}

export function samplePlanPose(plan, t) {
  if (!plan?.samples?.length) return plan?.goal || {};
  const samples = plan.samples;
  if (t <= samples[0].t) return { ...samples[0].q };
  const last = samples[samples.length - 1];
  if (t >= last.t) return { ...last.q };
  for (let i = 0; i < samples.length - 1; i++) {
    const a = samples[i], b = samples[i + 1];
    if (t >= a.t && t <= b.t) {
      const span = Math.max(1e-6, b.t - a.t);
      const frac = (t - a.t) / span;
      const out = {};
      for (const k of Object.keys(a.q)) {
        out[k] = a.q[k] + (b.q[k] - a.q[k]) * frac;
      }
      return out;
    }
  }
  return { ...last.q };
}

export function createRobotModelViewer({ scene, camera, controls, hud, onLoadingState = () => {} }) {
  const root = new THREE.Group();
  root.rotation.x = -Math.PI / 2; // URDF Z-up -> Three Y-up.
  scene.add(root);

  const axesHelper = new THREE.AxesHelper(0.35);
  axesHelper.visible = true;
  scene.add(axesHelper);

  // High-Visibility X-Ray IK Target Marker (placed in URDF base_link frame inside `root`)
  const ikMarkerGroup = new THREE.Group();
  ikMarkerGroup.visible = false;
  const ikCore = new THREE.Mesh(
    new THREE.OctahedronGeometry(0.028, 0),
    new THREE.MeshBasicMaterial({ color: 0xfbbf24, depthTest: false, depthWrite: false, toneMapped: false })
  );
  ikCore.renderOrder = 19;
  const ikRing = makeBeaconRing(0x38bdf8, 0.052, 0.0055, 0.90, 19);
  ikRing.rotation.x = 0; // root is URDF Z-up, so XY is horizontal tabletop plane
  const ikTriad = makeXrayAxes(0.14);
  ikMarkerGroup.add(ikCore, ikRing, ikTriad);
  root.add(ikMarkerGroup);

  // High-Visibility X-Ray Neon Trajectory Trail Layer (world Three.js Y-up coordinates)
  const trailRoot = new THREE.Group();
  trailRoot.visible = false;
  scene.add(trailRoot);

  let generation = 0;
  let currentRobot = null;
  let ghostRobot = null;
  let isLightStudio = false;
  let ghostMats = makeGhostMaterials(false);
  let descriptor = null;
  let currentConfig = null;
  let feedbackPositions = {};
  let targetPositions = {};
  let displayMode = "target";
  let ghostEnabled = true;
  let trailEnabled = true;
  let activePlan = null;
  let activePlanTime = 0;
  let pendingAutoFocus = false;
  let trails = {};
  let lastTrailPoints = [];
  const segmentRoot = new THREE.Group();
  trailRoot.add(segmentRoot);
  let handModels = {};
  let handTargets = {};
  let lastCenter = new THREE.Vector3(0, 0.65, 0);
  let lastRadius = 1.2;
  let camAnim = null;

  function activeTrailColor(def) {
    return isLightStudio && def.lightColor ? def.lightColor : def.color;
  }

  function clearSegmentGroup() {
    while (segmentRoot.children.length > 0) {
      const ch = segmentRoot.children[0];
      segmentRoot.remove(ch);
      ch.traverse(o => {
        if (o.geometry) o.geometry.dispose();
        if (o.material) {
          if (Array.isArray(o.material)) o.material.forEach(m => m.dispose());
          else o.material.dispose();
        }
      });
    }
  }

  function clearTrailObjects() {
    trailRoot.visible = false;
    clearSegmentGroup();
    while (trailRoot.children.length > 0) {
      const ch = trailRoot.children[0];
      trailRoot.remove(ch);
      if (ch === segmentRoot) continue;
      ch.traverse(o => {
        if (o.geometry) o.geometry.dispose();
        if (o.material) {
          if (Array.isArray(o.material)) o.material.forEach(m => m.dispose());
          else o.material.dispose();
        }
      });
    }
    trailRoot.add(segmentRoot);
    trails = {};
    lastTrailPoints = [];
  }

  function rebuildTrailSlots(trailLinks) {
    clearTrailObjects();
    for (const def of (trailLinks || [])) {
      const col = activeTrailColor(def);
      const line = makeGlowLine(col);

      // Start Beacon: Vivid Emerald Sphere + Horizontal Ring
      const start = new THREE.Group();
      start.add(makeMarkerBall(0x22c55e, 0.022, 1.0, true));
      start.add(makeBeaconRing(0x4ade80, 0.038, 0.0045, 0.90, 18));

      // End Beacon: Vivid Coral-Red Sphere + Pulsing Double Target Ring
      const end = new THREE.Group();
      end.add(makeMarkerBall(0xff2e55, 0.026, 1.0, true));
      const endRing = makeBeaconRing(0xff2e55, 0.052, 0.006, 0.92, 18);
      end.add(endRing);

      // Moving Playback Comet: Bright White Core + Pulsing Neon Halo + Live 6D Wrist Triad
      const tip = new THREE.Group();
      tip.add(makeMarkerBall(0xffffff, 0.024, 1.0, true));
      const tipHalo = makeBeaconRing(col, 0.048, 0.0055, 0.90, 19);
      const tipAura = makeMarkerBall(col, 0.050, 0.28, true);
      tipAura.renderOrder = 18;
      const tipTriad = makeXrayAxes(0.14);
      tip.add(tipHalo, tipAura, tipTriad);

      const kfs = [];
      for (let i = 0; i < KEYFRAME_COUNT; i++) {
        const s = makeMarkerBall(col, 0.011, 0.75, true);
        s.renderOrder = 17;
        s.visible = false;
        kfs.push(s);
        trailRoot.add(s);
      }
      start.visible = end.visible = tip.visible = false;
      trailRoot.add(line, start, end, tip);
      trails[def.name] = { def, line, start, end, endRing, tip, tipHalo, tipAura, tipTriad, kfs, pts: [] };
    }
  }

  function linkWorldPos(robotObj, linkName) {
    const link = robotObj?.links?.[linkName];
    if (!link) return null;
    link.updateWorldMatrix(true, false);
    return link.getWorldPosition(new THREE.Vector3());
  }

  function linkWorldQuat(robotObj, linkName) {
    const link = robotObj?.links?.[linkName];
    if (!link) return null;
    link.updateWorldMatrix(true, false);
    return link.getWorldQuaternion(new THREE.Quaternion());
  }

  function updateGhostBranchMask(movingJointNames) {
    const meshes = ghostRobot?.userData?.ghostMeshes;
    if (!meshes || !meshes.length || !descriptor) return;
    if (!movingJointNames || !movingJointNames.length) {
      for (const m of meshes) m.visible = true;
      return;
    }
    const movingSet = new Set(movingJointNames);
    const activeMeshes = new Set();
    for (const joint of (descriptor.joints || [])) {
      if (!movingSet.has(joint.name)) continue;
      const urdfName = joint.urdf_joint || joint.name;
      const jObj = ghostRobot.joints?.[urdfName];
      if (!jObj) continue;
      const rootObj = joint.is_gripper && jObj.parent ? jObj.parent : jObj;
      rootObj.traverse(node => {
        if (node.isMesh) activeMeshes.add(node);
      });
    }
    if (activeMeshes.size === 0) {
      for (const m of meshes) m.visible = true;
      return;
    }
    for (const m of meshes) {
      m.visible = activeMeshes.has(m);
    }
  }

  function parseHexHex(raw, fallback) {
    if (typeof raw === "number" && Number.isFinite(raw)) return raw;
    if (typeof raw === "string") {
      const parsed = Number.parseInt(raw.trim().replace(/^#/, "").replace(/^0x/i, ""), 16);
      if (Number.isFinite(parsed)) return parsed;
    }
    return fallback;
  }

  function computePlanTrails(plan) {
    lastTrailPoints = [];
    clearSegmentGroup();
    if (!plan || !ghostRobot || !Object.keys(trails).length) {
      trailRoot.visible = false;
      return;
    }
    const planSamples = plan.samples || [];
    const perLink = {};
    for (const name of Object.keys(trails)) perLink[name] = [];
    for (const sp of planSamples) {
      applyModelJoints(ghostRobot, descriptor, sp.q);
      ghostRobot.updateMatrixWorld(true);
      for (const name of Object.keys(trails)) {
        const p = linkWorldPos(ghostRobot, name);
        if (p) perLink[name].push({ t: Number(sp.t) || 0, pos: p });
      }
    }

    const segments = Array.isArray(plan.step_segments) && plan.step_segments.length > 1
      ? plan.step_segments
      : null;

    let anyVisible = false;
    for (const [name, tr] of Object.entries(trails)) {
      const timedPts = perLink[name];
      const pts = timedPts.map(x => x.pos);
      tr.pts = pts;
      const col = activeTrailColor(tr.def);
      tr.line.userData.color = col;
      tr.tipHalo.material.color.setHex(col);
      tr.tipAura.material.color.setHex(col);
      let moved = false;
      if (pts.length >= 2) {
        const p0 = pts[0];
        moved = pts.some(p => p.distanceTo(p0) > 0.008);
      }
      if (!moved) {
        setLinePoints(tr.line, [], isLightStudio);
        tr.start.visible = tr.end.visible = tr.tip.visible = false;
        tr.kfs.forEach(k => { k.visible = false; });
        continue;
      }
      anyVisible = true;
      lastTrailPoints.push(...pts);

      if (segments) {
        // Multi-step workflow: render each step in its own vivid color + 3D transition checkpoints!
        setLinePoints(tr.line, [], isLightStudio);
        tr.kfs.forEach(k => { k.visible = false; });
        let segsDrawn = 0;
        for (let sIdx = 0; sIdx < segments.length; sIdx++) {
          const seg = segments[sIdx];
          const segPts = timedPts
            .filter(x => x.t >= seg.t_start - 1e-4 && x.t <= seg.t_end + 1e-4)
            .map(x => x.pos);
          if (segPts.length < 2) continue;
          const segMoved = segPts.some(p => p.distanceTo(segPts[0]) > 0.005);
          if (!segMoved) continue;
          const segCol = parseHexHex(isLightStudio ? seg.light_color : seg.color, col);
          const segLine = makeGlowLine(segCol);
          setLinePoints(segLine, segPts, isLightStudio);
          segmentRoot.add(segLine);
          segsDrawn++;

          // Add glowing 3D Step-Transition Checkpoint diamond at intermediate step boundaries
          if (sIdx < segments.length - 1) {
            const boundaryPos = segPts[segPts.length - 1];
            const cpGroup = new THREE.Group();
            cpGroup.position.copy(boundaryPos);
            const diamond = new THREE.Mesh(
              new THREE.OctahedronGeometry(0.020, 0),
              new THREE.MeshBasicMaterial({ color: segCol, depthTest: false, depthWrite: false, toneMapped: false })
            );
            diamond.renderOrder = 18;
            const ring = makeBeaconRing(segCol, 0.036, 0.0045, 0.90, 18);
            cpGroup.add(diamond, ring);
            segmentRoot.add(cpGroup);
          }
        }
        if (segsDrawn === 0) {
          setLinePoints(tr.line, pts, isLightStudio);
        }
      } else {
        setLinePoints(tr.line, pts, isLightStudio);
        for (let i = 0; i < tr.kfs.length; i++) {
          const idx = Math.min(pts.length - 1, Math.round(((i + 0.5) / tr.kfs.length) * (pts.length - 1)));
          tr.kfs[i].material.color.setHex(col);
          tr.kfs[i].position.copy(pts[idx]);
          tr.kfs[i].visible = true;
        }
      }

      tr.start.position.copy(pts[0]);
      tr.start.visible = true;
      tr.end.position.copy(pts[pts.length - 1]);
      tr.end.visible = true;
      tr.tip.position.copy(pts[0]);
      tr.tip.visible = true;
    }
    if (!trailRoot.children.includes(segmentRoot)) trailRoot.add(segmentRoot);
    trailRoot.visible = trailEnabled && anyVisible;
  }

  function focusTrajectory(animate = true) {
    const pts = [];
    for (const p of lastTrailPoints) pts.push(p);
    if (ikMarkerGroup.visible) {
      ikMarkerGroup.updateWorldMatrix(true, false);
      pts.push(ikMarkerGroup.getWorldPosition(new THREE.Vector3()));
    }
    if (pts.length < 2) {
      framePreset("iso", animate);
      return;
    }
    const box = new THREE.Box3().setFromPoints(pts);
    // Blend lightly toward robot upper torso center so viewer retains full spatial reference
    const anchor = lastCenter.clone();
    anchor.y += lastRadius * 0.14;
    box.expandByPoint(anchor.clone().lerp(box.getCenter(new THREE.Vector3()), 0.55));
    const center = box.getCenter(new THREE.Vector3());
    const size = box.getSize(new THREE.Vector3());
    const radius = Math.max(size.length() * 0.58, 0.52);
    const isoDir = currentConfig?.defaultViewDir || [1.25, 0.65, 1.4];
    const dir = new THREE.Vector3(isoDir[0], Math.max(0.45, isoDir[1]), isoDir[2]).normalize();
    const endPos = center.clone().addScaledVector(dir, radius * 2.55);
    camera.near = 0.01;
    camera.far = Math.max(lastRadius * 30, radius * 40);
    camera.updateProjectionMatrix();
    if (!animate) {
      camAnim = null;
      controls.target.copy(center);
      camera.position.copy(endPos);
      controls.update();
      return;
    }
    camAnim = {
      startPos: camera.position.clone(),
      endPos,
      startTarget: controls.target.clone(),
      endTarget: center,
      startTime: performance.now(),
      duration: 380,
    };
  }

  function computePresetPose(preset = "iso") {
    const c = lastCenter;
    const r = Math.max(lastRadius, 0.45);
    const isoDir = currentConfig?.defaultViewDir || [1.25, 0.65, 1.4];
    const offsets = {
      iso: new THREE.Vector3(r * isoDir[0], r * isoDir[1], r * isoDir[2]),
      front: new THREE.Vector3(0, r * 0.14, r * 1.85),
      side: new THREE.Vector3(r * 1.85, r * 0.14, 0),
      top: new THREE.Vector3(0.01, r * 2.05, r * 0.25),
    };
    return {
      target: c.clone(),
      position: c.clone().add(offsets[preset] || offsets.iso),
      radius: r,
    };
  }

  function framePreset(preset = "iso", animate = true) {
    const goal = computePresetPose(preset);
    camera.near = 0.01;
    camera.far = goal.radius * 30;
    camera.updateProjectionMatrix();
    if (!animate) {
      camAnim = null;
      controls.target.copy(goal.target);
      camera.position.copy(goal.position);
      controls.update();
      return;
    }
    camAnim = {
      startPos: camera.position.clone(),
      endPos: goal.position,
      startTarget: controls.target.clone(),
      endTarget: goal.target,
      startTime: performance.now(),
      duration: 360,
    };
  }

  function syncHands() {
    for (const [side, targets] of Object.entries(handTargets)) {
      for (const [name, value] of Object.entries(targets)) {
        if (Number.isFinite(value) && handModels[side]?.joints[name]) {
          handModels[side].setJointValue(name, value);
        }
      }
    }
  }

  function diffJointNames(a, b) {
    const changed = [];
    for (const k of Object.keys(b || {})) {
      const va = a?.[k], vb = b[k];
      if (Number.isFinite(va) && Number.isFinite(vb) && Math.abs(va - vb) > 0.008) changed.push(k);
    }
    return changed;
  }

  function syncPoses() {
    if (!currentRobot) return;
    if (activePlan) {
      applyModelJoints(currentRobot, descriptor, feedbackPositions);
      if (ghostRobot) {
        const movingNames = activePlan.moving?.length
          ? activePlan.moving
          : diffJointNames(activePlan.start || feedbackPositions, activePlan.goal || {});
        updateGhostBranchMask(movingNames);
        const q = samplePlanPose(activePlan, activePlanTime);
        applyModelJoints(ghostRobot, descriptor, q);
        ghostRobot.visible = ghostEnabled;
        ghostRobot.updateMatrixWorld(true);
        const activeSeg = Array.isArray(activePlan.step_segments)
          ? (activePlan.step_segments.find(s => activePlanTime >= s.t_start - 1e-4 && activePlanTime <= s.t_end + 1e-4) || activePlan.step_segments[activePlan.step_segments.length - 1])
          : null;
        for (const [name, tr] of Object.entries(trails)) {
          if (tr.tip.visible) {
            const p = linkWorldPos(ghostRobot, name);
            if (p) tr.tip.position.copy(p);
            const quat = linkWorldQuat(ghostRobot, name);
            if (quat) tr.tipTriad.quaternion.copy(quat);
            if (activeSeg) {
              const sc = parseHexHex(isLightStudio ? activeSeg.light_color : activeSeg.color, activeTrailColor(tr.def));
              tr.tipHalo.material.color.setHex(sc);
              tr.tipAura.material.color.setHex(sc);
            }
          }
        }
      }
      return;
    }

    const changed = diffJointNames(feedbackPositions, targetPositions);
    if (displayMode === "feedback") {
      applyModelJoints(currentRobot, descriptor, feedbackPositions);
      if (ghostRobot) {
        ghostRobot.visible = ghostEnabled && changed.length > 0;
        if (ghostRobot.visible) {
          updateGhostBranchMask(changed);
          applyModelJoints(ghostRobot, descriptor, targetPositions);
        }
      }
    } else {
      if (ghostRobot && ghostEnabled && changed.length > 0) {
        applyModelJoints(currentRobot, descriptor, feedbackPositions);
        updateGhostBranchMask(changed);
        applyModelJoints(ghostRobot, descriptor, targetPositions);
        ghostRobot.visible = true;
      } else {
        applyModelJoints(currentRobot, descriptor, targetPositions);
        if (ghostRobot) ghostRobot.visible = false;
      }
    }
  }

  return {
    async select(nextDescriptor) {
      const ownGeneration = ++generation;
      const sameRobot = descriptor && nextDescriptor && descriptor.id === nextDescriptor.id;
      if (!sameRobot) {
        activePlan = null;
        activePlanTime = 0;
        pendingAutoFocus = false;
        ikMarkerGroup.visible = false;
      }
      clearTrailObjects();
      if (currentRobot) {
        root.remove(currentRobot);
        disposeModel(currentRobot);
        currentRobot = null;
      }
      if (ghostRobot) {
        root.remove(ghostRobot);
        disposeModel(ghostRobot);
        ghostRobot = null;
      }
      ghostMats = makeGhostMaterials(isLightStudio);
      handModels = {};
      descriptor = nextDescriptor;
      currentConfig = modelForDescriptor(descriptor);
      hud.dataset.modelState = "loading";
      hud.dataset.textureMaps = "0";
      hud.dataset.handsLoaded = "0";

      if (!currentConfig) {
        onLoadingState({ loading: false, progress: 0, label: "" });
        hud.dataset.modelState = "unavailable";
        hud.textContent = `${descriptor?.name || "当前机型"}：缺少该机型真实模型资产，未显示替代骨架或其他机型模型。只读视图。`;
        return;
      }

      onLoadingState({ loading: true, progress: 5, label: `加载 ${currentConfig.label} (0/${currentConfig.visualCount})...` });
      hud.textContent = `${currentConfig.label} 加载中… 0 / ${currentConfig.visualCount}（只读）`;

      try {
        const sharedCache = {
          urdfText: null,
          geoCache: new Map(),
          gltfCache: new Map(),
          asGhost: false,
          ghostMaterial: ghostMats.fill,
          ghostEdgeMaterial: ghostMats.edge,
        };
        const { robot, stats } = await loadLocalRobotModel(currentConfig, (loaded, total) => {
          if (ownGeneration !== generation) return;
          const pct = Math.min(92, Math.round((loaded / Math.max(1, total)) * 92));
          onLoadingState({ loading: true, progress: pct, label: `加载 ${currentConfig.label} (${loaded}/${total})...` });
          hud.textContent = `${currentConfig.label} 网格加载 ${loaded} / ${total} · 只读`;
        }, sharedCache);
        if (ownGeneration !== generation) {
          disposeModel(robot);
          return;
        }

        // Build zero-network purple Ghost overlay from cached geometries
        sharedCache.asGhost = true;
        const { robot: ghost } = await loadLocalRobotModel(currentConfig, () => {}, sharedCache);
        if (ownGeneration !== generation) {
          disposeModel(robot);
          disposeModel(ghost);
          return;
        }
        ghost.visible = false;
        currentRobot = robot;
        ghostRobot = ghost;
        root.add(robot);
        root.add(ghost);
        rebuildTrailSlots(currentConfig.trailLinks);
        syncPoses();

        // Optional hand attachments (e.g., Dexforce W1 BrainCo Revo1 hands)
        let handCount = 0;
        const expectedHands = (currentConfig.hands || []).length;
        if (expectedHands > 0) {
          const hands = await Promise.allSettled(currentConfig.hands.map(hand => loadLocalRobotModel(hand)));
          if (ownGeneration !== generation) {
            for (const result of hands) if (result.status === "fulfilled") disposeModel(result.value.robot);
            return;
          }
          hands.forEach((result, index) => {
            const hand = currentConfig.hands[index];
            if (result.status === "fulfilled" && robot.links[hand.anchor]) {
              robot.links[hand.anchor].add(result.value.robot);
              handModels[hand.side] = result.value.robot;
              handCount++;
            } else if (result.status === "fulfilled") {
              disposeModel(result.value.robot);
            }
          });
          syncHands();
        }

        root.updateMatrixWorld(true);
        const bounds = new THREE.Box3().setFromObject(currentRobot);
        if (!bounds.isEmpty()) {
          lastCenter = bounds.getCenter(new THREE.Vector3());
          const size = bounds.getSize(new THREE.Vector3());
          lastRadius = Math.max(size.x, size.y, size.z, 0.45);
          framePreset("iso", true);
        }

        const texLabel = currentConfig.embeddedImages > 0
          ? ` · ${stats.textureMaps} 贴图已解码`
          : " · PBR 原色材质";
        const handLabel = expectedHands > 0 ? ` · BrainCo Revo1 手 ${handCount}/${expectedHands}` : "";
        hud.textContent = `${currentConfig.label} · ${stats.visuals}/${currentConfig.visualCount} ${currentConfig.formatLabel} · ${stats.meshes} 网格${texLabel}${handLabel} · 幽灵/轨迹就绪`;

        let complete = stats.textureMaps >= currentConfig.embeddedImages && handCount === expectedHands;
        if (stats.textureMaps < currentConfig.embeddedImages) {
          hud.textContent += ` · 警告：预期 ${currentConfig.embeddedImages} 张贴图，尚未完整解码`;
        }
        if (handCount !== expectedHands) {
          hud.textContent += " · 手部资产加载不完整（未造假替代）";
        }
        hud.dataset.modelState = complete ? "loaded" : "partial";
        hud.dataset.textureMaps = String(stats.textureMaps);
        hud.dataset.handsLoaded = String(handCount);
        onLoadingState({ loading: false, progress: 100, label: "" });

        if (activePlan) {
          computePlanTrails(activePlan);
          syncPoses();
          if (pendingAutoFocus) {
            pendingAutoFocus = false;
            focusTrajectory(true);
          }
        }
      } catch (error) {
        if (ownGeneration !== generation) return;
        onLoadingState({ loading: false, progress: 0, label: "" });
        hud.textContent = `${currentConfig.label} 加载失败（未用骨架冒充）：${error.message}`;
        hud.dataset.modelState = "error";
      }
    },
    updatePose(nextPositions, nextFeedback = null, nextMode = null) {
      targetPositions = { ...(nextPositions || {}) };
      if (nextFeedback) feedbackPositions = { ...nextFeedback };
      else if (!Object.keys(feedbackPositions).length) feedbackPositions = { ...targetPositions };
      if (nextMode) displayMode = nextMode;
      syncPoses();
    },
    updateHandTargets(targets) {
      handTargets = targets || {};
      syncHands();
    },
    setPlan(plan, { autoFocus = false } = {}) {
      activePlan = plan || null;
      activePlanTime = 0;
      pendingAutoFocus = false;
      if (!activePlan) {
        trailRoot.visible = false;
        ikMarkerGroup.visible = false;
        syncPoses();
        return;
      }
      if (activePlan.ik?.target_xyz?.length >= 3) {
        const [x, y, z] = activePlan.ik.target_xyz;
        ikMarkerGroup.position.set(x, y, z);
        ikMarkerGroup.visible = true;
      } else {
        ikMarkerGroup.visible = false;
      }
      if (!ghostRobot) {
        pendingAutoFocus = Boolean(autoFocus);
        return;
      }
      computePlanTrails(activePlan);
      syncPoses();
      if (autoFocus) focusTrajectory(true);
    },
    setPlanTime(t) {
      if (!activePlan) return;
      activePlanTime = Math.max(0, Math.min(activePlan.duration || 0, Number(t) || 0));
      syncPoses();
    },
    clearPlan() {
      activePlan = null;
      activePlanTime = 0;
      pendingAutoFocus = false;
      trailRoot.visible = false;
      ikMarkerGroup.visible = false;
      syncPoses();
    },
    setTrailEnabled(enabled) {
      trailEnabled = !!enabled;
      if (activePlan) {
        computePlanTrails(activePlan);
        syncPoses();
      } else {
        trailRoot.visible = false;
      }
    },
    setGhostEnabled(enabled) {
      ghostEnabled = !!enabled;
      syncPoses();
    },
    setLightTheme(isLight) {
      isLightStudio = !!isLight;
      applyGhostTheme(ghostMats.fill, ghostMats.edge, isLightStudio);
      if (activePlan) {
        computePlanTrails(activePlan);
        syncPoses();
      }
    },
    focusTrajectory() {
      focusTrajectory(true);
    },
    setIkTargetMarker(xyz) {
      if (Array.isArray(xyz) && xyz.length >= 3) {
        ikMarkerGroup.position.set(Number(xyz[0]), Number(xyz[1]), Number(xyz[2]));
        ikMarkerGroup.visible = true;
      } else {
        ikMarkerGroup.visible = false;
      }
    },
    setCameraPreset(preset) {
      framePreset(preset, true);
    },
    setAxesVisible(visible) {
      axesHelper.visible = !!visible;
    },
    setAutoRotate(enabled) {
      controls.autoRotate = !!enabled;
      controls.autoRotateSpeed = 1.6;
    },
    stepFrame() {
      const now = performance.now();
      if (camAnim) {
        const elapsed = now - camAnim.startTime;
        const t = Math.min(1, elapsed / camAnim.duration);
        const ease = 1 - Math.pow(1 - t, 3);
        camera.position.lerpVectors(camAnim.startPos, camAnim.endPos, ease);
        controls.target.lerpVectors(camAnim.startTarget, camAnim.endTarget, ease);
        if (t >= 1) camAnim = null;
      }
      // Subtle holographic pulse on active ghost & target beacons for instant visual readability
      const pulse = 0.5 + 0.5 * Math.sin(now * 0.0052);
      if (ghostRobot?.visible) {
        const baseEmissive = isLightStudio ? 0.78 : 1.20;
        ghostMats.fill.emissiveIntensity = baseEmissive + pulse * 0.45;
      }
      if (trailRoot.visible) {
        const ringScale = 0.90 + pulse * 0.32;
        const tipScale = 0.88 + (1 - pulse) * 0.28;
        for (const tr of Object.values(trails)) {
          if (tr.endRing && tr.end.visible) {
            tr.endRing.scale.set(ringScale, ringScale, ringScale);
          }
          if (tr.tipHalo && tr.tip.visible) {
            tr.tipHalo.scale.set(tipScale, tipScale, tipScale);
          }
        }
      }
      if (ikMarkerGroup.visible) {
        const s = 0.88 + pulse * 0.30;
        ikRing.scale.set(s, s, s);
      }
    },
    destroy() {
      generation++;
      clearTrailObjects();
      if (currentRobot) {
        root.remove(currentRobot);
        disposeModel(currentRobot);
      }
      if (ghostRobot) {
        root.remove(ghostRobot);
        disposeModel(ghostRobot);
      }
      disposeModel(ikMarkerGroup);
      axesHelper.dispose?.();
      scene.remove(root);
      scene.remove(axesHelper);
      scene.remove(trailRoot);
    },
  };
}
