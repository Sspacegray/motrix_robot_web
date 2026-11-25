// Local Three.js 3D point-cloud viewer & interactive 3D picking/path visualization
// (supports real recorded LiDAR .pcd & occupancy-derived clouds).
// Strictly read-only presentation & raycast callbacks: zero robot command or Lease API imports.
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { raycastCloudSurface } from "./cloud-planner-3d.js";

function makeBadgeSprite(text, { bg = "rgba(15,23,42,0.88)", border = "#38bdf8", fg = "#f8fafc" } = {}) {
  const c = document.createElement("canvas");
  c.width = 128;
  c.height = 64;
  const ctx = c.getContext("2d");
  ctx.clearRect(0, 0, c.width, c.height);
  ctx.beginPath();
  if (typeof ctx.roundRect === "function") ctx.roundRect(12, 10, 104, 44, 14);
  else ctx.rect(12, 10, 104, 44);
  ctx.fillStyle = bg;
  ctx.fill();
  ctx.lineWidth = 4;
  ctx.strokeStyle = border;
  ctx.stroke();
  ctx.font = "bold 26px ui-monospace, SFMono-Regular, Menlo, monospace";
  ctx.fillStyle = fg;
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText(text, 64, 33);

  const tex = new THREE.CanvasTexture(c);
  tex.needsUpdate = true;
  const mat = new THREE.SpriteMaterial({ map: tex, transparent: true, depthTest: false });
  const sprite = new THREE.Sprite(mat);
  sprite.scale.set(1.05, 0.52, 1);
  sprite.renderOrder = 10;
  return sprite;
}

function disposeGroup(group) {
  while (group.children.length > 0) {
    const child = group.children[0];
    group.remove(child);
    child.traverse((obj) => {
      if (obj.geometry) obj.geometry.dispose();
      if (obj.material) {
        const mats = Array.isArray(obj.material) ? obj.material : [obj.material];
        for (const m of mats) {
          if (m.map) m.map.dispose();
          m.dispose();
        }
      }
    });
  }
}

export function createMapCloudViewer({ canvas, hud, onHover = null, onPick = null }) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: false, alpha: false, powerPreference: "high-performance" });
  renderer.setPixelRatio(Math.min(1.5, window.devicePixelRatio || 1));
  renderer.setClearColor(0x0b1220, 1);
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(45, 1, 0.05, 240);
  const controls = new OrbitControls(camera, canvas);
  controls.enableDamping = true;
  controls.dampingFactor = 0.12;

  const root = new THREE.Group();
  root.rotation.x = -Math.PI / 2; // ROS map Z-up -> Three.js Y-up
  scene.add(root);
  scene.add(new THREE.HemisphereLight(0xdbeafe, 0x0f172a, 1.15));
  scene.add(new THREE.AmbientLight(0xffffff, 0.45));

  const gridHelper = new THREE.GridHelper(64, 64, 0x26354d, 0x162033);
  gridHelper.position.y = -0.24;
  scene.add(gridHelper);

  let points = null;
  let pointMaterial = null;
  let currentCloud = null;
  let traversabilityGrid = null;
  let interactionMode = "orbit"; // "orbit" | "pick_goal" | "plan_waypoints"
  let pointSize = 0.08;
  let frame = 0;
  let active = false;
  let animClock = 0;
  let needsRender = true;
  let controlsActive = false;
  let lastRenderTs = 0;

  function markDirty() {
    needsRender = true;
  }
  controls.addEventListener("start", () => { controlsActive = true; markDirty(); });
  controls.addEventListener("change", () => { markDirty(); });
  controls.addEventListener("end", () => { controlsActive = false; markDirty(); });

  const raycaster = new THREE.Raycaster();
  const invRoot = new THREE.Matrix4();

  // Dynamic overlay groups (disposed cleanly on update)
  const pathGroup = new THREE.Group();
  const waypointGroup = new THREE.Group();
  const wallGroup = new THREE.Group();
  root.add(pathGroup);
  root.add(waypointGroup);
  root.add(wallGroup);

  // True-scale R1 Pro 3D robot footprint & heading marker
  function createRobotMarker() {
    const g = new THREE.Group();
    const base = new THREE.Mesh(
      new THREE.CylinderGeometry(0.36, 0.36, 0.08, 24),
      new THREE.MeshBasicMaterial({ color: 0x14b8a6, transparent: true, opacity: 0.72 })
    );
    base.rotation.x = Math.PI / 2;
    g.add(base);
    const ring = new THREE.Mesh(
      new THREE.RingGeometry(0.40, 0.50, 28),
      new THREE.MeshBasicMaterial({ color: 0x2dd4bf, side: THREE.DoubleSide, transparent: true, opacity: 0.85 })
    );
    ring.position.z = 0.05;
    g.add(ring);
    const arrow = new THREE.Mesh(
      new THREE.ConeGeometry(0.20, 0.56, 12),
      new THREE.MeshBasicMaterial({ color: 0x5eead4 })
    );
    arrow.rotation.z = -Math.PI / 2;
    arrow.position.set(0.42, 0, 0.16);
    g.add(arrow);
    return g;
  }

  // Pulsing 3D Goal Target Beacon
  function createGoalMarker() {
    const g = new THREE.Group();
    const disc = new THREE.Mesh(
      new THREE.CylinderGeometry(0.30, 0.30, 0.06, 24),
      new THREE.MeshBasicMaterial({ color: 0xf59e0b, transparent: true, opacity: 0.85 })
    );
    disc.rotation.x = Math.PI / 2;
    g.add(disc);
    const pulseRing = new THREE.Mesh(
      new THREE.RingGeometry(0.42, 0.56, 28),
      new THREE.MeshBasicMaterial({ color: 0xfbbf24, side: THREE.DoubleSide, transparent: true, opacity: 0.75 })
    );
    pulseRing.position.z = 0.04;
    pulseRing.name = "pulseRing";
    g.add(pulseRing);
    const stem = new THREE.Mesh(
      new THREE.CylinderGeometry(0.04, 0.04, 1.05, 10),
      new THREE.MeshBasicMaterial({ color: 0xfbbf24, transparent: true, opacity: 0.82 })
    );
    stem.rotation.x = Math.PI / 2;
    stem.position.z = 0.54;
    g.add(stem);
    const cap = new THREE.Mesh(
      new THREE.SphereGeometry(0.18, 16, 12),
      new THREE.MeshBasicMaterial({ color: 0xf59e0b })
    );
    cap.position.z = 1.08;
    g.add(cap);
    const arrow = new THREE.Mesh(
      new THREE.ConeGeometry(0.16, 0.45, 10),
      new THREE.MeshBasicMaterial({ color: 0xfde68a })
    );
    arrow.rotation.z = -Math.PI / 2;
    arrow.position.set(0.55, 0, 0.14);
    g.add(arrow);
    return g;
  }

  // Live 3D Hover Reticle (snaps to real point-cloud floor during pick_goal / plan_waypoints)
  function createHoverReticle() {
    const g = new THREE.Group();
    const ringMat = new THREE.MeshBasicMaterial({ color: 0x34d399, side: THREE.DoubleSide, transparent: true, opacity: 0.92 });
    const ring = new THREE.Mesh(new THREE.RingGeometry(0.28, 0.42, 28), ringMat);
    ring.name = "reticleRing";
    g.add(ring);
    const coreMat = new THREE.MeshBasicMaterial({ color: 0x34d399, transparent: true, opacity: 0.5 });
    const core = new THREE.Mesh(new THREE.CircleGeometry(0.16, 18), coreMat);
    core.name = "reticleCore";
    g.add(core);
    const pinMat = new THREE.MeshBasicMaterial({ color: 0x34d399, transparent: true, opacity: 0.72 });
    const pin = new THREE.Mesh(new THREE.CylinderGeometry(0.025, 0.025, 0.8, 8), pinMat);
    pin.rotation.x = Math.PI / 2;
    pin.position.z = 0.42;
    pin.name = "reticlePin";
    g.add(pin);
    g.visible = false;
    return g;
  }

  const poseMark = createRobotMarker();
  const goalMark = createGoalMarker();
  const hoverMark = createHoverReticle();
  goalMark.visible = false;
  root.add(poseMark);
  root.add(goalMark);
  root.add(hoverMark);

  function setReticleColor(hex) {
    for (const name of ["reticleRing", "reticleCore", "reticlePin"]) {
      const obj = hoverMark.getObjectByName(name);
      if (obj?.material) obj.material.color.setHex(hex);
    }
  }

  function raycastFromPointer(e) {
    if (!traversabilityGrid || !canvas) return null;
    const rect = canvas.getBoundingClientRect();
    if (rect.width <= 0 || rect.height <= 0) return null;
    const ndc = new THREE.Vector2(
      ((e.clientX - rect.left) / rect.width) * 2 - 1,
      -((e.clientY - rect.top) / rect.height) * 2 + 1
    );
    raycaster.setFromCamera(ndc, camera);
    root.updateMatrixWorld(true);
    invRoot.copy(root.matrixWorld).invert();
    const localOrigin = raycaster.ray.origin.clone().applyMatrix4(invRoot);
    const localDir = raycaster.ray.direction.clone().transformDirection(invRoot);
    return raycastCloudSurface(
      traversabilityGrid,
      [localOrigin.x, localOrigin.y, localOrigin.z],
      [localDir.x, localDir.y, localDir.z]
    );
  }

  // Pointer picking (strictly opt-in: does NOTHING in default "orbit" mode; drag > 5px still orbits cleanly!)
  let downPos = null;
  function onPointerDown(e) {
    if (interactionMode === "orbit" || e.button !== 0) {
      downPos = null;
      return;
    }
    downPos = { x: e.clientX, y: e.clientY };
  }

  function onPointerMove(e) {
    if (!active || interactionMode === "orbit") {
      hoverMark.visible = false;
      return;
    }
    const hit = raycastFromPointer(e);
    if (!hit) {
      if (hoverMark.visible) {
        hoverMark.visible = false;
        markDirty();
      }
      if (onHover) onHover(null);
      return;
    }
    hoverMark.visible = true;
    hoverMark.position.set(hit.x, hit.y, hit.z + 0.05);
    if (hit.state === "occupied") setReticleColor(0xf43f5e);
    else if (hit.state === "unknown") setReticleColor(0x94a3b8);
    else if (hit.clearanceM !== null && hit.clearanceM < 0.34) setReticleColor(0xfbbf24);
    else setReticleColor(0x34d399);
    markDirty();
    if (onHover) onHover(hit);
  }

  function onPointerUp(e) {
    if (!downPos || interactionMode === "orbit" || e.button !== 0) {
      downPos = null;
      return;
    }
    const dist = Math.hypot(e.clientX - downPos.x, e.clientY - downPos.y);
    downPos = null;
    if (dist > 5) return; // User orbited camera -> never trigger accidental pick!
    const hit = raycastFromPointer(e);
    if (hit && onPick) onPick(hit, interactionMode);
  }

  function onPointerLeave() {
    if (hoverMark.visible) {
      hoverMark.visible = false;
      markDirty();
    }
    downPos = null;
    if (onHover) onHover(null);
  }

  canvas.addEventListener("pointerdown", onPointerDown);
  canvas.addEventListener("pointermove", onPointerMove);
  canvas.addEventListener("pointerup", onPointerUp);
  canvas.addEventListener("pointerleave", onPointerLeave);

  function resize() {
    const w = canvas.clientWidth || canvas.width || 700;
    const h = canvas.clientHeight || canvas.height || 460;
    if (w <= 0 || h <= 0) return;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h, false);
    markDirty();
  }

  function frameLoop(ts = 0) {
    frame = requestAnimationFrame(frameLoop);
    if (!active) return;
    // Demand-driven WebGL rendering: keeps 981k-point clouds silky & GPU-friendly!
    const damped = controls.update();
    const shouldPulse = (goalMark.visible || hoverMark.visible) && (ts - lastRenderTs >= 90);
    if (!needsRender && !damped && !controlsActive && !shouldPulse) return;

    animClock += 0.06;
    if (goalMark.visible) {
      const pr = goalMark.getObjectByName("pulseRing");
      if (pr) {
        const s = 1.0 + 0.14 * Math.sin(animClock * 2.0);
        pr.scale.set(s, s, 1);
      }
    }
    if (hoverMark.visible) {
      const hr = hoverMark.getObjectByName("reticleRing");
      if (hr) hr.rotation.z = animClock * 0.8;
    }
    needsRender = false;
    lastRenderTs = ts;
    renderer.render(scene, camera);
  }

  function resetView(cloud = currentCloud) {
    const bounds = new THREE.Box3();
    if (cloud && cloud.count) {
      const pos = cloud.positions;
      const step = Math.max(1, Math.floor(cloud.count / 12000)) * 3;
      for (let i = 0; i < pos.length; i += step) {
        bounds.expandByPoint(new THREE.Vector3(pos[i], pos[i + 1], pos[i + 2]));
      }
    }
    const mapped = bounds.isEmpty()
      ? new THREE.Box3(new THREE.Vector3(-4, -4, 0), new THREE.Vector3(4, 4, 2))
      : bounds;
    const world = mapped.clone().applyMatrix4(root.matrixWorld);
    const center = world.getCenter(new THREE.Vector3());
    const size = world.getSize(new THREE.Vector3());
    const radius = Math.max(size.x, size.y, size.z, 2);
    controls.target.copy(center);
    camera.position.copy(center).add(new THREE.Vector3(radius * 0.36, radius * 0.78, radius * 0.44));
    camera.near = 0.05;
    camera.far = Math.max(140, radius * 16);
    camera.updateProjectionMatrix();
    controls.update();
  }

  function setCloud(cloud, { preserveCamera = false } = {}) {
    currentCloud = cloud || null;
    if (points) {
      root.remove(points);
      points.geometry.dispose();
      points.material.dispose();
      points = null;
      pointMaterial = null;
    }
    if (!cloud || !cloud.count) {
      if (hud) hud.textContent = "没有可用点云";
      return;
    }
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.BufferAttribute(cloud.positions, 3));
    geometry.setAttribute("color", new THREE.BufferAttribute(cloud.colors, 3));
    pointMaterial = new THREE.PointsMaterial({
      size: pointSize,
      vertexColors: true,
      sizeAttenuation: true,
    });
    points = new THREE.Points(geometry, pointMaterial);
    root.add(points);

    if (hud) {
      const extra = cloud.occupiedCells !== undefined
        ? `占用 ${cloud.occupiedCells} 格`
        : cloud.totalCount && cloud.totalCount !== cloud.count
          ? `切片 ${cloud.count.toLocaleString()} / ${cloud.totalCount.toLocaleString()} 点`
          : `${cloud.count.toLocaleString()} 点`;
      const dynNote = cloud.dynamicRemoved > 0
        ? ` · 已消除 ${cloud.dynamicRemoved.toLocaleString()} 动态鬼影点`
        : "";
      hud.textContent = `3D 点云 · ${extra}${dynNote} · 离线静态建图`;
    }
    root.updateMatrixWorld(true);
    if (!preserveCamera) resetView(cloud);
    markDirty();
  }

  function setTraversabilityGrid(grid) {
    traversabilityGrid = grid || null;
    if (traversabilityGrid) {
      gridHelper.position.y = (traversabilityGrid.defaultFloorZ ?? -0.2) - 0.04;
    }
    markDirty();
  }

  function setInteractionMode(mode) {
    interactionMode = mode === "pick_goal" || mode === "plan_waypoints" ? mode : "orbit";
    if (interactionMode === "orbit") {
      hoverMark.visible = false;
      canvas.style.cursor = "grab";
    } else {
      canvas.style.cursor = "crosshair";
    }
    markDirty();
  }

  function setPointSize(size) {
    pointSize = Math.max(0.02, Math.min(0.35, Number(size) || 0.08));
    if (pointMaterial) {
      pointMaterial.size = pointSize;
      pointMaterial.needsUpdate = true;
    }
    markDirty();
  }

  function setOverlays({ pose, goal, path, waypoints = [], virtualWalls = [], activeNav = false } = {}) {
    const floorZ = traversabilityGrid?.defaultFloorZ ?? 0.0;
    if (pose) {
      poseMark.position.set(pose.x || 0, pose.y || 0, (pose.z ?? floorZ) + 0.10);
      poseMark.rotation.z = pose.yaw || 0;
    }
    if (goal) {
      goalMark.visible = true;
      goalMark.position.set(goal.x, goal.y, (goal.z ?? floorZ) + 0.06);
      goalMark.rotation.z = goal.yaw || 0;
    } else {
      goalMark.visible = false;
    }

    // 1. Rebuild glowing 3D elevated path ribbon + directional chevrons
    disposeGroup(pathGroup);
    if (Array.isArray(path) && path.length > 1) {
      const vecs = [];
      for (const pt of path) {
        const vx = Number(pt[0]);
        const vy = Number(pt[1]);
        const vz = pt.length >= 3 && Number.isFinite(pt[2]) ? Number(pt[2]) : floorZ + 0.22;
        if (vecs.length === 0 || vecs[vecs.length - 1].distanceToSquared(new THREE.Vector3(vx, vy, vz)) > 1e-4) {
          vecs.push(new THREE.Vector3(vx, vy, vz));
        }
      }
      if (vecs.length >= 2) {
        const curve = new THREE.CatmullRomCurve3(vecs);
        const segs = Math.min(320, Math.max(24, vecs.length * 3));
        const mainColor = activeNav ? 0x2dd4bf : 0x38bdf8;
        const glowColor = activeNav ? 0x14b8a6 : 0x0ea5e9;

        const coreTube = new THREE.Mesh(
          new THREE.TubeGeometry(curve, segs, 0.10, 8, false),
          new THREE.MeshBasicMaterial({ color: mainColor })
        );
        pathGroup.add(coreTube);

        // Subtle X-Ray pass so path remains traceable behind tall 3D walls or overhead ceilings
        const xrayTube = new THREE.Mesh(
          new THREE.TubeGeometry(curve, Math.max(16, Math.floor(segs / 2)), 0.06, 6, false),
          new THREE.MeshBasicMaterial({ color: mainColor, transparent: true, opacity: 0.28, depthTest: false, depthWrite: false })
        );
        xrayTube.renderOrder = 6;
        pathGroup.add(xrayTube);

        const haloTube = new THREE.Mesh(
          new THREE.TubeGeometry(curve, Math.max(16, Math.floor(segs / 2)), 0.20, 8, false),
          new THREE.MeshBasicMaterial({ color: glowColor, transparent: true, opacity: 0.24, depthWrite: false })
        );
        pathGroup.add(haloTube);

        // Directional 3D flow cones along trajectory every ~2.4m
        const totalLen = curve.getLength();
        const nCones = Math.min(28, Math.max(1, Math.floor(totalLen / 2.4)));
        const coneGeo = new THREE.ConeGeometry(0.18, 0.44, 10);
        const coneMat = new THREE.MeshBasicMaterial({ color: 0xe0f2fe });
        const upAxis = new THREE.Vector3(0, 1, 0);
        for (let i = 1; i <= nCones; i++) {
          const u = i / (nCones + 1);
          const pos = curve.getPointAt(u);
          const tan = curve.getTangentAt(u).normalize();
          const cone = new THREE.Mesh(coneGeo, coneMat);
          cone.position.copy(pos).add(new THREE.Vector3(0, 0, 0.14));
          cone.quaternion.setFromUnitVectors(upAxis, tan);
          pathGroup.add(cone);
        }
      }
    }

    // 2. Rebuild numbered 3D point-cloud waypoint beacons (P1, P2, P3...)
    disposeGroup(waypointGroup);
    if (Array.isArray(waypoints) && waypoints.length > 0) {
      waypoints.forEach((wp, idx) => {
        const isLast = idx === waypoints.length - 1;
        const wz = Number.isFinite(wp.z) ? wp.z : floorZ;
        const accent = isLast ? 0xf59e0b : 0xc084fc;
        const g = new THREE.Group();
        g.position.set(wp.x, wp.y, wz + 0.04);

        const ring = new THREE.Mesh(
          new THREE.RingGeometry(0.26, 0.38, 24),
          new THREE.MeshBasicMaterial({ color: accent, side: THREE.DoubleSide, transparent: true, opacity: 0.9 })
        );
        g.add(ring);

        const pillar = new THREE.Mesh(
          new THREE.CylinderGeometry(0.035, 0.035, 0.95, 10),
          new THREE.MeshBasicMaterial({ color: accent, transparent: true, opacity: 0.78 })
        );
        pillar.rotation.x = Math.PI / 2;
        pillar.position.z = 0.48;
        g.add(pillar);

        const orb = new THREE.Mesh(
          new THREE.SphereGeometry(0.14, 14, 10),
          new THREE.MeshBasicMaterial({ color: accent })
        );
        orb.position.z = 0.96;
        g.add(orb);

        const badge = makeBadgeSprite(wp.label || `P${idx + 1}`, {
          border: isLast ? "#f59e0b" : "#c084fc",
          fg: isLast ? "#fef3c7" : "#f3e8ff",
        });
        badge.position.set(0, 0, 1.48);
        g.add(badge);
        waypointGroup.add(g);
      });
    }

    // 3. Render 2D Virtual Walls as translucent crimson 3D Keepout Laser Barriers inside the point cloud
    disposeGroup(wallGroup);
    if (Array.isArray(virtualWalls) && virtualWalls.length > 0) {
      const wallH = 1.45;
      for (const vw of virtualWalls) {
        const len = Math.hypot(vw.x2 - vw.x1, vw.y2 - vw.y1);
        if (len < 0.1) continue;
        const mx = (vw.x1 + vw.x2) * 0.5;
        const my = (vw.y1 + vw.y2) * 0.5;
        const ang = Math.atan2(vw.y2 - vw.y1, vw.x2 - vw.x1);
        const panel = new THREE.Mesh(
          new THREE.BoxGeometry(len, Math.max(0.08, vw.thickness_m || 0.16), wallH),
          new THREE.MeshBasicMaterial({ color: 0xf43f5e, transparent: true, opacity: 0.38, depthWrite: false })
        );
        panel.position.set(mx, my, floorZ + wallH * 0.5);
        panel.rotation.z = ang;
        wallGroup.add(panel);

        const topRail = new THREE.Mesh(
          new THREE.BoxGeometry(len, 0.10, 0.08),
          new THREE.MeshBasicMaterial({ color: 0xfbbf24 })
        );
        topRail.position.set(mx, my, floorZ + wallH);
        topRail.rotation.z = ang;
        wallGroup.add(topRail);
      }
    }
    markDirty();
  }

  resize();
  return {
    setCloud,
    setTraversabilityGrid,
    setInteractionMode,
    setPointSize,
    setOverlays,
    resetView: () => { resetView(); markDirty(); },
    resize,
    setActive(next) {
      active = next;
      if (next) {
        resize();
        markDirty();
        if (!frame) frameLoop();
      } else {
        hoverMark.visible = false;
        cancelAnimationFrame(frame);
        frame = 0;
      }
    },
    destroy() {
      cancelAnimationFrame(frame);
      frame = 0;
      active = false;
      canvas.removeEventListener("pointerdown", onPointerDown);
      canvas.removeEventListener("pointermove", onPointerMove);
      canvas.removeEventListener("pointerup", onPointerUp);
      canvas.removeEventListener("pointerleave", onPointerLeave);
      disposeGroup(pathGroup);
      disposeGroup(waypointGroup);
      disposeGroup(wallGroup);
      controls.dispose();
      renderer.dispose();
    },
  };
}
