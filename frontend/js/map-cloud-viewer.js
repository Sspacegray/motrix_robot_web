// Local Three.js occupancy-derived point cloud. No robot command imports.
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";

export function createMapCloudViewer({ canvas, hud }) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false });
  renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
  renderer.setClearColor(0x1e293b, 1);
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(45, 1, 0.05, 80);
  const controls = new OrbitControls(camera, canvas);
  controls.enableDamping = true;
  const root = new THREE.Group();
  root.rotation.x = -Math.PI / 2; // map Z-up -> Three Y-up
  scene.add(root);
  scene.add(new THREE.HemisphereLight(0xdbeafe, 0x0f172a, 1.1));
  scene.add(new THREE.AmbientLight(0xffffff, 0.4));
  const grid = new THREE.GridHelper(20, 20, 0x334155, 0x1e293b);
  scene.add(grid);
  let points = null, poseMark = null, goalMark = null, pathLine = null, frame = 0, active = false;
  function resize() {
    const w = canvas.clientWidth || canvas.width || 700;
    const h = canvas.clientHeight || canvas.height || 460;
    if (w <= 0 || h <= 0) return;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h, false);
  }
  function frameLoop() {
    frame = requestAnimationFrame(frameLoop);
    if (!active) return;
    controls.update();
    renderer.render(scene, camera);
  }
  function resetView(cloud) {
    const bounds = new THREE.Box3();
    if (cloud && cloud.count) {
      const pos = cloud.positions;
      for (let i = 0; i < pos.length; i += 3) bounds.expandByPoint(new THREE.Vector3(pos[i], pos[i + 1], pos[i + 2]));
    }
    const mapped = bounds.isEmpty() ? new THREE.Box3(new THREE.Vector3(-2, -2, 0), new THREE.Vector3(2, 2, 1)) : bounds;
    const local = new THREE.Box3(mapped.min, mapped.max);
    const world = local.clone().applyMatrix4(root.matrixWorld);
    const center = world.getCenter(new THREE.Vector3());
    const size = world.getSize(new THREE.Vector3());
    const radius = Math.max(size.x, size.y, size.z, 1);
    controls.target.copy(center);
    camera.position.copy(center).add(new THREE.Vector3(radius * 0.7, radius * 0.85, radius * 0.9));
    camera.near = 0.05; camera.far = radius * 20; camera.updateProjectionMatrix(); controls.update();
  }
  function setCloud(cloud) {
    if (points) { root.remove(points); points.geometry.dispose(); points.material.dispose(); points = null; }
    if (!cloud || !cloud.count) { if (hud) hud.textContent = "没有可用点云"; return; }
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.BufferAttribute(cloud.positions, 3));
    geometry.setAttribute("color", new THREE.BufferAttribute(cloud.colors, 3));
    points = new THREE.Points(geometry, new THREE.PointsMaterial({ size: 0.05, vertexColors: true, sizeAttenuation: true }));
    root.add(points);
    if (hud) hud.textContent = `${cloud.label} · ${cloud.count} 点 · 占用 ${cloud.occupiedCells} 格 · 非实时`;
    root.updateMatrixWorld(true);
    resetView(cloud);
  }
  function marker(color) {
    const mesh = new THREE.Mesh(new THREE.SphereGeometry(0.12, 16, 12), new THREE.MeshBasicMaterial({ color }));
    const shaft = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 0.4, 8), new THREE.MeshBasicMaterial({ color }));
    shaft.rotation.x = Math.PI / 2; shaft.position.z = 0.2; mesh.add(shaft); return mesh;
  }
  poseMark = marker(0x0d9488); goalMark = marker(0xf59e0b); goalMark.visible = false;
  root.add(poseMark); root.add(goalMark);
  function setOverlays({ pose, goal, path } = {}) {
    if (pose) { poseMark.position.set(pose.x || 0, pose.y || 0, 0.12); poseMark.rotation.z = pose.yaw || 0; }
    if (goal) { goalMark.visible = true; goalMark.position.set(goal.x, goal.y, 0.12); goalMark.rotation.z = goal.yaw || 0; }
    else goalMark.visible = false;
    if (pathLine) { root.remove(pathLine); pathLine.geometry.dispose(); pathLine.material.dispose(); pathLine = null; }
    if (path && path.length > 1) {
      const verts = path.flatMap(pt => [pt[0], pt[1], 0.05]);
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(verts), 3));
      pathLine = new THREE.Line(geometry, new THREE.LineDashedMaterial({ color: 0x38bdf8, dashSize: 0.15, gapSize: 0.1 }));
      pathLine.computeLineDistances();
      root.add(pathLine);
    }
  }
  resize();
  return {
    setCloud, setOverlays, resetView: () => resetView(), resize,
    setActive(next) { active = next; if (next) { resize(); if (!frame) frameLoop(); } },
    destroy() { cancelAnimationFrame(frame); frame = 0; active = false; controls.dispose(); renderer.dispose(); }
  };
}
