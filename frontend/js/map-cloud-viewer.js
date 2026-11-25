// Local Three.js 3D point-cloud viewer (supports real recorded LiDAR .pcd & occupancy-derived clouds).
// Strictly read-only: no robot command or Lease API imports.
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";

export function createMapCloudViewer({ canvas, hud }) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: false });
  renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
  renderer.setClearColor(0x0f172a, 1);
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(45, 1, 0.05, 220);
  const controls = new OrbitControls(camera, canvas);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;

  const root = new THREE.Group();
  root.rotation.x = -Math.PI / 2; // ROS map Z-up -> Three.js Y-up
  scene.add(root);
  scene.add(new THREE.HemisphereLight(0xdbeafe, 0x0f172a, 1.1));
  scene.add(new THREE.AmbientLight(0xffffff, 0.4));

  const grid = new THREE.GridHelper(60, 60, 0x334155, 0x1e293b);
  grid.position.y = -0.02;
  scene.add(grid);

  let points = null;
  let pointMaterial = null;
  let currentCloud = null;
  let pointSize = 0.08;
  let poseMark = null;
  let goalMark = null;
  let pathLine = null;
  let frame = 0;
  let active = false;

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
    camera.position.copy(center).add(new THREE.Vector3(radius * 0.42, radius * 0.72, radius * 0.52));
    camera.near = 0.05;
    camera.far = Math.max(120, radius * 16);
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
          ? `切片显示 ${cloud.count.toLocaleString()} / ${cloud.totalCount.toLocaleString()} 点`
          : `${cloud.count.toLocaleString()} 点`;
      hud.textContent = `${cloud.label} · ${extra} · 非实时`;
    }
    root.updateMatrixWorld(true);
    if (!preserveCamera) resetView(cloud);
  }

  function setPointSize(size) {
    pointSize = Math.max(0.02, Math.min(0.35, Number(size) || 0.08));
    if (pointMaterial) {
      pointMaterial.size = pointSize;
      pointMaterial.needsUpdate = true;
    }
  }

  function marker(color) {
    const mesh = new THREE.Mesh(
      new THREE.SphereGeometry(0.28, 16, 12),
      new THREE.MeshBasicMaterial({ color })
    );
    const shaft = new THREE.Mesh(
      new THREE.CylinderGeometry(0.06, 0.06, 0.9, 8),
      new THREE.MeshBasicMaterial({ color })
    );
    shaft.rotation.z = -Math.PI / 2;
    shaft.position.x = 0.45;
    mesh.add(shaft);
    return mesh;
  }

  poseMark = marker(0x14b8a6);
  goalMark = marker(0xf59e0b);
  goalMark.visible = false;
  root.add(poseMark);
  root.add(goalMark);

  function setOverlays({ pose, goal, path } = {}) {
    if (pose) {
      poseMark.position.set(pose.x || 0, pose.y || 0, 0.35);
      poseMark.rotation.z = pose.yaw || 0;
    }
    if (goal) {
      goalMark.visible = true;
      goalMark.position.set(goal.x, goal.y, 0.35);
      goalMark.rotation.z = goal.yaw || 0;
    } else {
      goalMark.visible = false;
    }
    if (pathLine) {
      root.remove(pathLine);
      pathLine.geometry.dispose();
      pathLine.material.dispose();
      pathLine = null;
    }
    if (path && path.length > 1) {
      const verts = path.flatMap(pt => [pt[0], pt[1], 0.2]);
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(verts), 3));
      pathLine = new THREE.Line(
        geometry,
        new THREE.LineDashedMaterial({ color: 0x38bdf8, dashSize: 0.25, gapSize: 0.14 })
      );
      pathLine.computeLineDistances();
      root.add(pathLine);
    }
  }

  resize();
  return {
    setCloud,
    setPointSize,
    setOverlays,
    resetView: () => resetView(),
    resize,
    setActive(next) {
      active = next;
      if (next) {
        resize();
        if (!frame) frameLoop();
      }
    },
    destroy() {
      cancelAnimationFrame(frame);
      frame = 0;
      active = false;
      controls.dispose();
      renderer.dispose();
    },
  };
}
