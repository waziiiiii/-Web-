/* Robot Arm - 3D Visualization + Joint/Task Control (integrated) */

let jointInfo = [];
let draggingSliders = new Set();
let wsCtrl = null;
let wsStream = null;
let currentMode = 'joint';
let lastEePose = null;

// Three.js
let scene, camera, renderer, orbitControls;
let stlMeshes = [];
let meshesLoaded = false;
let stlLoader = new THREE.STLLoader();
let streamFpsCount = 0, streamFps = 0, streamFpsLast = performance.now();
let renderFpsCount = 0, renderFps = 0, renderFpsLast = performance.now();

// End-effector camera (second renderer)
let eeRenderer = null;
let eeCamera = null;
let eeCamFpsCount = 0, eeCamFps = 0, eeCamFpsLast = performance.now();

// End-effector depth camera (third renderer)
let eeDepthRenderer = null;
let eeDepthCamera = null;
let eeDepthFpsCount = 0, eeDepthFps = 0, eeDepthFpsLast = performance.now();
let depthHeatmapMaterial = null;

// Interactive workspace objects
let workspaceObjects = [];
let dragControls = null;
let isDragging = false;
let eeCamEnabled = false;
let eeDepthEnabled = false;

const host = location.hostname || 'localhost';
const port = location.port || '8000';
const proto = location.protocol === 'https:' ? 'wss' : 'ws';
const baseWs = `${proto}://${host}:${port}`;

// ============================================================
// INIT
// ============================================================
document.addEventListener('DOMContentLoaded', async () => {
  initThreeJS();
  initEECamera();
  initEEDepthCamera();
  createWorkspaceObjects();
  await fetchJointInfo();
  loadAllMeshes();
  connectWS();
  initControlMode();
  animate();
});

async function fetchJointInfo() {
  const res = await fetch('/api/info');
  const data = await res.json();
  jointInfo = data.joints;
  buildSliders(jointInfo);
  buildStatusPanel(jointInfo);
}

// ============================================================
// TOAST NOTIFICATION
// ============================================================
function showToast(message, type = 'info', duration = 3000) {
  const toast = document.getElementById('toast');
  toast.textContent = message;
  toast.className = `toast ${type} show`;
  clearTimeout(toast._timeout);
  toast._timeout = setTimeout(() => { toast.className = 'toast'; }, duration);
}

// ============================================================
// MODE SWITCH
// ============================================================
function setMode(mode) {
  currentMode = mode;
  document.getElementById('btnJointMode').classList.toggle('active', mode === 'joint');
  document.getElementById('btnTaskMode').classList.toggle('active', mode === 'task');
  document.getElementById('jointCard').style.display = mode === 'joint' ? '' : 'none';
  document.getElementById('taskCard').style.display = mode === 'task' ? '' : 'none';
  if (mode === 'task') syncEToIK();
}

function syncEToIK() {
  // Read current EE pose directly from the status display (always up-to-date from stream)
  const x = parseFloat(document.getElementById('statX').textContent);
  const y = parseFloat(document.getElementById('statY').textContent);
  const z = parseFloat(document.getElementById('statZ').textContent);
  const roll = parseFloat(document.getElementById('statRoll').textContent);
  const pitch = parseFloat(document.getElementById('statPitch').textContent);
  const yaw = parseFloat(document.getElementById('statYaw').textContent);
  if (isNaN(x) || isNaN(y) || isNaN(z)) {
    showToast('尚未接收到机器人状态数据，请稍候再试', 'info', 2000);
    return;
  }
  document.getElementById('ikX').value = x.toFixed(3);
  document.getElementById('ikY').value = y.toFixed(3);
  document.getElementById('ikZ').value = z.toFixed(3);
  document.getElementById('ikRoll').value = isNaN(roll) ? '0.0' : (roll * 180 / Math.PI).toFixed(1);
  document.getElementById('ikPitch').value = isNaN(pitch) ? '0.0' : (pitch * 180 / Math.PI).toFixed(1);
  document.getElementById('ikYaw').value = isNaN(yaw) ? '0.0' : (yaw * 180 / Math.PI).toFixed(1);
  showToast('已同步当前末端位姿', 'success', 1500);
}

// ============================================================
// STL MESH LOADING
// ============================================================
function loadAllMeshes() {
  const meshNames = [
    "Link_Base_to_Shoulder_Inner.STL",
    "Link_Shoulder_Inner_to_Shoulder_Outer.STL",
    "Link_Shoulder_Outer_to_UpperArm.STL",
    "Link_UpperArm_to_Elbow.STL",
    "Link_Elbow_to_Forearm.STL",
    "Link_Forearm_to_Wrist_Upper.STL",
    "Link_Wrist_Upper_to_Wrist_Lower.STL",
    "Link_Wrist_Lower_to_Gripper.STL",
  ];

  for (let i = 0; i < meshNames.length; i++) {
    const group = new THREE.Group();
    group.visible = false;
    scene.add(group);
    stlMeshes.push(group);
  }

  let loadedCount = 0;
  meshNames.forEach((name, idx) => {
    stlLoader.load(`/meshes/${name}`, (geometry) => {
      geometry.computeVertexNormals();
      const material = new THREE.MeshPhongMaterial({
        color: 0x8899aa, specular: 0x222222, shininess: 40,
      });
      const mesh = new THREE.Mesh(geometry, material);
      stlMeshes[idx].add(mesh);
      stlMeshes[idx].visible = true;
      loadedCount++;
      if (loadedCount === meshNames.length) {
        meshesLoaded = true;
        console.log('[mesh] All meshes loaded');
      }
    }, undefined, (err) => {
      console.error(`[mesh] Failed to load ${name}:`, err);
    });
  });
}

// ============================================================
// WebSocket
// ============================================================
function connectWS() {
  connectControlWS();
  connectStreamWS();
}

function connectControlWS() {
  wsCtrl = new WebSocket(`${baseWs}/ws/control`);
  wsCtrl.onopen = () => {
    document.getElementById('dotCtrl').className = 'dot dot-ok';
    wsCtrl.send(JSON.stringify({ type: 'get_state' }));
  };
  wsCtrl.onclose = () => {
    document.getElementById('dotCtrl').className = 'dot dot-err';
    setTimeout(connectControlWS, 2000);
  };
  wsCtrl.onerror = () => document.getElementById('dotCtrl').className = 'dot dot-err';
  wsCtrl.onmessage = (evt) => handleCtrlMessage(JSON.parse(evt.data));
}

function connectStreamWS() {
  wsStream = new WebSocket(`${baseWs}/ws/stream`);
  wsStream.onopen = () => document.getElementById('dotStream').className = 'dot dot-ok';
  wsStream.onclose = () => {
    document.getElementById('dotStream').className = 'dot dot-err';
    setTimeout(connectStreamWS, 2000);
  };
  wsStream.onerror = () => document.getElementById('dotStream').className = 'dot dot-err';
  wsStream.onmessage = (evt) => {
    const msg = JSON.parse(evt.data);
    handleStreamMessage(msg);
    streamFpsCount++;
    const now = performance.now();
    if (now - streamFpsLast >= 1000) {
      streamFps = streamFpsCount;
      streamFpsCount = 0;
      streamFpsLast = now;
      document.getElementById('fpsStream').textContent = `${streamFps} FPS`;
    }
  };
}

// Handle messages from control WebSocket (responses to commands)
function handleCtrlMessage(msg) {
  if (msg.error) {
    showToast(`错误: ${msg.error}`, 'error');
    return;
  }
  // IK result
  if (msg.angles && msg.target_pos) {
    handleIKResult(msg);
  }
}


// Handle messages from stream WebSocket (continuous state updates)
function handleStreamMessage(msg) {
  if (msg.joints) updateJointUI(msg.joints);
  if (msg.links) updateMeshPoses(msg.links);
  if (msg.ee_pose) updateEEStatus(msg.ee_pose);
  // Camera data is now in a separate WebSocket
}

// ============================================================
// IK RESULT HANDLING
// ============================================================
function handleIKResult(result) {
  const resultEl = document.getElementById('ikResult');
  const err = result.ik_error || 0;
  const actual = result.actual_pos;
  const target = result.target_pos;
  const success = result.success;

  if (!success || err > 0.01) {
    // IK FAILED - robot NOT moved, inputs stay unchanged
    resultEl.innerHTML = `<span style="color:#f85149;">⚠ IK求解失败！目标超出机械臂工作空间</span><br>` +
      `误差: ${(err * 1000).toFixed(1)} mm (阈值: 10 mm)`;
    showToast(`IK求解失败！目标 (${target.x.toFixed(3)}, ${target.y.toFixed(3)}, ${target.z.toFixed(3)}) 超出机械臂工作空间，机械臂未移动`, 'error', 5000);
    // Do NOT change IK input fields or sliders - robot hasn't moved
  } else {
    // IK SUCCESS - robot moved to solved pose
    resultEl.innerHTML = `✓ 求解成功<br>` +
      `关节角: [${result.angles.map(a => a.toFixed(2)).join(', ')}]<br>` +
      `实际位置: (${actual.x.toFixed(3)}, ${actual.y.toFixed(3)}, ${actual.z.toFixed(3)})<br>` +
      `误差: ${(err * 1000).toFixed(1)} mm`;
    showToast(`IK求解成功！误差 ${(err * 1000).toFixed(1)} mm`, 'success', 2000);
    updateSlidersFromAngles(result.angles);
  }
}

function updateSlidersFromAngles(angles) {
  if (!jointInfo || jointInfo.length === 0) return;
  jointInfo.forEach((j, i) => {
    const slider = document.getElementById(`slider_${j.index}`);
    const valEl = document.getElementById(`val_${j.index}`);
    if (slider && i < angles.length) {
      slider.value = Math.round(angles[i] * 1000);
      if (valEl) valEl.textContent = angles[i].toFixed(3) + ' rad';
    }
  });
}

// ============================================================
// THREE.JS
// ============================================================
function initThreeJS() {
  const container = document.getElementById('threejs-container');
  const w = container.clientWidth;
  const h = container.clientHeight;

  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0d1117);

  camera = new THREE.PerspectiveCamera(50, w / h, 0.01, 50);
  camera.position.set(0.8, -0.8, 0.6);
  camera.up.set(0, 0, 1);

  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(w, h);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  container.appendChild(renderer.domElement);

  orbitControls = new THREE.OrbitControls(camera, renderer.domElement);
  orbitControls.target.set(0, 0, 0.2);
  orbitControls.enableDamping = true;
  orbitControls.dampingFactor = 0.05;
  orbitControls.enablePan = true;
  orbitControls.screenSpacePanning = true;
  orbitControls.rotateSpeed = 0.8;
  orbitControls.zoomSpeed = 1.2;

  scene.add(new THREE.AmbientLight(0xffffff, 0.5));
  const dl = new THREE.DirectionalLight(0xffffff, 0.8);
  dl.position.set(3, 3, 5);
  scene.add(dl);
  const dl2 = new THREE.DirectionalLight(0xffffff, 0.3);
  dl2.position.set(-2, -2, 3);
  scene.add(dl2);

  const grid = new THREE.GridHelper(3, 30, 0x30363d, 0x161b22);
  grid.rotation.x = Math.PI / 2;
  scene.add(grid);
  scene.add(new THREE.AxesHelper(0.2));

  window.addEventListener('resize', () => {
    const nw = container.clientWidth;
    const nh = container.clientHeight;
    camera.aspect = nw / nh;
    camera.updateProjectionMatrix();
    renderer.setSize(nw, nh);
  });
}

function updateMeshPoses(links) {
  if (!meshesLoaded || !links || links.length === 0) return;
  for (let i = 0; i < Math.min(links.length, stlMeshes.length); i++) {
    const link = links[i];
    const meshGroup = stlMeshes[i];
    meshGroup.position.set(link.px, link.py, link.pz);
    meshGroup.quaternion.set(link.qx, link.qy, link.qz, link.qw);
  }
}

function initEEDepthCamera() {
  const container = document.getElementById('eeDepthContainer');
  if (!container) return;
  const w = container.clientWidth;
  const h = container.clientHeight;

  eeDepthCamera = new THREE.PerspectiveCamera(60, w / h, 0.01, 10);

  eeDepthRenderer = new THREE.WebGLRenderer({ antialias: true });
  eeDepthRenderer.setSize(w, h);
  eeDepthRenderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  container.appendChild(eeDepthRenderer.domElement);

  // Handle resize
  const resizeObserver = new ResizeObserver(() => {
    const nw = container.clientWidth;
    const nh = container.clientHeight;
    if (nw > 0 && nh > 0) {
      eeDepthCamera.aspect = nw / nh;
      eeDepthCamera.updateProjectionMatrix();
      eeDepthRenderer.setSize(nw, nh);
    }
  });
  resizeObserver.observe(container);
}

function initEECamera() {
  const container = document.getElementById('eeCameraContainer');
  if (!container) return;
  const w = container.clientWidth;
  const h = container.clientHeight;

  eeCamera = new THREE.PerspectiveCamera(60, w / h, 0.01, 10);
  // Camera looks forward (-Z in local frame) from the end-effector
  // Position at the tip, looking along -Z (downward in tool frame)
  eeCamera.position.set(0, 0, 0);
  eeCamera.rotation.set(0, 0, 0);

  eeRenderer = new THREE.WebGLRenderer({ antialias: true });
  eeRenderer.setSize(w, h);
  eeRenderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  container.appendChild(eeRenderer.domElement);

  // Handle resize
  const resizeObserver = new ResizeObserver(() => {
    const nw = container.clientWidth;
    const nh = container.clientHeight;
    if (nw > 0 && nh > 0) {
      eeCamera.aspect = nw / nh;
      eeCamera.updateProjectionMatrix();
      eeRenderer.setSize(nw, nh);
    }
  });
  resizeObserver.observe(container);
}

function createWorkspaceObjects() {
  const colors = [0xe74c3c, 0x3498db, 0x2ecc71, 0xf39c12, 0x9b59b6, 0x1abc9c];
  const objDefs = [
    // Spheres — spread around robot, safe distance from base (>0.15m from center)
    { type: 'sphere', pos: [0.25, 0.25, 0.025], radius: 0.025, color: colors[0] },
    { type: 'sphere', pos: [0.30, -0.20, 0.03], radius: 0.03, color: colors[1] },
    { type: 'sphere', pos: [-0.20, 0.15, 0.02], radius: 0.02, color: colors[2] },
    { type: 'sphere', pos: [-0.15, -0.25, 0.035], radius: 0.035, color: colors[5] },
    // Boxes — spread around robot, safe distance
    { type: 'box', pos: [0.35, 0.10, 0.02], size: [0.04, 0.04, 0.04], color: colors[3] },
    { type: 'box', pos: [-0.25, -0.10, 0.015], size: [0.05, 0.03, 0.03], color: colors[4] },
    { type: 'box', pos: [0.15, -0.30, 0.035], size: [0.035, 0.035, 0.035], color: colors[0] },
    { type: 'box', pos: [0.35, -0.25, 0.025], size: [0.03, 0.05, 0.025], color: colors[2] },
  ];

  objDefs.forEach(def => {
    let geometry, mesh;
    if (def.type === 'sphere') {
      geometry = new THREE.SphereGeometry(def.radius, 24, 24);
    } else {
      geometry = new THREE.BoxGeometry(def.size[0], def.size[1], def.size[2]);
    }
    const material = new THREE.MeshPhongMaterial({
      color: def.color, specular: 0x333333, shininess: 30,
    });
    mesh = new THREE.Mesh(geometry, material);
    mesh.position.set(def.pos[0], def.pos[1], def.pos[2]);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    scene.add(mesh);
    workspaceObjects.push(mesh);
  });

  // Setup DragControls
  if (typeof THREE.DragControls !== 'undefined' && workspaceObjects.length > 0) {
    dragControls = new THREE.DragControls(workspaceObjects, camera, renderer.domElement);
    dragControls.addEventListener('dragstart', (event) => {
      isDragging = true;
      orbitControls.enabled = false;
      event.object.material.emissive = new THREE.Color(0x333333);
    });
    dragControls.addEventListener('dragend', (event) => {
      isDragging = false;
      orbitControls.enabled = true;
      event.object.material.emissive = new THREE.Color(0x000000);
      // Clamp Z to ground plane
      const obj = event.object;
      const halfH = obj.geometry.boundingSphere ? obj.geometry.boundingSphere.radius : 0.05;
      if (obj.position.z < halfH) obj.position.z = halfH;
      // Sync positions to backend for PyBullet collision + camera
      syncObjectPositions();
    });
    dragControls.addEventListener('drag', (event) => {
      // Constrain to XY plane (Z = ground level)
      const obj = event.object;
      const halfH = obj.geometry.boundingSphere ? obj.geometry.boundingSphere.radius : 0.05;
      if (obj.position.z < halfH) obj.position.z = halfH;
    });
  }
}

function animate() {
  requestAnimationFrame(animate);
  if (orbitControls) orbitControls.update();
  renderer.render(scene, camera);
  renderFpsCount++;
  const now = performance.now();
  if (now - renderFpsLast >= 1000) {
    renderFps = renderFpsCount;
    renderFpsCount = 0;
    renderFpsLast = now;
    document.getElementById('fps3D').textContent = `${renderFps} FPS`;
  }

  // Render end-effector camera view
  if (eeCamEnabled && eeRenderer && eeCamera && meshesLoaded && stlMeshes.length > 0) {
    const eeGroup = stlMeshes[stlMeshes.length - 1]; // last link = gripper
    const eePos = new THREE.Vector3();
    const eeQuat = new THREE.Quaternion();
    eeGroup.getWorldPosition(eePos);
    eeGroup.getWorldQuaternion(eeQuat);

    // Extract rotation matrix and get local X axis (first column: m[0], m[1], m[2])
    const rotMat = new THREE.Matrix4().makeRotationFromQuaternion(eeQuat);
    const m = rotMat.elements; // column-major: [m00,m10,m20,m30, m01,m11,m21,m31, ...]
    const localX = new THREE.Vector3(m[0], m[1], m[2]); // local X axis in world frame

    // Camera position: EE + 15cm along local X (forward, away from gripper body)
    eeCamera.position.set(
      eePos.x + localX.x * 0.15,
      eePos.y + localX.y * 0.15,
      eePos.z + localX.z * 0.15
    );

    // Look along local X direction
    eeCamera.lookAt(
      eeCamera.position.x + localX.x,
      eeCamera.position.y + localX.y,
      eeCamera.position.z + localX.z
    );

    // Up vector: world Z (same as PyBullet camera), with fallback if nearly collinear
    const dotZX = localX.x * 0 + localX.y * 0 + localX.z * 1;
    if (Math.abs(dotZX) > 0.95) {
      eeCamera.up.set(0, 1, 0);
    } else {
      eeCamera.up.set(0, 0, 1);
    }

    // Temporarily hide the last 3 link meshes to avoid self-occlusion
    for (let i = stlMeshes.length - 3; i < stlMeshes.length; i++) {
      stlMeshes[i].visible = false;
    }
    eeRenderer.render(scene, eeCamera);
    // Restore visibility
    for (let i = stlMeshes.length - 3; i < stlMeshes.length; i++) {
      stlMeshes[i].visible = true;
    }
    eeCamFpsCount++;
    if (now - eeCamFpsLast >= 1000) {
      eeCamFps = eeCamFpsCount;
      eeCamFpsCount = 0;
      eeCamFpsLast = now;
      const el = document.getElementById('eeCamFPS');
      if (el) el.textContent = `${eeCamFps} FPS`;
    }
  }

  // Render end-effector depth view using MeshDepthMaterial
  if (eeDepthEnabled && eeDepthRenderer && eeDepthCamera && meshesLoaded && stlMeshes.length > 0) {
    const eeGroup = stlMeshes[stlMeshes.length - 1];
    const eePos = new THREE.Vector3();
    const eeQuat = new THREE.Quaternion();
    eeGroup.getWorldPosition(eePos);
    eeGroup.getWorldQuaternion(eeQuat);

    const rotMat = new THREE.Matrix4().makeRotationFromQuaternion(eeQuat);
    const m = rotMat.elements;
    const localX = new THREE.Vector3(m[0], m[1], m[2]);

    // Depth camera position: EE + 15cm along local X (forward, away from gripper body)
    eeDepthCamera.position.set(
      eePos.x + localX.x * 0.15,
      eePos.y + localX.y * 0.15,
      eePos.z + localX.z * 0.15
    );
    eeDepthCamera.lookAt(
      eeDepthCamera.position.x + localX.x,
      eeDepthCamera.position.y + localX.y,
      eeDepthCamera.position.z + localX.z
    );

    // Up vector: world Z (same as PyBullet camera)
    const dotZXD = localX.x * 0 + localX.y * 0 + localX.z * 1;
    if (Math.abs(dotZXD) > 0.95) {
      eeDepthCamera.up.set(0, 1, 0);
    } else {
      eeDepthCamera.up.set(0, 0, 1);
    }

    // Swap materials to custom depth heatmap shader for depth rendering
    if (!depthHeatmapMaterial) {
      depthHeatmapMaterial = new THREE.ShaderMaterial({
        vertexShader: `
          varying float vDepth;
          void main() {
            vec4 mvPos = modelViewMatrix * vec4(position, 1.0);
            vDepth = -mvPos.z;
            gl_Position = projectionMatrix * mvPos;
          }
        `,
        fragmentShader: `
          uniform float uNear;
          uniform float uFar;
          varying float vDepth;
          vec3 colormap(float t) {
            // Red(warm, near) -> Yellow -> Cyan -> Blue(cool, far)
            float r = clamp(1.5 - abs(t - 0.25) * 4.0, 0.0, 1.0);
            float g = clamp(1.5 - abs(t - 0.5) * 4.0, 0.0, 1.0);
            float b = clamp(1.5 - abs(t - 0.75) * 4.0, 0.0, 1.0);
            return vec3(r, g, b);
          }
          void main() {
            float t = clamp((vDepth - uNear) / (uFar - uNear), 0.0, 1.0);
            gl_FragColor = vec4(colormap(t), 1.0);
          }
        `,
        uniforms: {
          uNear: { value: 0.01 },
          uFar: { value: 0.5 },
        },
        side: THREE.DoubleSide,
      });
    }

    const originalMaterials = [];
    for (let i = 0; i < stlMeshes.length; i++) {
      const group = stlMeshes[i];
      if (group.children && group.children.length > 0) {
        const mesh = group.children[0];
        originalMaterials.push(mesh.material);
        mesh.material = depthHeatmapMaterial;
      }
    }

    // Also swap workspace objects materials to depth shader
    const originalWkspMaterials = [];
    for (let i = 0; i < workspaceObjects.length; i++) {
      originalWkspMaterials.push(workspaceObjects[i].material);
      workspaceObjects[i].material = depthHeatmapMaterial;
    }

    // Hide last 3 links to avoid self-occlusion
    for (let i = stlMeshes.length - 3; i < stlMeshes.length; i++) {
      stlMeshes[i].visible = false;
    }
    eeDepthRenderer.render(scene, eeDepthCamera);
    for (let i = stlMeshes.length - 3; i < stlMeshes.length; i++) {
      stlMeshes[i].visible = true;
    }

    // Restore original materials
    let matIdx = 0;
    for (let i = 0; i < stlMeshes.length; i++) {
      const group = stlMeshes[i];
      if (group.children && group.children.length > 0) {
        group.children[0].material = originalMaterials[matIdx];
        matIdx++;
      }
    }

    // Restore workspace objects materials
    for (let i = 0; i < workspaceObjects.length; i++) {
      workspaceObjects[i].material = originalWkspMaterials[i];
    }

    eeDepthFpsCount++;
    if (now - eeDepthFpsLast >= 1000) {
      eeDepthFps = eeDepthFpsCount;
      eeDepthFpsCount = 0;
      eeDepthFpsLast = now;
      const el = document.getElementById('eeDepthFPS');
      if (el) el.textContent = `${eeDepthFps} FPS`;
    }
  }
}

// ============================================================
// UI: Sliders
// ============================================================
function buildSliders(joints) {
  const container = document.getElementById('jointSliders');
  container.innerHTML = '';
  joints.forEach((j, i) => {
    const row = document.createElement('div');
    row.className = 'joint-row';

    const label = document.createElement('span');
    label.className = 'joint-label';
    label.textContent = `J${i}: ${j.name}`;
    label.title = j.name;

    const slider = document.createElement('input');
    slider.type = 'range';
    slider.min = Math.round(j.lower * 1000);
    slider.max = Math.round(j.upper * 1000);
    slider.step = '1';
    slider.value = '0';
    slider.id = `slider_${j.index}`;

    slider.addEventListener('pointerdown', (e) => {
      draggingSliders.add(j.index);
      e.target.setPointerCapture(e.pointerId);
    });
    slider.addEventListener('pointerup', () => draggingSliders.delete(j.index));
    slider.addEventListener('lostpointercapture', () => draggingSliders.delete(j.index));

    slider.addEventListener('input', () => {
      const val = parseFloat(slider.value) / 1000;
      document.getElementById(`val_${j.index}`).textContent = val.toFixed(3) + ' rad';
      if (wsCtrl && wsCtrl.readyState === WebSocket.OPEN) {
        wsCtrl.send(JSON.stringify({ type: 'set_joint', index: j.index, angle: val }));
      }
    });

    const valSpan = document.createElement('span');
    valSpan.className = 'joint-val';
    valSpan.id = `val_${j.index}`;
    valSpan.textContent = '0.000 rad';

    row.appendChild(label);
    row.appendChild(slider);
    row.appendChild(valSpan);
    container.appendChild(row);
  });
}

function buildStatusPanel(joints) {
  const container = document.getElementById('jointStatus');
  container.innerHTML = '';
  const grid = document.createElement('div');
  grid.className = 'status-grid';
  joints.forEach((j, i) => {
    grid.innerHTML += `<span class="status-key">J${i} ${j.name}:</span><span class="status-val" id="statusPos_${j.index}">-</span>`;
  });
  container.appendChild(grid);
}

function updateJointUI(joints) {
  joints.forEach(j => {
    const slider = document.getElementById(`slider_${j.index}`);
    const valEl = document.getElementById(`val_${j.index}`);
    const statEl = document.getElementById(`statusPos_${j.index}`);
    const isDragging = draggingSliders.has(j.index);
    if (slider && !isDragging) slider.value = Math.round(j.position * 1000);
    if (valEl && !isDragging) valEl.textContent = j.position.toFixed(3) + ' rad';
    if (statEl) statEl.textContent = j.position.toFixed(3) + ' rad';
  });
}

// ============================================================
// UI: EE Status
// ============================================================
function updateEEStatus(ee) {
  lastEePose = ee;
  const fields = { 'X': 'statX', 'Y': 'statY', 'Z': 'statZ',
    'Roll': 'statRoll', 'Pitch': 'statPitch', 'Yaw': 'statYaw' };
  const vals = { 'X': ee.x, 'Y': ee.y, 'Z': ee.z,
    'Roll': ee.roll, 'Pitch': ee.pitch, 'Yaw': ee.yaw };
  for (const [k, id] of Object.entries(fields)) {
    const el = document.getElementById(id);
    if (el && typeof vals[k] === 'number') el.textContent = vals[k].toFixed(4);
  }
}


// ============================================================
// IK Control
// ============================================================
function ikDelta(axis, sign) {
  const step = 0.01;
  const input = document.getElementById(`ik${axis.toUpperCase()}`);
  let val = parseFloat(input.value) || 0;
  val += sign * step;
  input.value = val.toFixed(3);
  solveIK();
}

function ikDeltaRPY(axis, sign) {
  const step = 5; // 5 degrees
  const map = { roll: 'ikRoll', pitch: 'ikPitch', yaw: 'ikYaw' };
  const input = document.getElementById(map[axis]);
  let val = parseFloat(input.value) || 0;
  val += sign * step;
  input.value = val.toFixed(1);
  solveIK();
}

// ============================================================
// Control Mode Switching
// ============================================================
function switchControlMode(mode) {
  const statusEl = document.getElementById('controlModeStatus');
  statusEl.textContent = '切换中...';
  statusEl.style.color = '#d29922';

  fetch('/api/control_mode', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ mode: mode }),
  })
    .then(res => res.json())
    .then(data => {
      if (data.status === 'ok') {
        const label = mode === 'motor' ? '物理仿真' : '精确模式';
        statusEl.textContent = `✓ ${label}`;
        statusEl.style.color = '#3fb950';
        showToast(`已切换到${label}模式`, 'success', 2000);
        updateCollisionVisibility(mode);
        setTimeout(() => { statusEl.textContent = ''; }, 3000);
      } else {
        statusEl.textContent = '切换失败';
        statusEl.style.color = '#f85149';
      }
    })
    .catch(err => {
      statusEl.textContent = '切换失败';
      statusEl.style.color = '#f85149';
      showToast(`模式切换失败: ${err.message}`, 'error');
    });
}

function initControlMode() {
  fetch('/api/control_mode')
    .then(res => res.json())
    .then(data => {
      document.getElementById('controlModeSelect').value = data.mode;
      updateCollisionVisibility(data.mode);
    })
    .catch(() => {});
}

// ============================================================
// Collision Detection Polling
// ============================================================
let collisionPollInterval = null;

function updateCollisionVisibility(mode) {
  // Collision detection runs in both modes now
  const indicator = document.getElementById('collisionIndicator');
  indicator.style.display = '';
  startCollisionPolling();
}

function startCollisionPolling() {
  if (collisionPollInterval) return;
  collisionPollInterval = setInterval(pollCollisionState, 500);
}

function stopCollisionPolling() {
  if (collisionPollInterval) {
    clearInterval(collisionPollInterval);
    collisionPollInterval = null;
  }
  document.getElementById('collisionIndicator').style.display = 'none';
}

function pollCollisionState() {
  fetch('/api/collision')
    .then(res => res.json())
    .then(data => {
      const indicator = document.getElementById('collisionIndicator');
      const countEl = document.getElementById('collisionCount');
      if (data.detected) {
        indicator.style.display = '';
        indicator.style.color = '#f85149';
        // Build detailed text with force and position inline
        let parts = [];
        for (const c of data.collisions) {
          const f = c.force.toFixed(1);
          const px = c.position.map(v => v.toFixed(2)).join(',');
          if (c.type === 'self') {
            parts.push(`${c.link_a_name}↔${c.link_b_name}(F:${f}N)`);
          } else if (c.type === 'object') {
            parts.push(`${c.robot_link_name}↔${c.object_name}(F:${f}N)`);
          } else if (c.type === 'ground') {
            parts.push(`${c.robot_link_name}→地面(F:${f}N)`);
          }
        }
        countEl.textContent = parts.join(' | ') || `${data.count}处`;
      } else {
        indicator.style.display = '';
        indicator.style.color = '#3fb950';
        countEl.textContent = '无碰撞';
      }
    })
    .catch(() => {});
}


function syncObjectPositions() {
  const objects = workspaceObjects.map(obj => ({
    pos: [obj.position.x, obj.position.y, obj.position.z]
  }));
  fetch('/api/workspace/objects', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ objects: objects }),
  }).catch(() => {});
}


function toggleEECameras() {
  const rgbOn = document.getElementById('toggleEECam').checked;
  const depthOn = document.getElementById('toggleEEDepth').checked;
  document.getElementById('eeCamBox').style.display = rgbOn ? '' : 'none';
  document.getElementById('depthBox').style.display = depthOn ? '' : 'none';
  eeCamEnabled = rgbOn;
  eeDepthEnabled = depthOn;
}

// ============================================================
// PyBullet On-Demand Camera Capture
// ============================================================
function capturePybulletFrame() {
  const btn = document.getElementById('btnPybulletCapture');
  const statusEl = document.getElementById('pybulletCaptureStatus');

  // Disable button during capture
  btn.disabled = true;
  btn.textContent = '⏳ 拍摄中...';
  statusEl.textContent = '正在启动 PyBullet 后台线程...';
  statusEl.style.color = '#d29922';

  // Step 1: Collect workspace object positions from Three.js scene
  const objects = workspaceObjects.map(obj => ({
    pos: [obj.position.x, obj.position.y, obj.position.z]
  }));

  // Step 2: Trigger capture (send object positions along with the request)
  fetch('/api/camera/capture', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ objects: objects }),
  })
    .then(res => res.json())
    .then(data => {
      if (data.status === 'busy') {
        statusEl.textContent = '已有拍摄任务进行中，请稍候...';
        statusEl.style.color = '#f85149';
        btn.disabled = false;
        btn.textContent = '📷 拍摄一帧';
        return;
      }
      statusEl.textContent = '后台线程已启动，等待渲染完成...';

      // Step 2: Poll for completion
      pollCameraResult(0);
    })
    .catch(err => {
      statusEl.textContent = `请求失败: ${err.message}`;
      statusEl.style.color = '#f85149';
      btn.disabled = false;
      btn.textContent = '📷 拍摄一帧';
    });
}

function pollCameraResult(attempt) {
  const btn = document.getElementById('btnPybulletCapture');
  const statusEl = document.getElementById('pybulletCaptureStatus');
  const MAX_ATTEMPTS = 30; // 30 * 500ms = 15s timeout

  if (attempt >= MAX_ATTEMPTS) {
    statusEl.textContent = '拍摄超时（15秒），请重试';
    statusEl.style.color = '#f85149';
    btn.disabled = false;
    btn.textContent = '📷 拍摄一帧';
    return;
  }

  setTimeout(() => {
    fetch('/api/camera/status')
      .then(res => res.json())
    .then(status => {
        if (status.capturing) {
          // Still capturing — keep polling
          statusEl.textContent = `渲染中... (${(attempt + 1) * 0.5}s)`;
          pollCameraResult(attempt + 1);
        } else if (status.has_frame) {
          // Done — fetch the frame, pass save_path along
          statusEl.textContent = '拍摄完成，加载图像...';
          statusEl.style.color = '#3fb950';
          fetchCameraImages(status.save_path || '');
        } else if (status.error) {
          // Backend reported an error
          statusEl.textContent = `拍摄失败: ${status.error}`;
          statusEl.style.color = '#f85149';
          btn.disabled = false;
          btn.textContent = '📷 拍摄一帧';
        } else {
          // Not capturing, no frame, no error yet — might be a race, retry a few times
          if (attempt < 5) {
            pollCameraResult(attempt + 1);
          } else {
            statusEl.textContent = '拍摄失败（无图像数据，请检查服务器日志）';
            statusEl.style.color = '#f85149';
            btn.disabled = false;
            btn.textContent = '📷 拍摄一帧';
          }
        }
      })
      .catch(() => {
        // Network error — retry
        pollCameraResult(attempt + 1);
      });
  }, 500);
}

function fetchCameraImages(savePath) {
  const btn = document.getElementById('btnPybulletCapture');
  const statusEl = document.getElementById('pybulletCaptureStatus');

  fetch('/api/camera')
    .then(res => res.json())
    .then(data => {
      if (data.error) {
        statusEl.textContent = data.error;
        statusEl.style.color = '#f85149';
        btn.disabled = false;
        btn.textContent = '📷 拍摄一帧';
        return;
      }

      // Show RGB image
      const rgbImg = document.getElementById('pybulletRGB');
      const rgbPlaceholder = document.getElementById('pybulletRGBPlaceholder');
      rgbImg.src = 'data:image/jpeg;base64,' + data.rgb_base64;
      rgbImg.style.display = 'block';
      rgbPlaceholder.style.display = 'none';

      // Show depth image
      const depthImg = document.getElementById('pybulletDepth');
      const depthPlaceholder = document.getElementById('pybulletDepthPlaceholder');
      depthImg.src = 'data:image/jpeg;base64,' + data.depth_base64;
      depthImg.style.display = 'block';
      depthPlaceholder.style.display = 'none';

      const ts = new Date().toLocaleTimeString();
      let statusText = `✓ 拍摄完成 (${ts}) — ${data.width}×${data.height}`;
      if (savePath) statusText += `\n已保存: ${savePath}`;
      statusEl.textContent = statusText;
      statusEl.style.color = '#3fb950';

      btn.disabled = false;
      btn.textContent = '📷 再拍一帧';
    })
    .catch(err => {
      statusEl.textContent = `图像加载失败: ${err.message}`;
      statusEl.style.color = '#f85149';
      btn.disabled = false;
      btn.textContent = '📷 拍摄一帧';
    });
}


function solveIK() {
  const x = parseFloat(document.getElementById('ikX').value) || 0;
  const y = parseFloat(document.getElementById('ikY').value) || 0;
  const z = parseFloat(document.getElementById('ikZ').value) || 0;
  const rollDeg = parseFloat(document.getElementById('ikRoll').value) || 0;
  const pitchDeg = parseFloat(document.getElementById('ikPitch').value) || 0;
  const yawDeg = parseFloat(document.getElementById('ikYaw').value) || 0;
  const rollRad = rollDeg * Math.PI / 180;
  const pitchRad = pitchDeg * Math.PI / 180;
  const yawRad = yawDeg * Math.PI / 180;

  document.getElementById('ikResult').textContent = '求解中...';

  if (wsCtrl && wsCtrl.readyState === WebSocket.OPEN) {
    wsCtrl.send(JSON.stringify({
      type: 'ik', x: x, y: y, z: z,
      roll: rollRad, pitch: pitchRad, yaw: yawRad,
    }));
  }
}