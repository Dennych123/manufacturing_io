// Viewer + IO panel. The browser renders; it never runs the plant. Every moving-part pose comes
// from worldPoses() in lib/scene.js with the DOF values the server streams: the same function,
// the same inputs as the plant, so the picture cannot disagree with it. Geometry comes only
// from the component types' shapes in lib/components.js.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { compile, worldPoses, bindings, partRoles } from '/lib/scene.js';
import { TYPES } from '/lib/components.js';
import { qeuler } from '/lib/math.js';
import { createEditor } from '/web/editor.js';

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);          // Z-up in mm, like the scene and the plant

const RENDER_DELAY_MS = 50;   // render this far behind the newest frame and interpolate: never overshoots
const PANEL_MS = 125;         // panel text ~8x per second; rebuilding per SSE message stutters the 3D view

const $ = id => document.getElementById(id);

// ------------------------------------------------------------------ three
const view = $('view');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFShadowMap;          // PCFSoftShadowMap is gone since r18x
view.prepend(renderer.domElement);
const scene3 = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(40, 1, 10, 40000);
camera.position.set(1500, -2100, 1800);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0, 800);
controls.update();
scene3.add(new THREE.HemisphereLight(0xffffff, 0x556070, 1.4));
const sun = new THREE.DirectionalLight(0xffffff, 2.4);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
scene3.add(sun, sun.target);
const grid = new THREE.GridHelper(4000, 40, 0x8a939c, 0xb9c0c7);
grid.rotation.x = Math.PI / 2;                     // GridHelper lies in XZ; the floor here is XY
scene3.add(grid);
const floor = new THREE.Mesh(new THREE.PlaneGeometry(8000, 8000), new THREE.ShadowMaterial({ opacity: 0.18 }));
floor.receiveShadow = true;
scene3.add(floor);

function applyTheme() {
  scene3.background = new THREE.Color(getComputedStyle(document.documentElement).getPropertyValue('--view').trim() || '#dfe3e7');
}
applyTheme();
matchMedia('(prefers-color-scheme: dark)').addEventListener('change', applyTheme);

new ResizeObserver(() => {
  const w = view.clientWidth, h = view.clientHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / Math.max(1, h);
  camera.updateProjectionMatrix();
}).observe(view);

// Materials by the shapes' `mat` names.
const MAT = {
  alu: { color: '#b9c0c8', metalness: 0.5, roughness: 0.45 }, profile: { color: '#8b939c', metalness: 0.5, roughness: 0.5 },
  steel: { color: '#a4acb5', metalness: 0.7, roughness: 0.35 }, dark: { color: '#3b4047', metalness: 0.2, roughness: 0.7 },
  tube: { color: '#d5dade', metalness: 0.4, roughness: 0.3 }, rod: { color: '#eef1f3', metalness: 0.9, roughness: 0.2 },
  reed: { color: '#2b2b2b', metalness: 0.1, roughness: 0.6, glow: '#ff3b30' },
  paint: { color: '#888888', metalness: 0.1, roughness: 0.45 }, part: { color: '#c79a52', metalness: 0.3, roughness: 0.5 },
};
const shared = new Map();
function material(s, own) {
  const m = MAT[s.mat] || MAT.dark, key = s.mat + '|' + (s.color || '') + (s.ghost ? '|ghost' : '');
  if (!own && shared.has(key)) return shared.get(key);
  // ghost: a zone (remover box), seen through
  const mt = new THREE.MeshStandardMaterial({ color: s.color || m.color, metalness: m.metalness, roughness: m.roughness,
    ...(s.ghost ? { transparent: true, opacity: 0.16, depthWrite: false } : {}) });
  if (!own) shared.set(key, mt);
  return mt;
}
function geometry(s) {
  if (s.kind === 'box') return new THREE.BoxGeometry(s.size[0], s.size[1], s.size[2]);
  if (s.kind === 'sphere') return new THREE.SphereGeometry(s.r, 24, 16);
  const g = new THREE.CylinderGeometry(s.r, s.r, s.h, 32);
  g.rotateX(Math.PI / 2);                          // three cylinders run along Y; shapes run along Z
  return g;
}

/** A shape from lib/components.js as a mesh in its link's frame. */
function shapeMesh(s, own) {
  const mesh = new THREE.Mesh(geometry(s), material(s, own));
  mesh.position.set(s.at[0], s.at[1], s.at[2]);
  if (s.rot) mesh.quaternion.fromArray(qeuler(s.rot));      // the rot rule lives in lib/math.js only
  mesh.castShadow = mesh.receiveShadow = !s.ghost;
  return mesh;
}

// ------------------------------------------------------------------ model
let model = null;          // { scene, links: [{id, link, g}], glows: [...], pick: [meshes] }
let editorRef = null;      // set once the editor exists; in edit mode loose parts show at their start pose
function build(sc) {
  if (model) for (const l of model.links) { scene3.remove(l.g); l.g.traverse(o => o.geometry?.dispose()); }
  const { order, defs } = compile(sc);
  const loose = editorRef?.active ? new Map() : partRoles(sc);   // streamed, not drawn as machine
  const links = [], glows = [], pick = [], meshes = [], byKey = new Map();
  for (const c of order) {
    if (loose.has(c.id)) continue;
    const d = defs.get(c.id);
    for (const l of d.links) { const g = new THREE.Group(); scene3.add(g); links.push({ id: c.id, link: l.name, g }); byKey.set(c.id + '/' + l.name, g); }
    for (const s of d.shapes) {
      const tag = s.glow && c.io?.[s.glow];
      const mesh = shapeMesh(s, !!tag);
      mesh.userData.id = c.id;
      byKey.get(c.id + '/' + s.link).add(mesh);
      if (tag) glows.push({ mesh, tag, color: new THREE.Color(MAT[s.mat]?.glow || s.color || '#ffffff'), on: null });
      if (d.t.pressKey) pick.push(mesh);
      meshes.push(mesh);
    }
  }
  model = { scene: sc, links, glows, pick, meshes };
  for (const uid of [...partObjs.keys()]) dropPart(uid);          // templates may have changed
  partsGroup.visible = !editorRef?.active;
  place(curDof);
  fitLight();
  if (sc.name !== framed) { framed = sc.name; fitCamera(); }       // a new scene, not a rebuild after Save
}

let framed = null;
/** Look at the machine: orbit target at its bounding-box centre, from the front-right, above. */
function fitCamera() {
  const box = new THREE.Box3();
  for (const l of model.links) box.expandByObject(l.g);
  if (box.isEmpty()) return;
  const c = box.getCenter(new THREE.Vector3()), r = box.getSize(new THREE.Vector3()).length() / 2;
  controls.target.copy(c);
  camera.position.copy(c).add(new THREE.Vector3(0.5, -0.7, 0.5).normalize().multiplyScalar(r * 2.4));
  controls.update();
}

// ------------------------------------------------------------------ loose parts (streamed transforms)
const partsGroup = new THREE.Group();
scene3.add(partsGroup);
const partObjs = new Map();                                     // uid -> Group of the template's meshes
function dropPart(uid) {
  const g = partObjs.get(uid);
  if (!g) return;
  partsGroup.remove(g);
  g.traverse(o => o.geometry?.dispose());
  partObjs.delete(uid);
}
function placeParts(ps) {
  for (const uid of partObjs.keys()) if (!(uid in ps)) dropPart(uid);
  if (!model) return;
  const { defs } = compile(model.scene);
  for (const [uid, a] of Object.entries(ps)) {
    let g = partObjs.get(uid);
    if (!g) {
      const d = defs.get(partTpl[uid]);
      if (!d) continue;
      g = new THREE.Group();
      for (const s of d.shapes) { const m = shapeMesh(s, false); m.userData.part = uid; g.add(m); }
      partsGroup.add(g);
      partObjs.set(uid, g);
    }
    g.position.set(a[0], a[1], a[2]);
    g.quaternion.set(a[3], a[4], a[5], a[6]);
  }
}

function place(dof) {
  const W = worldPoses(model.scene, dof);
  for (const { id, link, g } of model.links) {
    const P = W[id]?.[link];
    if (!P) continue;
    g.position.set(P.p[0], P.p[1], P.p[2]);
    g.quaternion.set(P.q[0], P.q[1], P.q[2], P.q[3]);
  }
}

/** Shadow camera fitted to the machine's bounding box. */
function fitLight() {
  const box = new THREE.Box3();
  for (const l of model.links) box.expandByObject(l.g);
  if (box.isEmpty()) return;
  const c = box.getCenter(new THREE.Vector3()), r = box.getSize(new THREE.Vector3()).length() / 2 + 100;
  sun.target.position.copy(c);
  sun.position.copy(c).add(new THREE.Vector3(0.45, -0.35, 1).normalize().multiplyScalar(r * 2));
  const cam = sun.shadow.camera;
  cam.left = cam.bottom = -r; cam.right = cam.top = r; cam.near = 1; cam.far = r * 4;
  cam.updateProjectionMatrix();
}

// ------------------------------------------------------------------ frames and interpolation
const frames = [];
let curDof = {}, curIo = {}, forced = {}, offset = null, ioDirty = false;
let curParts = {}, partTpl = {};

function onState(m) {
  if (frames.length && m.t < frames[frames.length - 1].t) frames.length = 0;   // server restarted
  if (m.full) {
    curDof = { ...m.dof }; curIo = { ...m.io }; curParts = { ...m.parts }; partTpl = { ...m.ptpl };
  } else {
    Object.assign(curDof, m.dof); Object.assign(curIo, m.io);
    if (m.parts) Object.assign(curParts, m.parts);
    if (m.ptpl) Object.assign(partTpl, m.ptpl);
    for (const uid of m.pgone || []) { delete curParts[uid]; delete partTpl[uid]; }
  }
  if (m.forced) forced = m.forced;
  const est = m.t - performance.now();
  offset = offset == null || Math.abs(est - offset) > 500 ? est : offset + 0.05 * (est - offset);
  frames.push({ t: m.t, dof: { ...curDof }, parts: { ...curParts } });
  if (frames.length > 90) frames.splice(0, frames.length - 90);
  ioDirty = true;
}

/** DOF values at sim time t, interpolated between the two frames around it. Holds the newest; never extrapolates. */
function dofAt(t) {
  if (!frames.length) return curDof;
  if (t <= frames[0].t) return frames[0].dof;
  for (let i = frames.length - 1; i > 0; i--) {
    const a = frames[i - 1], b = frames[i];
    if (a.t <= t && t <= b.t) {
      const u = b.t === a.t ? 1 : (t - a.t) / (b.t - a.t), o = {};
      for (const k in b.dof) o[k] = a.dof[k] == null ? b.dof[k] : a.dof[k] + (b.dof[k] - a.dof[k]) * u;
      return o;
    }
  }
  return frames[frames.length - 1].dof;
}

/** Loose-part poses at sim time t: position lerp, quaternion nlerp. A part absent from either frame is taken as is. */
function partsAt(t) {
  if (!frames.length) return curParts;
  if (t <= frames[0].t) return frames[0].parts;
  for (let i = frames.length - 1; i > 0; i--) {
    const a = frames[i - 1], b = frames[i];
    if (a.t <= t && t <= b.t) {
      const u = b.t === a.t ? 1 : (t - a.t) / (b.t - a.t), o = {};
      for (const k in b.parts) {
        const p = a.parts[k], q = b.parts[k];
        if (!p) { o[k] = q; continue; }
        const s = p[3] * q[3] + p[4] * q[4] + p[5] * q[5] + p[6] * q[6] < 0 ? -1 : 1;   // shortest arc
        const r = p.map((v, j) => v + ((j < 3 ? q[j] : s * q[j]) - v) * u);
        const n = Math.hypot(r[3], r[4], r[5], r[6]) || 1;
        for (let j = 3; j < 7; j++) r[j] /= n;
        o[k] = r;
      }
      return o;
    }
  }
  return frames[frames.length - 1].parts;
}

function loop() {
  requestAnimationFrame(loop);
  if (model && offset != null) {
    const t = performance.now() + offset - RENDER_DELAY_MS;
    place(dofAt(t));
    if (!editorRef?.active) placeParts(partsAt(t));
  }
  controls.update();
  renderer.render(scene3, camera);
}
requestAnimationFrame(loop);

// ------------------------------------------------------------------ panel (built once, text updated)
let rows = new Map();
const fmtNum = v => (typeof v === 'number' ? (Number.isInteger(v) ? String(v) : v.toFixed(2)) : String(v));

function buildPanel(sc) {
  const tbody = $('io').tBodies[0];
  tbody.replaceChildren();
  rows = new Map();
  for (const b of bindings(sc)) {
    if (rows.has(b.tag)) continue;
    const tr = document.createElement('tr');
    const td = cls => { const x = document.createElement('td'); x.className = cls; tr.append(x); return x; };
    const dir = td('dir dir-' + b.dir); dir.textContent = b.dir === 'out' ? 'PLC→' : '→PLC';
    dir.title = b.dir === 'out' ? 'PLC output: actuator command' : 'plant output: sensor the PLC reads';
    td('tag').textContent = b.tag;
    const src = td('src'); src.textContent = b.comp ? b.comp + '.' + b.key : b.station ? b.station + ' step' : 'cycle ' + b.key; src.title = src.textContent;
    const val = td('val');
    const frc = td('frc');
    const send = value => post('/api/force', { tag: b.tag, value });
    if (b.type === 'BOOL') {
      for (const [label, v] of [['1', true], ['0', false], ['–', null]]) {
        const bt = document.createElement('button'); bt.textContent = label; bt.title = v == null ? 'release' : 'force ' + label;
        bt.onclick = () => send(v); frc.append(bt);
      }
    } else {
      const inp = document.createElement('input'); inp.type = 'number'; inp.step = 'any'; inp.title = 'force value (Enter)';
      inp.onchange = () => inp.value !== '' && send(Number(inp.value));       // on change, never on input
      const bt = document.createElement('button'); bt.textContent = '–'; bt.title = 'release'; bt.onclick = () => { inp.value = ''; send(null); };
      frc.append(inp, bt);
    }
    tbody.append(tr);
    rows.set(b.tag, { tr, val, type: b.type, last: '' });
  }
}

function updatePanel() {
  for (const [tag, r] of rows) {
    const v = curIo[tag], f = tag in forced;
    const txt = r.type === 'BOOL' ? (v ? 'ON' : 'off') : fmtNum(v);
    const key = txt + (f ? '|F' : '');
    if (key === r.last) continue;                          // redraw only when the text changes
    r.last = key;
    r.val.textContent = txt + (f ? ' F' : '');
    r.tr.classList.toggle('on', r.type === 'BOOL' && !!v);
    r.tr.classList.toggle('forced', f);
  }
  if (!model) return;
  for (const g of model.glows) {
    const on = !!curIo[g.tag];
    if (on === g.on) continue;
    g.on = on;
    g.mesh.material.emissive.copy(on ? g.color : new THREE.Color(0));
    g.mesh.material.emissiveIntensity = on ? 1.6 : 0;
  }
}
setInterval(() => { if (ioDirty) { ioDirty = false; updatePanel(); } }, PANEL_MS);

function onStatus(s) {
  const io = s.io || {}, pl = s.plant || {};
  const conn = $('conn');
  conn.textContent = (io.driver || '?') + ': ' + (io.msg || '');
  conn.className = io.ok ? 'ok' : 'bad';
  $('banner').hidden = io.driver !== 'internal';
  const mode = io.driver === 'internal' ? 'internal' : 'plc';
  if (!$('mode-pick').disabled && $('mode-pick').value !== mode) $('mode-pick').value = mode;
  // Run only starts the plant's clock. The machine waits for the sequence, which the PLC (or the
  // internal controller) starts from the START button: say so instead of looking frozen.
  const auto = serverScene?.cycle?.autoTag;
  const idle = pl.mode === 'run' && auto && curIo[auto] === false;
  $('status').textContent = 'plant ' + pl.mode + '   t ' + (pl.t / 1000).toFixed(1) + ' s   step ' + pl.stepUs + ' µs   overruns ' + pl.overruns
    + (pl.parts != null ? '   parts ' + pl.parts : '')
    + (io.samplingMs != null ? '\nsampling ' + io.samplingMs + ' ms   publishing ' + io.publishingMs + ' ms' + (io.rttMs != null ? '   write ' + io.rttMs + ' ms' : '') : '')
    + (idle ? '\nidle: press the green START button in 3D (the sequence has not started)' : '');
  const hb = $('hb');
  hb.hidden = io.heartbeat !== false;
  hb.textContent = 'MIO_HEARTBEAT is not moving: the PLC program is not running, or not assigned to a task.';
}

function onWarn(w) {
  const li = document.createElement('li');
  li.textContent = (w.t / 1000).toFixed(3) + '  ' + w.msg;
  const ul = $('warns');
  ul.prepend(li);
  while (ul.children.length > 40) ul.lastChild.remove();
}

async function post(url, data) {
  try {
    const r = await fetch(url, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(data) });
    if (!r.ok) onWarn({ t: performance.now() + (offset || 0), msg: url + ': ' + ((await r.json().catch(() => ({}))).error || r.status) });
  } catch (e) { onWarn({ t: 0, msg: url + ': ' + e.message }); }
}
for (const b of document.querySelectorAll('.cmds button')) if (b.dataset.op) b.onclick = () => post('/api/cmd', { op: b.dataset.op });

// ------------------------------------------------------------------ scene and controller pickers
// The server loads the scene and connects the driver; the page only asks. Both selects send on
// change (never on input) and wait for the `scene`/`status` events to come back.
const pickers = [$('scene-pick'), $('mode-pick')];
async function switchTo() {
  for (const s of pickers) s.disabled = true;
  await post('/api/switch', { scene: $('scene-pick').value, internal: $('mode-pick').value === 'internal' });
  for (const s of pickers) s.disabled = !!editor?.active;
}
for (const s of pickers) s.onchange = switchTo;
fetch('/api/scenes').then(r => r.json()).then(names => {
  $('scene-pick').replaceChildren(...names.map(n => {
    const o = document.createElement('option');
    o.value = o.textContent = n;
    o.selected = n === serverScene?.name;
    return o;
  }));
}).catch(() => {});

// ------------------------------------------------------------------ pressing panel parts in 3D
// The browser sends EDGES; the PLC enforces the conditions.
const ray = new THREE.Raycaster(), ndc = new THREE.Vector2();
let pressed = null;
function pickAt(e, list) {
  const r = renderer.domElement.getBoundingClientRect();
  ndc.set((e.clientX - r.left) / r.width * 2 - 1, -(e.clientY - r.top) / r.height * 2 + 1);
  ray.setFromCamera(ndc, camera);
  return ray.intersectObjects(list, false)[0] || null;
}
renderer.domElement.addEventListener('pointerdown', e => {
  if (!model || e.button !== 0 || editor.active) return;       // edit mode selects instead
  const hit = pickAt(e, model.pick);
  if (!hit) return;
  const id = hit.object.userData.id, c = model.scene.components.find(x => x.id === id);
  pressed = { id, key: TYPES[c.type].pressKey };
  controls.enabled = false;                                     // capture phase: before OrbitControls sees it
  post('/api/press', { ...pressed, down: true });
}, true);
addEventListener('pointerup', () => {
  if (!pressed) return;
  post('/api/press', { ...pressed, down: false });
  pressed = null;
  controls.enabled = true;
});

// ------------------------------------------------------------------ editor (web/editor.js)
let serverScene = null;
const editor = createEditor({
  scene3, camera, renderer, controls, post,
  rebuild: sc => build(sc),
  preview: sc => { model.scene = sc; place(curDof); },          // same links, new mount: no new meshes
  model: () => model, dof: () => curDof,
  pick: e => (model ? pickAt(e, model.meshes) : null),
  sceneName: () => serverScene?.name, serverScene: () => serverScene,
});
editorRef = editor;

// ------------------------------------------------------------------ stream
const es = new EventSource('/api/stream');
es.addEventListener('scene', e => {
  const { scene } = JSON.parse(e.data);
  serverScene = scene;
  $('scene-name').textContent = scene.name;
  if ($('scene-pick').value !== scene.name) $('scene-pick').value = scene.name;
  document.title = scene.name + ' · manufacturing_io';
  if (!editor.onServerScene(scene)) build(scene);
  buildPanel(scene);
  ioDirty = true;
});
es.addEventListener('state', e => onState(JSON.parse(e.data)));
es.addEventListener('status', e => onStatus(JSON.parse(e.data)));
es.addEventListener('warn', e => onWarn(JSON.parse(e.data)));
es.onerror = () => { $('conn').textContent = 'server not reachable - retrying'; $('conn').className = 'bad'; };
