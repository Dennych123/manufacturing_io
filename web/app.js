// Viewer + IO panel. The browser renders; it never runs the plant. Every moving-part pose comes
// from worldPoses() in lib/scene.js with the DOF values the server streams: the same function,
// the same inputs as the plant, so the picture cannot disagree with it. Geometry comes only
// from the component types' shapes in lib/components.js.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { ColladaLoader } from 'three/addons/loaders/ColladaLoader.js';
import { compile, worldPoses, bindings, partRoles, params } from '/lib/scene.js';
import { TYPES, COLORS } from '/lib/components.js';
import { qeuler } from '/lib/math.js';
import { createEditor } from '/web/editor.js';
import { createPov } from '/web/pov.js';
import { createWorkshop } from '/web/workshop.js';

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
// The hall around the machine (web/workshop.js). It is scenery: the plant never hears about it,
// and the only thing the viewer asks it for besides a picture is what the walk-in camera may not
// walk through. With it on, the grid and the shadow-catcher plane are its floor's job instead.
const workshop = createWorkshop(scene3);
function setWorkshop(on) {
  workshop.visible = on;
  grid.visible = floor.visible = !on;
  refreshSolids();
  $('shop-btn').textContent = on ? 'Workshop' : 'Grid';
}

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
// A machine drawn all in grey is unreadable, so each FAMILY has its own colour: blue is
// pneumatic power, bronze moves, orange touches the part, green holds it, teal senses it, and
// the structure stays grey so the working parts stand out against it.
const MAT = {
  alu: { color: '#b9c0c8', metalness: 0.5, roughness: 0.45 }, profile: { color: '#8b939c', metalness: 0.5, roughness: 0.5 },
  steel: { color: '#a4acb5', metalness: 0.7, roughness: 0.35 }, dark: { color: '#3b4047', metalness: 0.2, roughness: 0.7 },
  tube: { color: '#7f95b5', metalness: 0.5, roughness: 0.35 }, rod: { color: '#eef1f3', metalness: 0.9, roughness: 0.2 },
  reed: { color: '#2b2b2b', metalness: 0.1, roughness: 0.6, glow: '#ff3b30' },
  paint: { color: '#888888', metalness: 0.1, roughness: 0.45 }, part: { color: '#c79a52', metalness: 0.3, roughness: 0.5 },
  motion: { color: '#a8792c', metalness: 0.6, roughness: 0.4 },   // servo rails, index tables: what drives
  tool: { color: '#c06a30', metalness: 0.4, roughness: 0.45 },    // pusher plates, stopper pins, press heads
  holder: { color: '#3f8f5a', metalness: 0.4, roughness: 0.45 },  // nests, gripper fingers, vacuum cups
  sensor: { color: '#1f6f5c', metalness: 0.3, roughness: 0.55 },  // photo-eye and proximity bodies
};
const shared = new Map();
function material(s, own) {
  const m = MAT[s.mat] || MAT.dark, key = s.mat + '|' + (s.color || '') + (s.ghost ? '|ghost' : '') + (s.opacity ? '|o' + s.opacity : '');
  if (!own && shared.has(key)) return shared.get(key);
  // ghost: a zone (remover box), seen through. `opacity`: a part that is really there but must be
  // seen through, e.g. a machine cover over the board it is pressing.
  const mt = new THREE.MeshStandardMaterial({ color: s.color || m.color, metalness: m.metalness, roughness: m.roughness,
    ...(s.ghost ? { transparent: true, opacity: 0.16, depthWrite: false } : {}),
    ...(s.opacity ? { transparent: true, opacity: s.opacity } : {}) });
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

// A `mesh` shape names an asset (a robot's own shell, from its URDF package) instead of a
// primitive. It loads once per asset and the geometry is SHARED by every link that names it, so
// a rebuild must not dispose it - see `shared` below.
// Two formats, because robot makers do not agree on one: ROS-Industrial ships the Fanuc as STL
// (one BufferGeometry) and DENSO ships the VS-060 as COLLADA (a whole scene graph, with its own
// materials). So this hands back an OBJECT3D either way, and the caller clones it - a cached
// geometry cannot be added to two links at once, but a clone of a cached prototype can.
//
// A .dae is NOT rotated on the way in, even though these files say <up_axis>Y_UP</up_axis>. That
// tag is metadata the URDF toolchain ignores: a URDF visual has rpy="0 0 0", so the vertex data
// must already sit in the link frame, which is Z-up. Measured on the DENSO meshes - J2's upper
// arm is 421 mm along Z, J1 is tallest in Z, base_link is a flat 171 x 160 x 30 plate - so the
// data is Z-up and an "up-axis correction" here rotates them a second time. That is what an arm
// scattered into loose pieces looks like.
const stl = new STLLoader();
const dae = new ColladaLoader();
const meshGeo = new Map();
function meshAsset(asset) {
  let p = meshGeo.get(asset);
  if (!p) {
    p = (/\.dae$/i.test(asset)
      ? dae.loadAsync(asset).then(c => c.scene)
      : stl.loadAsync(asset).then(g => { g.computeVertexNormals(); return new THREE.Mesh(g); }))
      .catch(e => { console.warn('mesh ' + asset + ' did not load:', e.message); return null; });
    meshGeo.set(asset, p);
  }
  return p;
}

/** A shape from lib/components.js as a mesh in its link's frame. */
function shapeMesh(s, own) {
  const mesh = new THREE.Mesh(geometry(s), material(s, own));
  mesh.position.set(s.at[0], s.at[1], s.at[2]);
  if (s.rot) mesh.quaternion.fromArray(qeuler(s.rot));      // the rot rule lives in lib/math.js only
  mesh.castShadow = mesh.receiveShadow = !s.ghost && !s.opacity;   // a see-through leaf casting a solid shadow reads as solid
  mesh.userData.ghost = !!s.ghost;        // a zone is drawn, not there: the walk-in camera goes through it
  return mesh;
}

// ------------------------------------------------------------------ model
let model = null;          // { scene, links: [{id, link, g}], glows: [...], pick: [meshes] }
let editorRef = null;      // set once the editor exists; in edit mode loose parts show at their start pose
function build(sc) {
  if (model) for (const l of model.links) { scene3.remove(l.g); l.g.traverse(o => { if (!o.userData.shared) o.geometry?.dispose(); }); }
  const { order, defs } = compile(sc);
  const loose = editorRef?.active ? new Map() : partRoles(sc);   // streamed, not drawn as machine
  const links = [], glows = [], pick = [], meshes = [], byKey = new Map();
  for (const c of order) {
    if (loose.has(c.id)) continue;
    const d = defs.get(c.id);
    if (d.t.group === 'operator') continue;                     // drawn in the HTML operator panel, not in 3D
    for (const l of d.links) { const g = new THREE.Group(); scene3.add(g); links.push({ id: c.id, link: l.name, g }); byKey.set(c.id + '/' + l.name, g); }
    // `draw: false` is a collider the viewer must not draw: the link has a real shell instead, and
    // drawing both puts a grey box through the middle of the robot. It IS what gets drawn when the
    // shell is missing, though - a maker's CAD is often not ours to ship, so a scene that names one
    // must still draw as boxes for anyone who has not got it.
    const hidden = new Map();
    for (const s of d.shapes) if (s.draw === false) (hidden.get(s.link) || hidden.set(s.link, []).get(s.link)).push(s);
    for (const s of d.shapes) {
      if (s.draw === false) continue;
      if (s.kind === 'mesh') {
        // The asset loads asynchronously, so the link gets an empty group now and the shell when
        // it arrives. A rebuild in between removes this group from the scene, and the load then
        // resolves into an orphan that is never drawn - which is what should happen.
        const g = new THREE.Group();
        g.position.set(s.at[0], s.at[1], s.at[2]);
        if (s.rot) g.quaternion.fromArray(qeuler(s.rot));
        const k = s.scale ?? 1;
        g.scale.set(k, k, k);
        byKey.get(c.id + '/' + s.link).add(g);
        meshAsset(s.asset).then(proto => {
          if (!proto) {
            // The shell did not load: draw the primitives it was standing in for, so the link is
            // still there. An invisible arm is the one failure a viewer must never show quietly.
            for (const f of hidden.get(s.link) || []) byKey.get(c.id + '/' + s.link).add(shapeMesh(f, false));
            return;
          }
          const m = proto.clone(true);
          // A scene's own colour wins over the model's, so a robot can be painted the maker's
          // yellow or white without editing the mesh. Without a colour, a COLLADA keeps the
          // materials it shipped with.
          m.traverse(o => {
            if (!o.isMesh) return;
            if (s.color || !/\.dae$/i.test(s.asset)) o.material = material(s, false);
            o.castShadow = o.receiveShadow = true;
            o.userData.id = c.id;
            o.userData.shared = true;               // the prototype's geometry is cached: never dispose it
          });
          m.userData.shared = true;
          g.add(m);
        });
        continue;
      }
      const tag = s.glow && c.io?.[s.glow];
      const mesh = shapeMesh(s, !!tag);
      mesh.userData.id = c.id;
      byKey.get(c.id + '/' + s.link).add(mesh);
      if (tag) glows.push({ mesh, tag, color: new THREE.Color(MAT[s.mat]?.glow || s.color || '#ffffff'), on: null });
      if (d.t.pressKey) pick.push(mesh);
      meshes.push(mesh);
    }
  }
  model = { scene: sc, links, glows, pick, meshes, defs };
  for (const uid of [...partObjs.keys()]) dropPart(uid);          // templates may have changed
  partsGroup.visible = !editorRef?.active;
  place(curDof);
  fitLight();
  const box = sceneBox();
  workshop.fit(box);                                               // the hall is sized to the machine
  pov.spawnFrom(box);                                              // and the walk starts in front of it
  refreshSolids();
  if (sc.name !== framed) { framed = sc.name; fitCamera(); }       // a new scene, not a rebuild after Save
}

let framed = null;
/** The machine's bounding box: what the camera frames, the shadow camera covers and the hall is built around. */
function sceneBox() {
  const box = new THREE.Box3();
  for (const l of model.links) box.expandByObject(l.g);
  return box;
}
/** Look at the machine: orbit target at its bounding-box centre, from the front-right, above. */
function fitCamera() {
  const box = sceneBox();
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
let pins = new Set();                                           // uids the hand is holding (highlighted)
const LIT = new THREE.Color('#2a6fdb');
const litMats = new Map();                                      // base material -> its highlighted copy
function litOf(base) {
  let m = litMats.get(base);
  if (!m) { m = base.clone(); m.emissive = LIT.clone(); m.emissiveIntensity = 0.7; litMats.set(base, m); }
  return m;
}
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
  const defs = model.defs;                                      // compiled once per build, NOT per frame
  for (const [uid, a] of Object.entries(ps)) {
    let g = partObjs.get(uid);
    if (!g) {
      const d = defs.get(partTpl[uid]);
      if (!d) continue;
      g = new THREE.Group();
      // Shared materials: a pile of parts is the case that has to stay cheap. A held part swaps
      // to its own lit copy instead (litOf), and swaps back when it is let go.
      for (const s of d.shapes) { const m = shapeMesh(s, false); m.userData.part = uid; g.add(m); }
      partsGroup.add(g);
      partObjs.set(uid, g);
    }
    // A part in the hand glows, so it is obvious which one is being held or dragged.
    const lit = pins.has(uid);
    if (g.userData.lit !== lit) {
      g.userData.lit = lit;
      g.traverse(o => {
        if (!o.material) return;
        if (lit) { o.userData.base ??= o.material; o.material = litOf(o.userData.base); }
        else if (o.userData.base) o.material = o.userData.base;
      });
    }
    g.position.set(a[0], a[1], a[2]);
    g.quaternion.set(a[3], a[4], a[5], a[6]);
  }
}

/**
 * Where a loose part is on screen, for tests/browser.test.js. A WebGL canvas cannot be hit-tested
 * from the DOM, and sweeping the view with clicks presses the machine's own pushbuttons (measured:
 * the sweep hit STOP and the line stopped feeding). Read-only, and used by nothing else.
 */
window.mioPartScreen = uid => {
  const g = partObjs.get(uid);
  if (!g) return null;
  const v = new THREE.Vector3().setFromMatrixPosition(g.matrixWorld).project(camera);
  const r = renderer.domElement.getBoundingClientRect();
  return [r.left + (v.x + 1) / 2 * r.width, r.top + (1 - v.y) / 2 * r.height];
};

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
  const box = sceneBox();
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
let simScale = 1;                     // slow motion: sim time runs at this fraction of wall time

function onState(m) {
  if (frames.length && m.t < frames[frames.length - 1].t) frames.length = 0;   // server restarted
  if (m.full) {
    curDof = { ...m.dof }; curIo = { ...m.io }; curParts = { ...m.parts }; partTpl = { ...m.ptpl };
    pins = new Set(m.pins || []);
  } else {
    Object.assign(curDof, m.dof); Object.assign(curIo, m.io);
    if (m.parts) Object.assign(curParts, m.parts);
    if (m.ptpl) Object.assign(partTpl, m.ptpl);
    for (const uid of m.pgone || []) { delete curParts[uid]; delete partTpl[uid]; }
    if (m.pins) pins = new Set(m.pins);
  }
  if (m.forced) forced = m.forced;
  const est = m.t - performance.now() * simScale;
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

let lastFrame = performance.now();
function loop() {
  requestAnimationFrame(loop);
  const now = performance.now();
  const dt = (now - lastFrame) / 1000;
  lastFrame = now;
  if (model && offset != null) {
    const t = performance.now() * simScale + offset - RENDER_DELAY_MS * simScale;
    place(dofAt(t));
    if (!editorRef?.active) placeParts(partsAt(t));
  }
  // Walking is the camera's own business: OrbitControls is off while it runs, or the two fight
  // over the same camera and the picture shakes.
  if (pov.active) { pov.update(dt); dragFromCrosshair(now); } else controls.update();
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
  for (const it of opItems) {
    if (it.pct) {                                   // the dial shows what the PLC really reads
      const v = it.tag && curIo[it.tag];
      const txt = (typeof v === 'number' ? Math.round(v) : 100) + '%';
      if (it.el.textContent !== txt) it.el.textContent = txt;
      continue;
    }
    const on = !!(it.tag && curIo[it.tag]);
    if (it.el.classList.contains(it.cls) !== on) it.el.classList.toggle(it.cls, on);
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

// ------------------------------------------------------------------ operator panel (HTML, hideable)
// The machine's selector, pushbuttons and lamps are scene components (they own their tags), but
// they are drawn HERE, as a Denso-style panel beside the view, not in 3D. Every button sends
// edges only (down, up); the PLC enforces what each mode allows. Built once per scene; the lit
// state is updated with the IO table.
let opItems = [];
function buildOpPanel(sc) {
  const body = $('op-body');
  body.replaceChildren();
  opItems = [];
  const edge = (id, key, down) => post('/api/press', { id, key, down });
  for (const c of sc.components) {
    const t = TYPES[c.type];
    if (!t || t.group !== 'operator') continue;
    const label = c.label || c.id;
    if (c.type === 'selector') {
      const el = document.createElement('div'); el.className = 'sel'; el.title = label;
      for (const [cls, txt] of [['l', 'INDIVIDUAL'], ['r', 'AUTO']]) { const s = document.createElement('span'); s.className = cls; s.textContent = txt; el.append(s); }
      el.onpointerdown = () => edge(c.id, 'sel', true);
      el.onpointerup = () => edge(c.id, 'sel', false);
      body.append(el);
      opItems.push({ el, tag: c.io?.sel, cls: 'auto' });
    } else if (c.type === 'pushbutton') {
      const el = document.createElement('button'); el.className = 'op-btn'; el.textContent = label;
      el.style.setProperty('--c', COLORS[params(c).color] || '#888');
      let down = false;
      el.onpointerdown = () => { down = true; el.classList.add('down'); edge(c.id, 'pb', true); };
      const up = () => { if (!down) return; down = false; el.classList.remove('down'); edge(c.id, 'pb', false); };
      el.onpointerup = up; el.onpointerleave = up; el.onpointercancel = up;
      body.append(el);
      // A latching mushroom has no lamp: what it shows is its OWN state, in or out.
      const latch = params(c).kind === 'alternate';
      opItems.push({ el, tag: latch ? c.io?.pb : c.io?.lamp, cls: latch ? 'latched' : 'lit' });
    } else if (t.dialKey) {
      // A dial carries a value, not an edge. It sends on change, never while dragging: one write
      // per setting, the way a slider must behave here.
      const p = params(c), el = document.createElement('label');
      el.className = 'op-dial';
      el.title = label;
      const inp = document.createElement('input');
      inp.type = 'range'; inp.min = String(p.min ?? 1); inp.max = '100'; inp.step = '5'; inp.value = '100';
      const out = document.createElement('b');
      out.textContent = '100%';
      inp.onchange = () => { out.textContent = inp.value + '%'; post('/api/dial', { id: c.id, key: t.dialKey, value: Number(inp.value) }); };
      el.append(document.createTextNode(label), inp, out);
      body.append(el);
      opItems.push({ el: out, tag: c.io?.[t.dialKey], pct: true });
    } else if (c.type === 'lamp') {
      const el = document.createElement('span'); el.className = 'op-lamp'; el.textContent = label;
      el.style.setProperty('--c', COLORS[params(c).color] || '#888');
      body.append(el);
      opItems.push({ el, tag: c.io?.lamp, cls: 'lit' });
    }
  }
  $('oppanel').hidden = !opItems.length;
}
$('op-hide').onclick = () => { const b = $('op-body'); b.hidden = !b.hidden; $('op-hide').textContent = b.hidden ? 'show' : 'hide'; };

/** ms -> hh:mm:ss */
function hms(ms) {
  const t = Math.max(0, Math.round(ms / 1000));
  return [Math.floor(t / 3600), Math.floor(t / 60) % 60, t % 60].map(n => String(n).padStart(2, '0')).join(':');
}
const pageAt = Date.now();

function onStatus(s) {
  const io = s.io || {}, pl = s.plant || {};
  if (typeof pl.scale === 'number' && pl.scale !== simScale) {
    simScale = pl.scale;
    offset = null;                                          // the render clock changed rate: re-sync
    if ($('scale-pick').value !== String(simScale)) $('scale-pick').value = String(simScale);
  }
  const conn = $('conn');
  conn.textContent = (io.driver || '?') + ': ' + (io.msg || '');
  conn.className = io.ok ? 'ok' : 'bad';
  $('banner').hidden = io.driver !== 'internal';
  const mode = io.driver === 'internal' ? 'internal' : 'plc';
  if (!$('mode-pick').disabled && $('mode-pick').value !== mode) $('mode-pick').value = mode;
  // Run only starts the plant's clock. The machine waits for the sequence, which the PLC (or the
  // internal controller) starts from the START button: say so instead of looking frozen.
  const auto = serverScene?.cycle?.autoTag;
  // A selector on the panel (io key 'sel'): on INDIVIDUAL the individual buttons drive the actuators
  // and START is ignored, so the idle hint must say so instead of "press START".
  const selTag = serverScene && bindings(serverScene).find(b => b.key === 'sel')?.tag;
  const individual = selTag != null && curIo[selTag] === false;
  const idle = pl.mode === 'run' && auto && curIo[auto] === false && !individual;
  // Two clocks side by side say plainly whether the world is running fast or slow, and the cycle
  // time is what anyone actually watches a machine for.
  const cyc = pl.cycleMs ? (pl.cycleMs / 1000).toFixed(2) + ' s' : '--';
  const avg = pl.avgMs ? (pl.avgMs / 1000).toFixed(2) + ' s over ' + pl.cycles : '--';
  $('status').textContent = 'plant ' + pl.mode + '   sim ' + hms(pl.t) + '   wall ' + hms(Date.now() - pageAt) + '   ' + (pl.scale ?? 1) + 'x'
    + '\ncycle ' + cyc + '   avg ' + avg
    + '\nstep ' + pl.stepUs + ' µs   overruns ' + pl.overruns + (pl.behindMs ? '   behind ' + pl.behindMs + ' ms' : '') + (pl.parts != null ? '   parts ' + pl.parts : '')
    + (io.samplingMs != null ? '\nsampling ' + io.samplingMs + ' ms   publishing ' + io.publishingMs + ' ms' + (io.rttMs != null ? '   write ' + io.rttMs + ' ms' : '') : '')
    + (idle ? '\nidle: press the green START button in 3D (the sequence has not started)' : '')
    + (pl.scale != null && pl.scale !== 1 ? '\nworld speed ' + pl.scale + 'x: sim time does not match the wall clock' : '')
    + (individual ? '\nINDIVIDUAL: the panel buttons drive the actuators one at a time; turn the selector to AUTO to run the sequence' : '');
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

// Clearing the list is the viewer's own business: the plant keeps every warning in its event log
// and the recording on disk, so nothing is lost by tidying the panel.
$('warn-clear').onclick = () => $('warns').replaceChildren();

async function post(url, data) {
  try {
    const r = await fetch(url, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(data) });
    if (!r.ok) onWarn({ t: performance.now() + (offset || 0), msg: url + ': ' + ((await r.json().catch(() => ({}))).error || r.status) });
  } catch (e) { onWarn({ t: 0, msg: url + ': ' + e.message }); }
}
for (const b of document.querySelectorAll('.cmds button')) if (b.dataset.op) b.onclick = () => post('/api/cmd', { op: b.dataset.op });
// Simulation speed: on change, never while dragging. The plant decides - with a PLC it stays 1x.
$('scale-pick').onchange = () => post('/api/scale', { value: Number($('scale-pick').value) });

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
function pickNdc(x, y, list) {
  ndc.set(x, y);
  ray.setFromCamera(ndc, camera);
  return ray.intersectObjects(list, false)[0] || null;
}
function pickAt(e, list) {
  // Under pointer lock the mouse has no position on the page at all - clientX stops moving - so
  // the middle of the screen, which is where the crosshair is, is what a click means.
  if (pov.active) return pickNdc(0, 0, list);
  const r = renderer.domElement.getBoundingClientRect();
  return pickNdc((e.clientX - r.left) / r.width * 2 - 1, -(e.clientY - r.top) / r.height * 2 + 1, list);
}
let grabbed = null, lastDrag = 0;
const dragPlane = new THREE.Plane(), dragAt = new THREE.Vector3(), camDir = new THREE.Vector3();
renderer.domElement.addEventListener('pointerdown', e => {
  if (!model || e.button !== 0 || editor.active) return;       // edit mode selects instead
  const hit = pickAt(e, model.pick);
  if (hit) {
    const id = hit.object.userData.id, c = model.scene.components.find(x => x.id === id);
    pressed = { id, key: TYPES[c.type].pressKey };
    controls.enabled = false;                                   // capture phase: before OrbitControls sees it
    post('/api/press', { ...pressed, down: true });
    return;
  }
  // Nothing pressable: try a loose part. Holding one still jams the line on purpose, and dragging
  // it puts it somewhere else - out of a gripper, off a belt, back into a nest. The plant does the
  // holding; this only sends edges.
  const ph = pickAt(e, [...partObjs.values()].flatMap(g => g.children));
  if (!ph) return;
  grabbed = ph.object.userData.part;
  // Drag in the plane facing the camera through the grab point: left/right and up/down both work
  // from any orbit angle, with no mode to choose.
  dragPlane.setFromNormalAndCoplanarPoint(camera.getWorldDirection(camDir), ph.point);
  controls.enabled = false;
  post('/api/hold', { uid: grabbed, down: true });
}, true);
/** While walking, a held part follows the crosshair: look somewhere else and it goes there. */
function dragFromCrosshair(now) {
  if (!grabbed || now - lastDrag < 40) return;
  lastDrag = now;
  ndc.set(0, 0);
  ray.setFromCamera(ndc, camera);
  if (!ray.ray.intersectPlane(dragPlane, dragAt)) return;
  post('/api/hold', { uid: grabbed, down: true, at: [dragAt.x, dragAt.y, dragAt.z] });
}
renderer.domElement.addEventListener('pointermove', e => {
  if (!grabbed || pov.active) return;                           // walking drags from the crosshair instead
  const now = performance.now();
  if (now - lastDrag < 40) return;                              // the plant applies one edge per step
  lastDrag = now;
  const r = renderer.domElement.getBoundingClientRect();
  ndc.set((e.clientX - r.left) / r.width * 2 - 1, -(e.clientY - r.top) / r.height * 2 + 1);
  ray.setFromCamera(ndc, camera);
  if (!ray.ray.intersectPlane(dragPlane, dragAt)) return;
  post('/api/hold', { uid: grabbed, down: true, at: [dragAt.x, dragAt.y, dragAt.z] });
});
addEventListener('pointerup', () => {
  if (pressed) { post('/api/press', { ...pressed, down: false }); pressed = null; }
  else if (grabbed) { post('/api/hold', { uid: grabbed, down: false }); grabbed = null; }
  else return;
  controls.enabled = !pov.active;          // while walking, the orbit camera stays off
});

// ------------------------------------------------------------------ walking in (web/pov.js)
// Standing in the cell is how the height of a guard, the reach of an arm and the noise a layout
// makes are read - none of which an orbit camera two metres up and outside ever shows. The camera
// is the only thing that changes: every press and every grab goes through the same /api calls.
let solidList = [];
function refreshSolids() {
  solidList = model ? model.links.map(l => l.g) : [];
  if (workshop.visible) solidList = solidList.concat(workshop.solids);
}
const aside = document.querySelector('aside');
let panelWas = true;
const pov = createPov({
  camera, dom: renderer.domElement, controls,
  solids: () => solidList,
  onChange: on => {
    $('cross').hidden = $('pov-hud').hidden = !on;
    $('hint').hidden = on;
    $('pov-btn').textContent = on ? 'Leave' : 'Walk in';
    // The panel comes back exactly as it was: walking in is not a way to lose your layout.
    if (on) { panelWas = !aside.hidden; setPanel(false); } else setPanel(panelWas);
  },
});
$('pov-hud').textContent = `WASD walk · mouse look · Shift run · C crouch · Space jump
click: press a button, or grab a part and carry it · R back to the door · Esc out`;
$('pov-btn').onclick = () => { if (!editor.active) pov.toggle(); };
/**
 * What the view is doing, for tests/browser.test.js: walking or orbiting, where the eye is, and
 * whether the panel is up. Read-only, like window.mioPartScreen, and used by nothing else - a
 * headless browser cannot ask a WebGL canvas any of this.
 */
window.mioView = () => ({ pov: pov.active, eye: camera.position.toArray(), panel: !aside.hidden, shop: workshop.visible, solids: solidList.length });
$('shop-btn').onclick = () => setWorkshop(!workshop.visible);

// ------------------------------------------------------------------ hiding the panels
// Every section hides on its own, and the whole side panel hides with it. A hidden panel still
// has to be reachable, so the way back is a button floating over the view (and H).
function setPanel(on) {
  aside.hidden = !on;
  $('panel-show').hidden = on;
  $('panel-hide').textContent = on ? 'Hide panel' : 'Show panel';
}
$('panel-hide').onclick = () => setPanel(aside.hidden);
$('panel-show').onclick = () => setPanel(true);
for (const b of document.querySelectorAll('[data-hide]')) {
  const t = $(b.dataset.hide);
  b.onclick = () => { t.hidden = !t.hidden; b.textContent = t.hidden ? 'show' : 'hide'; };
}
// The editor hides the IO table; its heading has to go with it, or the panel grows a heading over
// nothing. The editor owns that hiding, so this follows it instead of duplicating the rule.
new MutationObserver(() => { $('io-head').hidden = editor.active; }).observe($('io-wrap'), { attributes: true, attributeFilter: ['hidden'] });

addEventListener('keydown', e => {
  if (editor.active || e.ctrlKey || e.metaKey || e.altKey || /^(INPUT|SELECT|TEXTAREA)$/.test(e.target.tagName)) return;
  const k = e.key.toLowerCase();
  if (k === 'f') pov.toggle();
  else if (k === 'h') setPanel(aside.hidden);
  else return;
  e.preventDefault();
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
// Edit mode selects with the pointer and moves things with a gizmo: a locked mouse cannot do
// either, so entering the editor leaves the walk.
$('edit-btn').addEventListener('click', () => pov.exit());
setWorkshop(true);

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
  buildOpPanel(scene);
  ioDirty = true;
});
es.addEventListener('state', e => onState(JSON.parse(e.data)));
es.addEventListener('status', e => onStatus(JSON.parse(e.data)));
es.addEventListener('warn', e => onWarn(JSON.parse(e.data)));
es.onerror = () => { $('conn').textContent = 'server not reachable - retrying'; $('conn').className = 'bad'; };
