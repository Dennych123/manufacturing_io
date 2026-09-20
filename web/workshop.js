// The hall the machine stands in: an SPM builder's workshop, drawn around whatever scene is
// loaded. It is SCENERY - the plant knows nothing about it, no collider is ever built from it and
// no sensor can see it. It exists because a machine floating in an empty grey void gives the eye
// nothing to judge size, height or distance against, and because that is what these cells look
// like on the build floor before they are shipped.
//
// The only thing it is asked for besides a picture is `solids`: the walls and the furniture the
// walk-in camera (web/pov.js) may not stroll through. That is a viewer convenience, not physics.
import * as THREE from 'three';

const M = {
  floor:  { color: '#8d9194', roughness: 0.95, metalness: 0.0 },
  line:   { color: '#d8b400', roughness: 0.8, metalness: 0.0 },
  dado:   { color: '#55636f', roughness: 0.9, metalness: 0.0 },
  wall:   { color: '#cdd2d6', roughness: 0.95, metalness: 0.0 },
  roof:   { color: '#3a4046', roughness: 0.9, metalness: 0.1 },
  beam:   { color: '#6e7580', roughness: 0.6, metalness: 0.4 },
  steel:  { color: '#9aa1a8', roughness: 0.45, metalness: 0.6 },
  crane:  { color: '#e2b100', roughness: 0.6, metalness: 0.3 },
  wood:   { color: '#ad7f48', roughness: 0.85, metalness: 0.0 },
  crate:  { color: '#b7a079', roughness: 0.9, metalness: 0.0 },
  rack:   { color: '#c2611c', roughness: 0.7, metalness: 0.2 },
  drum:   { color: '#2f5fa0', roughness: 0.6, metalness: 0.3 },
  box:    { color: '#b23b2e', roughness: 0.6, metalness: 0.2 },
  dark:   { color: '#2f3338', roughness: 0.8, metalness: 0.1 },
  glass:  { color: '#a9c6d8', roughness: 0.15, metalness: 0.0, transparent: true, opacity: 0.28 },
  panel:  { color: '#fff4d8', roughness: 0.9, metalness: 0.0, emissive: '#fff0c8', emissiveIntensity: 0.9 },
};
const mats = new Map();
function mat(name) {
  let m = mats.get(name);
  if (!m) {
    const d = M[name];
    m = new THREE.MeshStandardMaterial({ color: d.color, roughness: d.roughness, metalness: d.metalness,
      ...(d.transparent ? { transparent: true, opacity: d.opacity } : {}),
      ...(d.emissive ? { emissive: new THREE.Color(d.emissive), emissiveIntensity: d.emissiveIntensity } : {}) });
    mats.set(name, m);
  }
  return m;
}

// Geometries are shared by size, so a rack of forty identical crates is forty draw calls over one
// buffer, and a rebuild disposes nothing that another mesh is still using.
const geos = new Map();
function geo(w, d, h) {
  const k = w + ',' + d + ',' + h;
  let g = geos.get(k);
  if (!g) geos.set(k, g = new THREE.BoxGeometry(w, d, h));
  return g;
}
function cyl(r, h) {
  const k = 'c' + r + ',' + h;
  let g = geos.get(k);
  if (!g) { g = new THREE.CylinderGeometry(r, r, h, 20); g.rotateX(Math.PI / 2); geos.set(k, g); }
  return g;
}

/** A box standing ON z: x,y are its centre, z is its underside. */
function slab(parent, name, w, d, h, x, y, z, yaw = 0) {
  const m = new THREE.Mesh(geo(w, d, h), mat(name));
  m.position.set(x, y, z + h / 2);
  if (yaw) m.rotation.z = yaw;
  m.receiveShadow = true;
  parent.add(m);
  return m;
}
function post(parent, name, r, h, x, y, z) {
  const m = new THREE.Mesh(cyl(r, h), mat(name));
  m.position.set(x, y, z + h / 2);
  parent.add(m);
  return m;
}

/**
 * The workshop around a machine. `fit(box)` sizes the hall to the scene's bounding box and puts
 * the furniture against the walls, so nothing it draws can ever stand in the machine's way.
 */
export function createWorkshop(scene3) {
  const group = new THREE.Group();
  group.name = 'workshop';
  scene3.add(group);
  const solids = [];                       // walls and furniture the walk-in camera stops at
  let last = '';

  function clear() {
    group.clear();                         // geometries and materials are shared: never disposed here
    solids.length = 0;
  }

  function fit(box) {
    if (box.isEmpty()) return;
    const c = box.getCenter(new THREE.Vector3()), s = box.getSize(new THREE.Vector3());
    // Round the hall to whole metres: a scene that grows by a millimetre must not rebuild the
    // whole shop, and the rebuild is what a scene switch does anyway.
    const W = Math.max(16, Math.ceil((s.x + 14000) / 1000)) * 1000;
    const D = Math.max(16, Math.ceil((s.y + 14000) / 1000)) * 1000;
    const key = [W, D, Math.round(c.x / 500), Math.round(c.y / 500)].join('/');
    if (key === last) return;
    last = key;
    clear();
    build(W, D, Math.round(c.x / 500) * 500, Math.round(c.y / 500) * 500);
  }

  function build(W, D, cx, cy) {
    const H = 7000, T = 200;               // eaves height, wall thickness
    const x0 = cx - W / 2, x1 = cx + W / 2, y0 = cy - D / 2, y1 = cy + D / 2;
    const walls = new THREE.Group(), stuff = new THREE.Group();
    group.add(walls, stuff);

    // ---- floor: one slab of power-floated concrete, with the aisle marked out in epoxy.
    const f = new THREE.Mesh(new THREE.PlaneGeometry(W, D), mat('floor'));
    f.position.set(cx, cy, 0);
    f.receiveShadow = true;
    group.add(f);
    for (const sx of [-1, 1]) slab(group, 'line', W - 2 * T, 120, 2, cx, cy + sx * (D / 2 - 2200), 0);
    for (const sy of [-1, 1]) slab(group, 'line', 120, D - 4600, 2, cx + sy * (W / 2 - 2200), cy, 0);

    // ---- walls: a painted dado to 1200, block above it, a window band under the eaves. The
    // dado is not decoration - it is the height everything in the shop is judged against.
    const band = [[1200, 'dado', 0], [2600, 'wall', 1200], [1000, 'glass', 3800], [H - 4800, 'wall', 4800]];
    for (const [h, name, z] of band) {
      slab(walls, name, W + 2 * T, T, h, cx, y0 - T / 2, z);
      slab(walls, name, W + 2 * T, T, h, cx, y1 + T / 2, z);
      slab(walls, name, T, D, h, x0 - T / 2, cy, z);
      slab(walls, name, T, D, h, x1 + T / 2, cy, z);
    }
    // Mullions, or the glass band reads as a gap in the building.
    for (let x = x0 + 2000; x < x1; x += 2000) for (const y of [y0 - T / 2, y1 + T / 2]) slab(walls, 'steel', 90, T + 20, 1000, x, y, 3800);
    for (let y = y0 + 2000; y < y1; y += 2000) for (const x of [x0 - T / 2, x1 + T / 2]) slab(walls, 'steel', T + 20, 90, 1000, x, y, 3800);

    // ---- the roller shutter the finished machine leaves through, and the personnel door beside it.
    slab(walls, 'steel', 4400, 120, 4600, cx + W / 2 - 4000, y0 - T, 0);
    for (let z = 200; z < 4600; z += 400) slab(walls, 'dark', 4400, 160, 60, cx + W / 2 - 4000, y0 - T, z);
    slab(walls, 'dark', 1000, 120, 2100, cx + W / 2 - 6200, y0 - T, 0);

    // ---- roof: deck, trusses across the short span, purlins, and the skylights that light the
    // shop. The bay lights hang between them.
    slab(group, 'roof', W + 2 * T, D + 2 * T, 150, cx, cy, H);
    for (let y = y0 + 3000; y < y1; y += 6000) {
      slab(group, 'beam', W, 200, 500, cx, y, H - 600);
      for (const z of [H - 600, H - 200]) slab(group, 'beam', W, 320, 60, cx, y, z);
      for (let x = x0 + 1500; x < x1; x += 3000) {
        slab(group, 'panel', 2000, 1400, 40, x, y + 3000, H - 60);                    // skylight
        slab(group, 'panel', 1400, 200, 90, x, y, H - 900);                           // bay light
        slab(group, 'dark', 120, 120, 300, x, y, H - 1200);
      }
    }
    for (let x = x0 + 2000; x < x1; x += 4000) slab(group, 'beam', 150, D, 150, x, cy, H - 200);

    // ---- overhead travelling crane: two runway beams on brackets, a bridge and a hoist. Every
    // shop that builds special machines has one, and it is what says how high the hall is.
    const ry = [cy - D / 2 + 3000, cy + D / 2 - 3000];
    for (const y of ry) {
      slab(group, 'beam', W, 300, 700, cx, y, 5200);
      for (let x = x0 + 4000; x < x1; x += 8000) slab(group, 'beam', 400, 400, 5200, x, y, 0);
    }
    const bridge = cx - W / 2 + Math.min(W * 0.62, W - 4000);
    slab(group, 'crane', 900, ry[1] - ry[0] + 600, 500, bridge, cy, 5900);
    slab(group, 'dark', 700, 700, 600, bridge, cy + 1800, 5300);
    post(group, 'dark', 40, 2600, bridge, cy + 1800, 2700);
    slab(group, 'steel', 300, 160, 400, bridge, cy + 1800, 2400);

    // ---- the shop itself, all of it against a wall: benches and a pegboard, pallet racking with
    // stock, a cantilever rack of extrusion, drums, crates and the welding bay.
    const bx = x0 + 1400;
    for (let i = 0; i < 4; i++) {
      const y = cy - 3600 + i * 2400;
      slab(stuff, 'wood', 800, 2000, 50, bx, y, 850);
      for (const [dx, dy] of [[-320, -880], [320, -880], [-320, 880], [320, 880]]) slab(stuff, 'steel', 60, 60, 850, bx + dx, y + dy, 0);
      slab(stuff, 'dark', 700, 1900, 600, bx, y, 200);                                 // drawer unit under the bench
      slab(stuff, 'steel', 180, 220, 200, bx + 150, y - 700, 900);                     // vice
      slab(stuff, 'dark', 60, 2000, 1400, x0 + 900, y, 1300);                          // pegboard
      for (let k = 0; k < 7; k++) slab(stuff, 'steel', 30, 60, 260 + (k % 3) * 90, x0 + 940, y - 800 + k * 260, 1500);
    }
    // pallet racking down the far wall
    const ry0 = cy - 4500;
    for (let bay = 0; bay < 4; bay++) {
      const y = ry0 + bay * 2800;
      for (const dy of [-1300, 1300]) for (const dx of [-500, 500]) slab(stuff, 'rack', 90, 90, 5000, x1 - 1200 + dx, y + dy, 0);
      for (const z of [1400, 2900, 4300]) {
        for (const dx of [-500, 500]) slab(stuff, 'rack', 100, 2600, 120, x1 - 1200 + dx, y, z);
        slab(stuff, 'wood', 1200, 1000, 120, x1 - 1200, y - 600, z + 120);
        slab(stuff, 'crate', 1000, 800, 700, x1 - 1200, y - 600, z + 240);
        slab(stuff, 'wood', 1200, 1000, 120, x1 - 1200, y + 600, z + 120);
        if ((bay + z) % 2) slab(stuff, 'box', 900, 700, 500, x1 - 1200, y + 600, z + 240);
      }
    }
    // cantilever rack of aluminium extrusion and steel bar: the stock an SPM is cut from
    const sx = cx + W / 2 - 1500;
    for (let k = 0; k < 3; k++) slab(stuff, 'rack', 200, 200, 2600, sx, cy + 5200 + k * 2600, 0);
    for (const z of [900, 1700, 2500]) {
      slab(stuff, 'rack', 1100, 200, 100, sx - 500, cy + 5200, z);
      slab(stuff, 'rack', 1100, 200, 100, sx - 500, cy + 10400, z);
      for (let k = 0; k < 4; k++) slab(stuff, 'steel', 90, 5600, 90, sx - 900 + k * 220, cy + 7800, z + 100);
    }
    // welding bay, drums, gas bottles, a bin and a stack of empty pallets
    slab(stuff, 'dark', 1600, 900, 900, x0 + 2600, y1 - 1800, 0);
    slab(stuff, 'steel', 1700, 1000, 60, x0 + 2600, y1 - 1800, 900);
    slab(stuff, 'dark', 700, 700, 1000, x0 + 4200, y1 - 1800, 0);
    for (let k = 0; k < 3; k++) post(stuff, 'steel', 120, 1400, x0 + 5300 + k * 320, y1 - 1500, 0);
    for (let k = 0; k < 4; k++) post(stuff, 'drum', 290, 880, x0 + 7000 + k * 700, y1 - 1200, 0);
    slab(stuff, 'box', 1200, 1000, 1000, x0 + 9600, y1 - 1300, 0);
    for (let k = 0; k < 5; k++) slab(stuff, 'wood', 1200, 800, 140, x1 - 3600, y1 - 1400, k * 145);

    for (const w of walls.children) solids.push(w);
    for (const o of stuff.children) { o.castShadow = false; solids.push(o); }
    solids.push(f);
  }

  return {
    group, solids, fit,
    set visible(v) { group.visible = v; },
    get visible() { return group.visible; },
  };
}
