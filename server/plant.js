// @ts-check
// The plant: Rapier world, component models, IO image, recorder. Runs headless in Node, so a
// hidden browser tab never pauses the machine a PLC is controlling.
//
// Step order (docs/PLAN.md §5), one fixed dt:
//   1. PLC outputs that arrived, panel presses, the internal controller's scan
//   2. component.step()
//   3. worldPoses() -> kinematic targets
//   4. conveyors and held parts (P3)
//   5. world.step()
//   6. part sensors (P3)
//   7. publish sensors (minPulseMs hold), record edges
// Physics never writes to the PLC: exchange() is the only path to driver.write.
import fs from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import { compile, worldPoses, tags as sceneTags, validate, stringify, partRoles } from '../lib/scene.js';
import { qmul, qaxis, qeuler, qrot, pose, compose, invert, apply, rng, hash32 } from '../lib/math.js';
import { DENSITY, luminance, DARK } from '../lib/components.js';

// Collision groups, (memberships << 16) | filter. Machine and parts collide with everything;
// part sensors cast with PART_RAYS so they see parts only, never the machine.
const G_MACHINE = (0x0001 << 16) | 0xffff, G_PART = (0x0002 << 16) | 0xffff;
export const PART_RAYS = (0xffff << 16) | 0x0002;
/** Below this a part has fallen off the machine: removed, and reported. mm */
const LOST_Z = -1000;
/** How far below an emitter the landing spot must be free: a part falls, it does not hover. mm */
const DROP_CHECK = 250;
/**
 * How fast the viewer's hand may drag a part, mm/s, unless the scene says otherwise in
 * `sim.handMmS`. A pointer can jump half a metre between two frames, and a kinematic body
 * teleported into a queue of resting parts scatters them across the hall. Held to a speed a
 * person could actually move something at, it pushes them instead: measured on a nine-part queue,
 * dragging the front one out moves the rest by 0.2 mm.
 */
const HAND_MM_S = 1200;

/** mm -> m. The ONLY unit conversion between the scene and Rapier. */
export const SK = 0.001;
/**
 * Steps one tick may run at 1x, past which the rest are dropped and counted as overruns. It is a
 * cap on SIM time, so it has to grow with the world speed: at 4x one 15 ms tick legitimately owes
 * 60 ms of plant, and a tick that slips to 30 ms owes 120 ms. Left fixed at 50 it warned
 * "plant stalled" several times a second at 2x-4x on a plant that was keeping up perfectly.
 */
const MAX_STEPS = 50;
/**
 * How far behind wall time the plant may fall before that is a STALL. Below this it catches up:
 * a hiccup on the box (a GC, the OS scheduling Sysmac Studio and Chrome ahead of Node, a timer
 * that fires late) leaves the plant owing a few hundred ms, and dropping those steps at once and
 * warning "plant stalled" is what made the simulation feel fragile - measured while jogging
 * palletizing with the PLC on the same laptop. Sim time is in ms, so this scales with the world
 * speed like MAX_STEPS does.
 */
const DEBT_MAX_MS = 500;
/**
 * Wall time one tick may spend stepping while it catches up. Windows fires the 4 ms interval
 * every ~15 ms, so 10 leaves the event loop 5 ms for HTTP, SSE and the OPC UA client; the rest
 * of the debt carries to the next tick instead of being dropped. A sustained overload (a step
 * that costs more than dt) still grows the debt until DEBT_MAX_MS says so.
 */
const CATCHUP_MS = 10;
const RING = 200000;
/** Rapier cylinders run along Y; scene cylinders run along Z. */
const Y_TO_Z = qaxis([1, 0, 0], 90);

/**
 * Belt drive, friction-clamped slip (docs/PLAN.md §3, spike A0): the velocity change one step
 * gives a part touching a belt. The part's in-plane velocity is pulled toward the belt's by at
 * most μ·g·dt, so a part blocked by a stopper slips on the belt instead of being shoved into
 * the stopper (velocity override stacked parts on top of each other, measured). m/s.
 * @param {number[]} v part velocity @param {number[]} vb belt surface velocity @param {number[]} n belt normal (unit)
 * @param {number} mu @param {number} dt @returns {number[]}
 */
export function beltDv(v, vb, n, mu, dt, g = 9.81) {
  const r = [vb[0] - v[0], vb[1] - v[1], vb[2] - v[2]];
  const k = r[0] * n[0] + r[1] * n[1] + r[2] * n[2];
  const d = [r[0] - k * n[0], r[1] - k * n[1], r[2] - k * n[2]];
  const lim = mu * g * dt * Math.max(0, n[2]), m = Math.hypot(d[0], d[1], d[2]);
  return m <= lim ? d : d.map(x => x * lim / m);
}

/**
 * The belt's friction torque: a part's spin about the belt normal is pulled toward the belt's
 * (none) by at most mu*g*dt/r, r being the part's effective friction radius (m). Without it a
 * few-millidegree landing spin persists forever on a frictionless belt surface (measured:
 * -2.3°/s, 9° of yaw after 4 s, which widens a photo-eye pulse by 9%). rad/s.
 * @param {number} wn spin about the normal @param {number} mu @param {number} dt @param {number} r
 */
export function beltSpin(wn, mu, dt, r, g = 9.81) {
  const lim = mu * g * dt / Math.max(r, 1e-3);
  return Math.max(-lim, Math.min(lim, -wn));
}

/** @type {any} */
let RAPIER = null;
export async function loadRapier() {
  if (RAPIER) return RAPIER;
  const mod = await import('@dimforge/rapier3d-deterministic-compat');
  RAPIER = mod.default ?? mod;
  await RAPIER.init();
  return RAPIER;
}

/**
 * Appends events to runs/<iso>_<scene>.ndjson. The first line is the header.
 * @param {string} dir @param {any} scene @param {string} driver
 */
export function createRecorder(dir, scene, driver) {
  fs.mkdirSync(dir, { recursive: true });
  const wall = new Date().toISOString();
  const file = path.join(dir, wall.replace(/[:.]/g, '-') + '_' + scene.name + '.ndjson');
  const out = fs.createWriteStream(file, { flags: 'a' });
  const hash = crypto.createHash('sha1').update(stringify(scene)).digest('hex').slice(0, 12);
  out.write(JSON.stringify({ k: 'header', format: 'mio-run/1', scene: scene.name, hash, driver, dtMs: scene.sim?.dtMs ?? 2, wall }) + '\n');
  return {
    file,
    write: (/** @type {any} */ ev) => { out.write(JSON.stringify(ev) + '\n'); },
    close: () => new Promise(res => out.end(res)),
  };
}

/**
 * @param {any} scene a valid scene
 * @param {{driver?: any, controller?: {scan: (io: Record<string, any>, t: number) => void}|null,
 *          recorder?: {write: (ev: any) => void}|null, clock?: () => number, wall?: () => number}} [opts]
 *   clock: ms, drives real-time pacing; wall: ms since epoch, for PLC timestamps (both injectable for tests)
 */
export async function createPlant(scene, { driver = null, controller = null, recorder = null,
                                           clock = () => performance.now(), wall = () => Date.now() } = {}) {
  const errs = validate(scene);
  if (errs.length) throw new Error('scene ' + scene?.name + ' is invalid:\n  ' + errs.join('\n  '));
  const R = await loadRapier();
  const dtMs = scene.sim?.dtMs ?? 2, dt = dtMs / 1000;
  // 20 ms: measured on the Studio 1.66 simulator (docs/SETUP.md §4). The PLC counts 10 ms
  // pulses; the hold covers the plant's own exchange tick plus the write.
  const minPulseMs = scene.io?.minPulseMs ?? 20;
  const handMmS = scene.sim?.handMmS ?? HAND_MM_S;
  const { order, defs, still } = compile(scene);
  const tagInfo = sceneTags(scene);
  const inTags = [...tagInfo].filter(([, i]) => i.dir === 'in').map(([t]) => t);
  const outTags = [...tagInfo].filter(([, i]) => i.dir === 'out').map(([t]) => t);
  const stepTags = new Map((scene.stations || []).filter((/** @type {any} */ s) => s.stepTag).map((/** @type {any} */ s) => [s.stepTag, s.id]));
  /** The scene says which tag counts finished cycles; the plant times the gaps between them. */
  const countTag = scene.cycle?.countTag ?? null;
  const avgN = Math.max(1, scene.cycle?.avgN ?? 10);
  let cycleAt = 0, cycleLast = 0;
  /** @type {number[]} */
  const cycleRing = [];
  const zero = (/** @type {string} */ tag) => (tagInfo.get(tag)?.type === 'BOOL' ? false : 0);

  // ------------------------------------------------------------ components
  const comps = order.map((/** @type {any} */ c) => {
    const d = defs.get(c.id);
    const bound = Object.entries(c.io || {}).map(([key, tag]) => [key, tag, d.io[key].dir]);
    return { id: c.id, t: d.t, p: d.p, s: d.t.init ? d.t.init(d.p) : {}, cio: /** @type {Record<string, any>} */ ({}),
             outs: bound.filter(b => b[2] === 'out'), ins: bound.filter(b => b[2] === 'in') };
  });
  const byId = new Map(comps.map(r => [r.id, r]));
  const emitters = comps.filter(r => r.t.flow === 'emitter').map(r => ({ r, rand: rng(hash32(r.id)), next: /** @type {any} */ (null), slot: 0 }));
  const removers = comps.filter(r => r.t.flow === 'remover');
  const sensors = comps.filter(r => r.t.sense);
  const holders = comps.filter(r => r.t.hold).map(r => ({ r, link: '', prev: /** @type {any} */ (null) }));
  /** Holders a gripper may take a part OUT of (a chuck, a locating pin), by id and as a list. */
  const nestHolders = holders.filter(h => h.r.t.hold === 'nest');
  const holderOf = new Map(holders.map(h => [h.r.id, h]));
  /** Metal parts (inductive proximity sees them). */
  const METAL = new Set(['steel', 'alu']);
  /** A part too dark to return light to a retro-reflective sensor (`seesDark: false`). */
  const tooDark = (/** @type {any} */ pt) => luminance(defs.get(pt.tpl).p.color) < DARK;
  /** Handshake replies the PLC provably saw (schema `hold: false`): no minPulseMs hold. */
  const noHold = new Set(order.flatMap((/** @type {any} */ c) => Object.entries(c.io || {})
    .filter(([key]) => defs.get(c.id).io[key].hold === false).map(([, tag]) => tag)));
  const dofOf = () => {
    /** @type {Record<string, number>} */
    const dof = {};
    for (const r of comps) if (typeof r.s.x === 'number') dof[r.id] = r.s.x;
    return dof;
  };

  // ------------------------------------------------------------ Rapier
  const world = new R.World({ x: 0, y: 0, z: -9.81 });
  world.timestep = dt;
  // A link moves if it has a DOF, sits on one, or its component is mounted on something moving.
  const movesMemo = new Map();
  /** @param {any} c @param {string} link @returns {boolean} */
  const linkMoves = (c, link) => {
    const key = c.id + '/' + link;
    if (movesMemo.has(key)) return movesMemo.get(key);
    const d = defs.get(c.id), l = d.links.find((/** @type {any} */ x) => x.name === link);
    let m = !!l.dof || (!!l.parent && linkMoves(c, l.parent));
    if (!m && c.parent != null) {
      const pd = defs.get(c.parent);
      m = linkMoves(pd.c, c.socket != null ? pd.sockets[c.socket].link : pd.root);
    }
    movesMemo.set(key, m);
    return m;
  };
  /** A shape (lib/components.js) as a Rapier collider, placed in its link's frame. @param {any} s */
  const colliderDesc = s => {
    let q = qeuler(s.rot), cd;
    if (s.kind === 'box') cd = R.ColliderDesc.cuboid(s.size[0] / 2 * SK, s.size[1] / 2 * SK, s.size[2] / 2 * SK);
    else if (s.kind === 'cyl') { cd = R.ColliderDesc.cylinder(s.h / 2 * SK, s.r * SK); q = qmul(q, Y_TO_Z); }
    else if (s.kind === 'sphere') cd = R.ColliderDesc.ball(s.r * SK);
    // A drawn-only kind (a mesh shell) must carry collide:false and never reach here. Falling
    // through to a ball would read s.r as undefined and make a NaN collider, which Rapier takes
    // without complaint and which then breaks contacts somewhere else entirely.
    else throw new Error('shape kind ' + s.kind + ' cannot be a collider: mark it collide:false');
    return cd.setTranslation(s.at[0] * SK, s.at[1] * SK, s.at[2] * SK).setRotation({ x: q[0], y: q[1], z: q[2], w: q[3] });
  };
  const roles = partRoles(scene);
  const W0 = worldPoses(scene, dofOf());
  /** @type {Array<{id: string, link: string, body: any, kinematic: boolean, cols: any[], last: number[]|null, moved: boolean, off: boolean}>} */
  const bodies = [];
  /** Belt surfaces: friction 0 (combine Min), so beltDv is the belt's only grip. @type {Array<{id: string, link: string, col: any}>} */
  const belts = [];
  for (const c of order) {
    if (roles.has(c.id)) continue;                          // loose parts and templates are not machine bodies
    const d = defs.get(c.id);
    for (const l of d.links) {
      const shapes = d.shapes.filter((/** @type {any} */ s) => s.link === l.name && s.collide !== false);
      if (!shapes.length) continue;
      const kinematic = linkMoves(c, l.name), P = W0[c.id][l.name];
      const desc = (kinematic ? R.RigidBodyDesc.kinematicPositionBased() : R.RigidBodyDesc.fixed())
        .setTranslation(P.p[0] * SK, P.p[1] * SK, P.p[2] * SK)
        .setRotation({ x: P.q[0], y: P.q[1], z: P.q[2], w: P.q[3] });
      const body = world.createRigidBody(desc);
      const cols = [];
      for (const s of shapes) {
        const cd = colliderDesc(s).setCollisionGroups(G_MACHINE);
        if (s.belt) cd.setFriction(0).setFrictionCombineRule(R.CoefficientCombineRule.Min);
        const col = world.createCollider(cd, body);
        cols.push(col);
        if (s.belt) belts.push({ id: c.id, link: l.name, col });
      }
      bodies.push({ id: c.id, link: l.name, body, kinematic, cols, last: /** @type {number[]|null} */ (null), moved: false, off: false });
    }
  }
  const kin = bodies.filter(b => b.kinematic);

  // ------------------------------------------------------------ loose parts
  // Dynamic Rapier bodies: CCD on, and they NEVER sleep (a sleeping part is swept through by a
  // kinematic pusher without an error; tests/rapier.test.js). `tpl` names the component whose
  // shapes and physics params the part has.
  /** @type {Map<string, {uid: string, tpl: string, body: any, cols: any[], ctr: number[], cw: number[], rf: number, held: any, rel: any, pin: any, off: boolean, parked: boolean}>} */
  const parts = new Map();
  /** collider handle -> part, for sensors that must know WHAT they see (metal or not). */
  const colPart = new Map();
  /**
   * Parts no holder and no hand has: the only ones a holder can take. A scene like the palletizing scene
   * has 100 parts of which 95 sit held in pallet pockets, and every empty nest used to scan all
   * of them every step to find the one it might take. Measured in the profile as the single
   * biggest cost in the plant after the physics itself.
   */
  const free = new Set();
  /** @param {string} tpl @param {import('../lib/math.js').Pose} P @param {string} uid @param {string} [src] */
  function spawnPart(tpl, P, uid, src) {
    const d = defs.get(tpl), pp = d.p;
    const body = world.createRigidBody(R.RigidBodyDesc.dynamic().setCanSleep(false).setCcdEnabled(true)
      .setTranslation(P.p[0] * SK, P.p[1] * SK, P.p[2] * SK).setRotation({ x: P.q[0], y: P.q[1], z: P.q[2], w: P.q[3] }));
    const cols = [];
    for (const s of d.shapes) {
      if (s.collide === false) continue;
      cols.push(world.createCollider(colliderDesc(s).setDensity(DENSITY[pp.material] ?? 1000).setFriction(pp.friction)
        .setRestitution(pp.restitution).setCollisionGroups(G_PART), body));
    }
    // ctr: the first solid shape's centre in the part's frame. The origin sits on the part's
    // underside, which rests a hair INSIDE the belt (contact penetration), so zone tests use this.
    const s0 = d.shapes.find((/** @type {any} */ s) => s.collide !== false);
    // rf: effective friction radius of the footprint (m), for the belt's friction torque
    const rf = (s0?.kind === 'box' ? Math.hypot(s0.size[0], s0.size[1]) / 4 : (s0?.r ?? 10) * 2 / 3) * SK;
    const pt = { uid, tpl, body, cols, rf, ctr: s0?.at ?? [0, 0, 0], cw: /** @type {number[]} */ ([0, 0, 0]),
                 held: /** @type {any} */ (null), rel: /** @type {any} */ (null), pin: /** @type {any} */ (null), pinTo: /** @type {any} */ (null), off: false,
                 // A part held by something that never moves is written to Rapier once and then
                 // left alone: 95 plugs standing in pallet pockets used to cost 300 allocations
                 // and 200 boundary calls a step to be told, again, exactly where they already are.
                 parked: false };
    pt.cw = partCentre(pt);
    parts.set(uid, pt);
    free.add(pt);
    for (const c of cols) colPart.set(c.handle, pt);
    rec({ t: plant.t, k: 'part', uid, ev: 'spawn', ...(src ? { src } : {}) });
  }
  /**
   * Drop a part's contacts for one step. Rapier keeps the contacts a part had while it was
   * kinematic: a part laid on a running belt by a vacuum cup then sat still on it (measured),
   * exactly like the lifted stopper. Called whenever a part's body type changes.
   * @param {any} pt
   */
  function refreshPart(pt) {
    for (const c of pt.cols) c.setEnabled(false);
    pt.off = true;
  }

  /**
   * World centre of a part in mm. It crosses into Rapier twice, so it is read ONCE a step into
   * pt.cw (below) and every remover and holder uses that: reading it per holder was 4000 boundary
   * calls a step in the palletizing scene, which is what kept it from holding 4x world speed.
   * @param {{body: any, ctr: number[]}} pt
   */
  function partCentre(pt) {
    const t = pt.body.translation(), q = pt.body.rotation();
    return apply({ p: [t.x / SK, t.y / SK, t.z / SK], q: [q.x, q.y, q.z, q.w] }, pt.ctr);
  }
  /**
   * A part a gripper's fingers could close on: free, its own, or one a NEST is holding. The last
   * is the hand-over a tending robot lives on - it grips the part in the chuck and the chuck opens
   * afterwards, never the other way round: a chuck that lets go first drops the part. The nest
   * gives it up in the holders loop below, and cannot take it back while another holder has it.
   * @param {any} r @param {any} F @param {any} z
   */
  function inZone(r, F, z) {
    const inv = invert(F);
    const mine = r.s.uid ? parts.get(r.s.uid) : null;
    const nested = nestHolders.map(h => (h.r.s.uid ? parts.get(h.r.s.uid) : null)).filter(Boolean);
    for (const pt of (mine ? [mine, ...free, ...nested] : [...free, ...nested])) {
      if (pt.pin) continue;                                   // a part the viewer is holding is not there to be taken
      if (pt.held && pt.held.id !== r.id && defs.get(pt.held.id).t.hold !== 'nest') continue;
      const l = apply(inv, pt.cw);
      if (l.every((v, i) => Math.abs(v - z.at[i]) <= z.size[i] / 2)) return pt;
    }
    return null;
  }

  /** Half-extent of a part along a world axis: exact for a box at any angle. @param {any} pt @param {number[]} ax */
  function halfAlong(pt, ax) {
    const s = defs.get(pt.tpl).shapes.find((/** @type {any} */ x) => x.collide !== false);
    if (!s) return 0;
    if (s.kind !== 'box') return s.r;
    const q = pt.body.rotation(), Q = [q.x, q.y, q.z, q.w];
    let sum = 0;
    for (let i = 0; i < 3; i++) {
      const e = [0, 0, 0];
      e[i] = s.size[i] / 2;
      const w = qrot(Q, e);
      sum += Math.abs(w[0] * ax[0] + w[1] * ax[1] + w[2] * ax[2]);
    }
    return sum;
  }

  /**
   * The part a holder would take: for a vacuum cup, the nearest free part within `reach` of its
   * suction face; for a nest, a free part whose centre is in the pocket.
   * @param {any} r @param {import('../lib/math.js').Pose} F
   */
  function candidate(r, F) {
    if (r.t.hold === 'vacuum') {
      let found = null;
      world.intersectionsWithShape({ x: F.p[0] * SK, y: F.p[1] * SK, z: F.p[2] * SK }, { x: 0, y: 0, z: 0, w: 1 }, new R.Ball(r.p.reach * SK),
        (/** @type {any} */ col) => { const pt = colPart.get(col.handle); if (pt && !pt.held && !pt.pin) { found = pt; return false; } return true; }, undefined, PART_RAYS);
      return found;
    }
    // A gripper reaches here only as the fallback in the holders loop (`cand ?? candidate(...)`),
    // and it has no `size` - its catch volume is `zone(p)`. Reading r.p.size for it threw
    // "undefined is not iterable" and killed the plant mid-run. It went unseen for as long as it
    // did because the fallback is only taken when a FREE part sits in the gripper's zone at the
    // moment it is asked, which no earlier scene managed to arrange.
    const box = r.p.size ?? (r.t.zone ? r.t.zone(r.p).size : null);
    if (!box) return null;
    const inv = invert(F), [sx, sy, sz] = box;
    // `free` is the parts no holder and no hand has, so the held/pinned test is already made.
    for (const pt of free) {
      // A pallet lift takes the CARRIER, never the load riding on it (`holdOnly` on the type).
      if (r.t.holdOnly && !defs.get(pt.tpl)?.t?.[r.t.holdOnly]) continue;
      const l = apply(inv, pt.cw);
      if (Math.abs(l[0]) <= sx / 2 && Math.abs(l[1]) <= sy / 2 && l[2] >= -sz / 2 && l[2] <= sz) return pt;
    }
    return null;
  }

  /** True when no part overlaps where a new copy of `tpl` would appear. @param {any} d @param {import('../lib/math.js').Pose} P */
  function spawnClear(d, P, dropOnto = false) {
    for (const s of d.shapes) {
      if (s.collide === false) continue;
      const c = apply(P, s.at), r = s.kind === 'box' ? Math.max(...s.size) / 2 : s.kind === 'cyl' ? Math.max(s.r, s.h / 2) : s.r;
      // The whole COLUMN under the emitter has to be free, not just the spawn height: a part
      // falls onto whatever is below. Testing only at spawn height dropped parts onto parts
      // already on the belt and built stacks (measured on buffer-queue). A feeder that MEANS to
      // stack - a lid onto a base - sets dropOnto and only needs its own spot free.
      const maxDrop = dropOnto ? 0 : DROP_CHECK;
      for (let drop = 0; drop <= maxDrop; drop += r) {
        let hit = false;
        world.intersectionsWithShape({ x: c[0] * SK, y: c[1] * SK, z: (c[2] - drop) * SK }, { x: 0, y: 0, z: 0, w: 1 }, new R.Ball(r * SK),
          () => { hit = true; return false; }, undefined, PART_RAYS);
        if (hit) return false;
      }
    }
    return true;
  }
  /** @param {string} uid @param {'remove'|'lost'|'reset'} ev */
  function removePart(uid, ev) {
    const pt = parts.get(uid);
    if (!pt) return;
    for (const c of pt.cols) colPart.delete(c.handle);
    world.removeRigidBody(pt.body);
    parts.delete(uid);
    free.delete(pt);
    rec({ t: plant.t, k: 'part', uid, ev });
  }
  function spawnSceneParts() {
    const W = worldPoses(scene, dofOf());
    for (const [id, role] of roles) if (role === 'free') spawnPart(id, W[id][defs.get(id).root], id);
  }

  // ------------------------------------------------------------ IO image
  /** What the PLC sees for `in` tags, and what the plant uses for `out` tags. */
  const io = /** @type {Record<string, any>} */ ({});
  const raw = /** @type {Record<string, any>} */ ({});        // sensor values before the pulse hold
  const plc = /** @type {Record<string, any>} */ ({});        // last value the PLC (or controller) wrote
  const forced = new Map();
  const pubAt = /** @type {Record<string, number>} */ ({});
  const stretched = new Set();
  const dirty = new Set();                                   // `in` tags to write in the next batch
  const sent = /** @type {Record<string, {v: any, at: number}>} */ ({});
  const overwritten = new Set();
  /** @type {Array<[string, any, number]>} */
  const inbox = [];
  /** @type {Array<[string, string, boolean]>} */
  const presses = [];
  /** Dial settings from the browser: [id, key, value]. Queued like presses, so a run replays. @type {Array<[string, string, number]>} */
  const dials = [];
  /** Viewer hand edges: [part uid, down, at]. Queued like presses, so a recorded run replays. @type {Array<[string, boolean, number[]|null]>} */
  const hands = [];
  const ctlIo = /** @type {Record<string, any>} */ ({});
  const events = [];
  const warned = new Set();
  /** Scratch list, reused: removing while iterating the map is what the copy was for. */
  const taken = /** @type {string[]} */ ([]);

  const plant = {
    scene, world, bodies, parts, io, dtMs, inTags, outTags,
    t: 0, mode: 'stop', overruns: 0, stepUs: 0, behindMs: 0, events, scale: 1,
    /** @type {Array<(msg: string, t: number) => void>} */
    warnListeners: [],
    dof: dofOf(),
    step, exchange, start, stop, reset, press, dial, holdPart, force, fromPlc, plcSaw, driverUp, snapshot, status, close, warn, setScale,
    /** Advance `ms` of sim time now, without pacing (tests, fast internal runs). @param {number} ms */
    run(ms) { for (let n = Math.round(ms / dtMs); n > 0; n--) step(); },
  };

  /** @param {any} ev */
  function rec(ev) {
    events.push(ev);
    if (events.length > RING) events.splice(0, RING / 10);
    recorder?.write(ev);
  }
  /** @param {string} msg */
  function warn(msg) {
    rec({ t: plant.t, k: 'warn', msg });
    for (const f of plant.warnListeners) f(msg, plant.t);
  }
  const warnOnce = (/** @type {string} */ msg) => { if (!warned.has(msg)) { warned.add(msg); warn(msg); } };

  function initIo() {
    for (const [tag] of tagInfo) { io[tag] = zero(tag); raw[tag] = zero(tag); pubAt[tag] = -Infinity; }
    for (const tag of outTags) plc[tag] = zero(tag);
    Object.assign(ctlIo, io);
  }
  initIo();
  spawnSceneParts();
  settle();

  /** @param {string} tag @param {any} v @param {number} [tp] PLC-stamped sim time */
  function applyOut(tag, v, tp) {
    plc[tag] = v;
    if (forced.has(tag) || io[tag] === v) return;
    io[tag] = v;
    if (tag === countTag && typeof v === 'number' && v > 0) {
      // A finished cycle: the gap since the last one is the cycle time, in sim milliseconds.
      if (cycleAt > 0) {
        cycleLast = plant.t - cycleAt;
        cycleRing.push(cycleLast);
        if (cycleRing.length > avgN) cycleRing.shift();
      }
      cycleAt = plant.t;
    }
    const st = stepTags.get(tag);
    const ev = st ? { t: plant.t, k: 'step', st, v } : { t: plant.t, k: 'out', tag, v };
    if (tp != null && Math.round(tp) !== plant.t) /** @type {any} */ (ev).tp = Math.round(tp);
    rec(ev);
  }

  /**
   * A sensor change becomes visible to the PLC only after the previous value was held for
   * minPulseMs. A blip shorter than one write batch would otherwise never reach the PLC. Every
   * stretch is a `warn`: never silence it.
   * @param {string} tag @param {any} v
   */
  function publish(tag, v) {
    if (v === io[tag]) return;
    if (typeof v === 'boolean' && minPulseMs > 0 && !noHold.has(tag)) {
      const held = plant.t - pubAt[tag];
      if (held < minPulseMs) {
        if (!stretched.has(tag)) { stretched.add(tag); warn('pulse stretched ' + tag + ' ' + held + ' -> ' + minPulseMs + ' ms'); }
        return;
      }
    }
    stretched.delete(tag);
    io[tag] = v;
    pubAt[tag] = plant.t;
    dirty.add(tag);
    if (typeof v === 'boolean' || tagInfo.get(tag)?.type !== 'LREAL') rec({ t: plant.t, k: 'in', tag, v });
  }

  function step() {
    plant.t += dtMs;
    // 1. commands
    for (const [tag, v, tp] of inbox.splice(0)) applyOut(tag, v, tp);
    for (const [id, key, down] of presses.splice(0)) { const r = byId.get(id); r?.t.press?.(r.s, r.p, key, down); }
    for (const [id, key, v] of dials.splice(0)) { const r = byId.get(id); r?.t.dial?.(r.s, r.p, key, v); }
    for (const [uid, down, at] of hands.splice(0)) grab(uid, down, at);
    if (controller) {
      for (const tag of inTags) ctlIo[tag] = io[tag];
      controller.scan(ctlIo, plant.t);
      for (const tag of outTags) if (ctlIo[tag] !== plc[tag]) applyOut(tag, ctlIo[tag]);
    }
    // 2. component models
    sample(dt);
    // 3. kinematic targets from THE pose function
    plant.dof = dofOf();
    const W = worldPoses(scene, plant.dof);
    for (const b of kin) {
      const P = W[b.id][b.link];
      b.body.setNextKinematicTranslation({ x: P.p[0] * SK, y: P.p[1] * SK, z: P.p[2] * SK });
      b.body.setNextKinematicRotation({ x: P.q[0], y: P.q[1], z: P.q[2], w: P.q[3] });
      // Contact refresh (measured, tests/rapier.test.js): a part that was pressed against a
      // kinematic stopper keeps a stale blocking contact after the stopper moves clear, even
      // 20 mm clear. When a kinematic link comes to rest, its colliders sit out one step, so
      // the next contacts are computed fresh from the real geometry.
      if (b.off) { for (const c of b.cols) c.setEnabled(true); b.off = false; }
      const now = [...P.p, ...P.q];
      const moving = !!b.last && now.some((v, i) => Math.abs(v - /** @type {number[]} */ (b.last)[i]) > 1e-9);
      if (b.moved && !moving) { for (const c of b.cols) c.setEnabled(false); b.off = true; }
      b.moved = moving;
      b.last = now;
    }
    // 4. conveyors: friction-clamped slip toward the belt velocity (beltDv, spike A0). A stopped
    // belt brakes parts the same way, as a real one does.
    for (const bl of belts) {
      const r = byId.get(bl.id), F = W[bl.id][bl.link], vb = r.s.v * SK;
      const u = qrot(F.q, [1, 0, 0]), n = qrot(F.q, [0, 0, 1]), vbelt = [u[0] * vb, u[1] * vb, u[2] * vb];
      for (const pt of parts.values()) {
        if (pt.held || pt.pin) continue;                      // a held part follows its holder; a pinned one stays put
        let touch = false;
        for (const pc of pt.cols) world.contactPair(bl.col, pc, (/** @type {any} */ m) => { if (m.numContacts() > 0) touch = true; });
        if (!touch) continue;
        const v = pt.body.linvel(), dv = beltDv([v.x, v.y, v.z], vbelt, n, r.p.mu, dt), m = pt.body.mass();
        pt.body.applyImpulse({ x: m * dv[0], y: m * dv[1], z: m * dv[2] }, true);
        const w = pt.body.angvel(), dw = beltSpin(w.x * n[0] + w.y * n[1] + w.z * n[2], r.p.mu, dt, pt.rf);
        pt.body.setAngvel({ x: w.x + n[0] * dw, y: w.y + n[1] * dw, z: w.z + n[2] * dw }, true);
      }
    }
    // 4b. held parts ride their holder at the pose stored when it took them. A part whose body
    // type just changed sat out one step (see the take/release below), so put it back first.
    for (const pt of parts.values()) {
      if (pt.off) { for (const c of pt.cols) c.setEnabled(true); pt.off = false; }
      // The hand (grab) pins a part to the world: it stays exactly where it was taken, so the
      // belt slips under it and the parts behind it queue up. The machine sees a jam, not a
      // teleport.
      if (pt.pin) {
        // Walk the hold toward where the pointer is, at a hand's speed. Teleporting it there
        // scatters whatever it is resting against.
        if (pt.pinTo) {
          const lim = handMmS * SK * dt;
          const d = [pt.pinTo[0] - pt.pin.p[0], pt.pinTo[1] - pt.pin.p[1], pt.pinTo[2] - pt.pin.p[2]];
          const m = Math.sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
          const k = m > lim ? lim / m : 1;
          pt.pin.p = [pt.pin.p[0] + d[0] * k, pt.pin.p[1] + d[1] * k, pt.pin.p[2] + d[2] * k];
        }
        pt.body.setNextKinematicTranslation({ x: pt.pin.p[0], y: pt.pin.p[1], z: pt.pin.p[2] });
        pt.body.setNextKinematicRotation({ x: pt.pin.q[0], y: pt.pin.q[1], z: pt.pin.q[2], w: pt.pin.q[3] });
        continue;
      }
      if (!pt.held) continue;
      if (pt.parked) continue;                 // its holder never moves: it is already there
      const P = compose(W[pt.held.id][pt.held.link], pt.rel);
      pt.body.setNextKinematicTranslation({ x: P.p[0] * SK, y: P.p[1] * SK, z: P.p[2] * SK });
      pt.body.setNextKinematicRotation({ x: P.q[0], y: P.q[1], z: P.q[2], w: P.q[3] });
      if (still.has(pt.held.id)) pt.parked = true;
    }
    // 5. physics
    world.step();
    // A part with a NaN pose, or below the floor, is gone: say so instead of streaming garbage.
    for (const pt of parts.values()) {
      const t = pt.body.translation();
      if (Number.isFinite(t.x) && Number.isFinite(t.y) && Number.isFinite(t.z) && t.z >= LOST_Z * SK) continue;
      warn('part lost ' + pt.uid + ' at ' + [t.x, t.y, t.z].map(v => Math.round(v / SK)).join(', ') + ' mm');
      removePart(pt.uid, 'lost');
    }
    // Every part's world centre for this step: the removers and the holders all read it from here.
    // A parked part sits still on something that never moves, so its centre is read once.
    for (const pt of parts.values()) if (!pt.parked) pt.cw = partCentre(pt);
    // 6a. material flow: emitters spawn at their frame once the spot is clear; removers take
    // every part whose centre is inside their box.
    for (const em of emitters) {
      const r = em.r;
      if (r.s.req <= r.s.done) continue;
      const cols = r.p.gridCols || 1, rows = r.p.gridRows || 1, slots = cols * rows;
      if (!em.next) {
        // A grid emitter fills a tray: one part per hole, going round the grid.
        const j = r.p.jitterMm, pitch = r.p.gridPitch || 0, pitchY = r.p.gridPitchY || pitch;
        const n = slots > 1 ? em.slot % slots : 0;
        const gx = slots > 1 ? (n % cols - (cols - 1) / 2) * pitch : 0;
        const gy = slots > 1 ? (Math.floor(n / cols) - (rows - 1) / 2) * pitchY : 0;
        const off = j > 0 ? [gx + (em.rand() * 2 - 1) * j, gy + (em.rand() * 2 - 1) * j, 0] : (gx || gy ? [gx, gy, 0] : undefined);
        em.next = compose(W[r.id][defs.get(r.id).root], pose(off));
      }
      if (!spawnClear(defs.get(r.p.template), em.next, r.p.dropOnto)) {
        // A tray loader moves on to the next hole rather than waiting on a full one, one hole a
        // step. A single-spot feeder just waits, as it must: its part has not gone yet.
        if (slots > 1) { em.slot = (em.slot + 1) % slots; em.next = null; }
        continue;
      }
      r.s.done++;
      spawnPart(r.p.template, em.next, r.id + '.' + r.s.done, r.id);
      if (slots > 1) em.slot = (em.slot + 1) % slots;
      em.next = null;
    }
    for (const r of removers) {
      const F = invert(W[r.id][defs.get(r.id).root]), [sx, sy, sz] = r.p.size;
      taken.length = 0;
      for (const pt of parts.values()) {
        const l = apply(F, pt.cw);
        if (Math.abs(l[0]) <= sx / 2 && Math.abs(l[1]) <= sy / 2 && l[2] >= 0 && l[2] <= sz) taken.push(pt.uid);
      }
      for (const uid of taken) { removePart(uid, 'remove'); r.s.n++; }
    }
    // 6b. part sensors: Rapier queries against PARTS only (PART_RAYS). The component model
    // turns s.hit into its output at the next step (NO/NC, off-delay).
    for (const r of sensors) {
      const F = W[r.id][defs.get(r.id).root], o = F.p, ax = qrot(F.q, [1, 0, 0]);
      let hit = false;
      if (r.t.sense === 'ray') {
        // A sensor that cannot see a dark part skips those colliders, so the beam goes straight
        // through a black workpiece the way a real retro-reflective one does. Rapier's filter
        // predicate KEEPS a collider when it returns true (measured: written the other way round,
        // the beam saw the black part and nothing else).
        const keep = r.p.seesDark === false
          ? (/** @type {any} */ col) => { const pt = colPart.get(col.handle); return !pt || !tooDark(pt); }
          : undefined;
        hit = !!world.castRay(new R.Ray({ x: o[0] * SK, y: o[1] * SK, z: o[2] * SK }, { x: ax[0], y: ax[1], z: ax[2] }),
                              r.p.range * SK, true, undefined, PART_RAYS, undefined, undefined, keep);
      } else {
        const c = apply(F, [r.p.range / 2, 0, 0]);
        world.intersectionsWithShape({ x: c[0] * SK, y: c[1] * SK, z: c[2] * SK }, { x: 0, y: 0, z: 0, w: 1 }, new R.Ball(r.p.range / 2 * SK), (/** @type {any} */ col) => {
          const pt = colPart.get(col.handle);
          if (pt && (!r.p.metalOnly || METAL.has(defs.get(pt.tpl).p.material)) && !(r.p.seesDark === false && tooDark(pt))) { hit = true; return false; }
          return true;
        }, undefined, PART_RAYS);
      }
      r.s.hit = hit;
    }
    // 6c. holding: ONE mechanism for the vacuum cup and the nest (docs/PLAN.md §3). Take = the
    // part becomes kinematic at a stored relative pose; release = dynamic again, starting at the
    // holder's velocity (rb4axis fisikaJatuhkan), so a part let go while moving flies on.
    for (const h of holders) {
      // Most holders hold at their root link. A pallet lift holds at its LIFT link (`holdLink`),
      // so the pallet it locates rises with the lift instead of staying where it was caught.
      const r = h.r, link = h.link || (h.link = r.t.holdLink || defs.get(r.id).root), F = W[r.id][link];
      if (r.s.uid && !parts.has(r.s.uid)) r.s.uid = null;             // a remover took it
      // A gripper's fingers stop at the width of the part between them, which only physics
      // knows; the model closes onto `blockAt` and decides when it has a grip.
      let cand = null;
      if (r.t.hold === 'grip') {
        cand = inZone(r, F, r.t.zone(r.p));
        if (!r.s.uid) r.s.blockAt = cand ? halfAlong(cand, qrot(F.q, [0, 1, 0])) : 0;
      }
      const want = r.t.hold === 'vacuum' ? !!r.cio.on : r.t.hold === 'grip' ? !!r.s.grip : r.cio.clamp !== false;
      if (want && !r.s.uid) {
        const pt = cand ?? candidate(r, F);
        if (pt) {
          const t = pt.body.translation(), q = pt.body.rotation();
          // A nest LOCATES the part (t.snap): it sits square on the pocket floor, whatever pose
          // it was caught in. Without this a part is frozen wherever it happened to be when its
          // centre entered the pocket - measured 11.6 mm in the air, which then made a press
          // that stops at the nominal part height squeeze it and fling it off the table.
          pt.rel = r.t.snap ? pose([0, 0, 0]) : compose(invert(F), { p: [t.x / SK, t.y / SK, t.z / SK], q: [q.x, q.y, q.z, q.w] });
          // Taken out of a nest (see inZone): that holder loses it here and now, so its `present`
          // drops on the same step - the machine must notice the part it thinks it has is gone.
          if (pt.held && pt.held.id !== r.id) { const prev = holderOf.get(pt.held.id); if (prev) prev.r.s.uid = null; }
          pt.held = { id: r.id, link };
          pt.parked = false;
          free.delete(pt);
          pt.body.setBodyType(R.RigidBodyType.KinematicPositionBased, true);
          refreshPart(pt);
          r.s.uid = pt.uid;
          rec({ t: plant.t, k: 'part', uid: pt.uid, ev: 'hold', by: r.id });
        }
      } else if (!want && r.s.uid) {
        const pt = parts.get(r.s.uid);
        r.s.uid = null;
        if (pt) {
          pt.held = null;
          pt.parked = false;
          free.add(pt);
          pt.body.setBodyType(R.RigidBodyType.Dynamic, true);
          refreshPart(pt);
          const v = h.prev ? F.p.map((x, i) => (x - h.prev.p[i]) / dt * SK) : [0, 0, 0];
          pt.body.setLinvel({ x: v[0], y: v[1], z: v[2] }, true);
          rec({ t: plant.t, k: 'part', uid: pt.uid, ev: 'release', by: r.id });
        }
      }
      h.prev = F;
    }
    // 7. sensors -> PLC image
    publishAll();
  }

  /** Every component model for one interval, PLC outputs in, sensor values out (raw). @param {number} dt s */
  function sample(dt) {
    for (const r of comps) {
      if (!r.t.step) continue;
      for (const [key, tag] of r.outs) r.cio[key] = io[tag];
      r.t.step(r.s, r.p, r.cio, dt);
      for (const [key, tag] of r.ins) raw[tag] = r.cio[key];
    }
  }
  function publishAll() { for (const tag of inTags) publish(tag, forced.has(tag) ? forced.get(tag) : raw[tag]); }
  /**
   * The IO image starts from what the components really say, not from zeros: a selector on AUTO,
   * a cylinder's retracted switch on. Otherwise the first scan (and the first scan after Reset)
   * sees every sensor off for one step and a phantom edge on the next. Measured: the a-to-b
   * controller saw the selector "turn to AUTO" on the same scan as START and tripped to FAULT.
   */
  function settle() { sample(0); publishAll(); }

  /** The ONE path from the plant to the PLC: one batch, at most one in flight. */
  let inFlight = false;
  function exchange() {
    if (!driver || inFlight || !dirty.size || driver.ready === false) return;   // driverUp() resends all on connect
    const batch = [...dirty].map(tag => ({ name: tag, value: io[tag] }));
    dirty.clear();
    inFlight = true;
    return driver.write(batch).then(() => {
      const at = wall();
      for (const b of batch) sent[b.name] = { v: b.value, at };
    }, (/** @type {any} */ e) => {
      for (const b of batch) dirty.add(b.name);
      warnOnce('write failed: ' + String(e.message || e).split('\n')[0]);
    }).finally(() => { inFlight = false; });
  }

  /** PLC output sample from the driver. srcWall: the sample's PLC source timestamp. @param {string} tag @param {any} v @param {number} [srcWall] */
  function fromPlc(tag, v, srcWall) {
    if (!tagInfo.has(tag) || tagInfo.get(tag)?.dir !== 'out') return;
    inbox.push([tag, v, srcWall == null ? plant.t : plant.t - Math.max(0, wall() - srcWall)]);
  }

  /**
   * The plant also watches the tags it writes. If the PLC shows another value well after our
   * write landed, something in the PLC program writes the tag too (a coil on a sensor tag).
   * @param {string} tag @param {any} v @param {number} [publishingMs]
   */
  function plcSaw(tag, v, publishingMs = 50) {
    const s = sent[tag];
    if (!s || inFlight || dirty.has(tag)) return;
    const same = typeof v === 'number' ? Math.abs(v - s.v) < 1e-9 : v === s.v;
    if (same) { overwritten.delete(tag); return; }
    if (wall() - s.at > 3 * publishingMs && !overwritten.has(tag)) {
      overwritten.add(tag);
      warn(tag + ' overwritten by the PLC (coil on this tag?): plant wrote ' + JSON.stringify(s.v) + ', PLC holds ' + JSON.stringify(v));
    }
  }

  /** After (re)connecting, every sensor is written once so the PLC starts from the plant's state. */
  function driverUp() { for (const tag of inTags) dirty.add(tag); }

  /**
   * The hand: a viewer holds a part still where it is, to see what the machine does about it
   * ("ijiwaru" testing - jam the line on purpose). It is the SAME take/follow/release mechanism
   * as a vacuum cup or a nest, with the world as the holder, so nothing new can go wrong in the
   * solver. The hand never steals a part a machine already holds, and it releases with zero
   * velocity: a part let go over a running belt is carried away, not thrown.
   * @param {string} uid @param {boolean} down
   */
  function grab(uid, down, at) {
    const pt = parts.get(uid);
    if (!pt) return;
    if (down && pt.pin) {                                          // a drag: the same hold, moved
      if (at) pt.pinTo = [at[0] * SK, at[1] * SK, at[2] * SK];
      return;
    }
    if (!!pt.pin === down) return;
    if (down) {
      // Taking a part OUT of a gripper, a cup or a nest is the point of the exercise: the machine
      // then runs its cycle with nothing in its hand, and the PLC has to notice. The holder loses
      // the part here, so its own switch (vacuum, nest present) goes false at the next step.
      if (pt.held) {
        const h = byId.get(pt.held.id);
        if (h) h.s.uid = null;
        pt.held = null;
      }
      pt.parked = false;
      const t = pt.body.translation(), q = pt.body.rotation();
      pt.pin = { p: [t.x, t.y, t.z], q: [q.x, q.y, q.z, q.w] };     // metres: written straight back to Rapier
      pt.pinTo = null;
      free.delete(pt);
      pt.body.setBodyType(R.RigidBodyType.KinematicPositionBased, true);
    } else {
      pt.pin = null;
      pt.pinTo = null;
      free.add(pt);
      pt.body.setBodyType(R.RigidBodyType.Dynamic, true);
      pt.body.setLinvel({ x: 0, y: 0, z: 0 }, true);
      pt.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
    }
    refreshPart(pt);                                               // the body type changed: drop stale contacts
    rec({ t: plant.t, k: 'part', uid, ev: down ? 'hold' : 'release', by: 'hand' });
  }
  /**
   * Browser edge for the hand, applied at the start of the next step. `at` (mm, world) moves a
   * part that is already held: dragging is the same hold, put somewhere else.
   * @param {string} uid @param {boolean} down @param {number[]} [at]
   */
  function holdPart(uid, down, at) { hands.push([uid, !!down, at && at.length === 3 ? at.map(Number) : null]); }

  /** Browser edge from a 3D panel part. Applied at the start of the next step. @param {string} id @param {string} key @param {boolean} down */
  function press(id, key, down) {
    if (!byId.get(id)?.t.press) throw new Error('not pressable: ' + id);
    presses.push([id, key, !!down]);
  }

  /**
   * A panel dial carries a VALUE, not an edge: the speed override is a percentage the operator
   * sets. Applied at the start of the next step, like a press.
   * @param {string} id @param {string} key @param {number} value
   */
  function dial(id, key, value) {
    if (!byId.get(id)?.t.dial) throw new Error('not a dial: ' + id);
    if (!Number.isFinite(Number(value))) throw new Error('dial value must be a number');
    dials.push([id, key, Number(value)]);
    rec({ t: plant.t, k: 'dial', id, key, v: Number(value) });
  }

  /**
   * Forcing acts on the IO image, so the PLC sees it too. null releases.
   * @param {string} tag @param {any} value
   */
  function force(tag, value) {
    if (!tagInfo.has(tag)) throw new Error('unknown tag ' + tag);
    if (value == null) forced.delete(tag); else forced.set(tag, value);
    rec({ t: plant.t, k: 'force', tag, v: value ?? null });
    if (tagInfo.get(tag)?.dir === 'out') {
      const v = value ?? plc[tag];
      if (io[tag] !== v) { io[tag] = v; rec({ t: plant.t, k: 'out', tag, v }); }
    }
  }

  function reset() {
    for (const r of comps) r.s = r.t.init ? r.t.init(r.p) : {};
    forced.clear();
    stretched.clear();
    initIo();
    cycleAt = 0; cycleLast = 0; cycleRing.length = 0;
    hands.length = 0;                                          // a hand edge for a part that is about to go
    dials.length = 0;
    for (const uid of [...parts.keys()]) removePart(uid, 'reset');
    spawnSceneParts();
    settle();
    // The internal controller keeps its own copies of plant counters (the last emitter count it
    // saw). Reset zeroes the plant's, so a controller that kept the old ones waits for a part
    // that already "arrived" and the sequence stalls. A real PLC cannot be reset from here:
    // restart its program (docs/SETUP.md).
    controller?.reset?.();
    driverUp();
    rec({ t: plant.t, k: 'mark', label: 'reset' });
  }

  // ------------------------------------------------------------ real-time pacing
  /** @type {any} */
  let timer = null, last = 0, acc = 0, firstTick = false;
  /**
   * World speed: sim time advances at this multiple of wall time. Below 1 it is slow motion for
   * watching a fast machine; above it the plant runs ahead, which costs CPU in proportion - the
   * step budget is what stops it, and an overrun says so. It is a SIMULATOR control, not a machine
   * one: the speed override on the panel is the machine's own. Forced to 1 whenever a PLC is
   * connected, because Sysmac timers run on wall time (docs/PLAN.md §5).
   * @param {number} v 0.05 .. 4 @returns {number} the scale actually in force
   */
  function setScale(v) {
    const want = Math.min(4, Math.max(0.05, Number(v) || 1));
    if (driver && want !== 1) {
      warnOnce('time scale stays 1x while a PLC is connected: its timers run on wall time');
      plant.scale = 1;
      return plant.scale;
    }
    if (want !== plant.scale) { plant.scale = want; rec({ t: plant.t, k: 'scale', v: want }); }
    return plant.scale;
  }

  function tick() {
    const now = clock();
    // The gap before the FIRST tick is not the plant falling behind: it is everything that
    // happened between start() and the event loop getting round to the timer, and on this PC that
    // is a major GC right after setup - measured at 240-440 ms on every scene, always exactly one.
    // Counting it left a permanent "overruns 171" on the status line of a plant that then held
    // 99% of real time for the rest of the run.
    if (firstTick) { firstTick = false; last = now; }
    acc += (now - last) * plant.scale;
    last = now;
    let n = Math.floor(acc / dtMs);
    const k = Math.max(1, plant.scale), cap = Math.round(MAX_STEPS * k);
    const t0 = performance.now();
    if (acc > DEBT_MAX_MS * k) {
      // A real stall: the plant owes more than it could sensibly catch up. Sim time falls behind
      // wall time here. Say how long, so it can be traced to what blocked the event loop (a
      // synchronous require, a browse, GC) or to steps that cost more than dt (see stepUs).
      warn('plant stalled ' + Math.round(n * dtMs) + ' ms: ' + (n - cap) + ' steps dropped (overruns)'
        + (plant.scale !== 1 ? ' at ' + plant.scale + 'x' : '') + ', step ' + plant.stepUs + ' us vs dt ' + dtMs + ' ms');
      plant.overruns += n - cap; n = cap; acc = 0;
      for (let i = 0; i < n; i++) step();
    } else {
      // Behind by less than that: CATCH UP. Run what is owed within this tick's wall budget and
      // carry the rest to the next tick - nothing is dropped and nothing warns. A normal tick owes
      // a handful of steps and never touches the budget; a hiccup owes a hundred and repays them
      // over the next few ticks, sim time briefly running ahead of the clock.
      let ran = 0;
      while (ran < n) { step(); ran++; if (performance.now() - t0 > CATCHUP_MS) break; }
      acc -= ran * dtMs; n = ran;
    }
    plant.behindMs = Math.round(acc / k);
    if (n) plant.stepUs = Math.round(plant.stepUs * 0.9 + (performance.now() - t0) / n * 1000 * 0.1);
    exchange();
  }
  function start() {
    if (timer) return;
    last = clock(); acc = 0; firstTick = true;
    timer = setInterval(tick, 4);             // Windows fires this every ~15 ms; the accumulator catches up
    plant.mode = 'run';
    rec({ t: plant.t, k: 'mark', label: 'run' });
  }
  function stop() {
    if (!timer) return;
    clearInterval(timer); timer = null;
    plant.mode = 'stop';
    rec({ t: plant.t, k: 'mark', label: 'stop' });
  }

  function snapshot() {
    /** @type {Record<string, number[]>} */
    const ps = {}, tpl = /** @type {Record<string, string>} */ ({});
    /** Parts the viewer's hand is holding: the viewer highlights them. @type {string[]} */
    const pins = [];
    for (const pt of parts.values()) {
      const t = pt.body.translation(), q = pt.body.rotation();
      ps[pt.uid] = [t.x / SK, t.y / SK, t.z / SK, q.x, q.y, q.z, q.w];
      tpl[pt.uid] = pt.tpl;
      if (pt.pin) pins.push(pt.uid);
    }
    return { t: plant.t, dof: plant.dof, io: { ...io }, forced: Object.fromEntries(forced), parts: ps, ptpl: tpl, pins };
  }
  function status() {
    return {
      io: driver ? driver.status() : { driver: 'internal', ok: true, msg: 'INTERNAL CONTROLLER - not a PLC', heartbeat: null },
      plant: { mode: plant.mode, t: plant.t, overruns: plant.overruns, stepUs: plant.stepUs, behindMs: plant.behindMs, parts: parts.size, scale: plant.scale,
               cycleMs: cycleLast, avgMs: cycleRing.length ? Math.round(cycleRing.reduce((a, b) => a + b, 0) / cycleRing.length) : 0,
               cycles: cycleRing.length, cycleTag: countTag },
    };
  }
  async function close() {
    stop();
    await driver?.close?.();
    world.free();
  }

  return plant;
}
