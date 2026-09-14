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
import { DENSITY } from '../lib/components.js';

// Collision groups, (memberships << 16) | filter. Machine and parts collide with everything;
// part sensors cast with PART_RAYS so they see parts only, never the machine.
const G_MACHINE = (0x0001 << 16) | 0xffff, G_PART = (0x0002 << 16) | 0xffff;
export const PART_RAYS = (0xffff << 16) | 0x0002;
/** Below this a part has fallen off the machine: removed, and reported. mm */
const LOST_Z = -1000;
/** How far below an emitter the landing spot must be free: a part falls, it does not hover. mm */
const DROP_CHECK = 250;

/** mm -> m. The ONLY unit conversion between the scene and Rapier. */
export const SK = 0.001;
const MAX_STEPS = 50;             // per tick; past it the step is dropped and counted as an overrun
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
  const { order, defs } = compile(scene);
  const tagInfo = sceneTags(scene);
  const inTags = [...tagInfo].filter(([, i]) => i.dir === 'in').map(([t]) => t);
  const outTags = [...tagInfo].filter(([, i]) => i.dir === 'out').map(([t]) => t);
  const stepTags = new Map((scene.stations || []).filter((/** @type {any} */ s) => s.stepTag).map((/** @type {any} */ s) => [s.stepTag, s.id]));
  const zero = (/** @type {string} */ tag) => (tagInfo.get(tag)?.type === 'BOOL' ? false : 0);

  // ------------------------------------------------------------ components
  const comps = order.map((/** @type {any} */ c) => {
    const d = defs.get(c.id);
    const bound = Object.entries(c.io || {}).map(([key, tag]) => [key, tag, d.io[key].dir]);
    return { id: c.id, t: d.t, p: d.p, s: d.t.init ? d.t.init(d.p) : {}, cio: /** @type {Record<string, any>} */ ({}),
             outs: bound.filter(b => b[2] === 'out'), ins: bound.filter(b => b[2] === 'in') };
  });
  const byId = new Map(comps.map(r => [r.id, r]));
  const emitters = comps.filter(r => r.t.flow === 'emitter').map(r => ({ r, rand: rng(hash32(r.id)), next: /** @type {any} */ (null) }));
  const removers = comps.filter(r => r.t.flow === 'remover');
  const sensors = comps.filter(r => r.t.sense);
  const holders = comps.filter(r => r.t.hold).map(r => ({ r, link: '', prev: /** @type {any} */ (null) }));
  /** Metal parts (inductive proximity sees them). */
  const METAL = new Set(['steel', 'alu']);
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
    else cd = R.ColliderDesc.ball(s.r * SK);
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
  /** @type {Map<string, {uid: string, tpl: string, body: any, cols: any[], ctr: number[], rf: number, held: any, rel: any, pin: any, off: boolean}>} */
  const parts = new Map();
  /** collider handle -> part, for sensors that must know WHAT they see (metal or not). */
  const colPart = new Map();
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
    const pt = { uid, tpl, body, cols, rf, ctr: s0?.at ?? [0, 0, 0], held: /** @type {any} */ (null), rel: /** @type {any} */ (null), pin: /** @type {any} */ (null), off: false };
    parts.set(uid, pt);
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

  /** World centre of a part in mm. @param {{body: any, ctr: number[]}} pt */
  function partCentre(pt) {
    const t = pt.body.translation(), q = pt.body.rotation();
    return apply({ p: [t.x / SK, t.y / SK, t.z / SK], q: [q.x, q.y, q.z, q.w] }, pt.ctr);
  }
  /** A free part (or this holder's own) whose centre is inside `z`, a box in the holder's frame. @param {any} r @param {any} F @param {any} z */
  function inZone(r, F, z) {
    const inv = invert(F);
    for (const pt of parts.values()) {
      if (pt.pin) continue;                                   // a part the viewer is holding is not there to be taken
      if (pt.held && pt.held.id !== r.id) continue;
      const l = apply(inv, partCentre(pt));
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
    const inv = invert(F), [sx, sy, sz] = r.p.size;
    for (const pt of parts.values()) {
      if (pt.held || pt.pin) continue;
      // A pallet lift takes the CARRIER, never the load riding on it (`holdOnly` on the type).
      if (r.t.holdOnly && !defs.get(pt.tpl)?.t?.[r.t.holdOnly]) continue;
      const l = apply(inv, partCentre(pt));
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
  /** Viewer hand edges: [part uid, down, at]. Queued like presses, so a recorded run replays. @type {Array<[string, boolean, number[]|null]>} */
  const hands = [];
  const ctlIo = /** @type {Record<string, any>} */ ({});
  const events = [];
  const warned = new Set();

  const plant = {
    scene, world, bodies, parts, io, dtMs, inTags, outTags,
    t: 0, mode: 'stop', overruns: 0, stepUs: 0, events,
    /** @type {Array<(msg: string, t: number) => void>} */
    warnListeners: [],
    dof: dofOf(),
    step, exchange, start, stop, reset, press, holdPart, force, fromPlc, plcSaw, driverUp, snapshot, status, close, warn,
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

  /** @param {string} tag @param {any} v @param {number} [tp] PLC-stamped sim time */
  function applyOut(tag, v, tp) {
    plc[tag] = v;
    if (forced.has(tag) || io[tag] === v) return;
    io[tag] = v;
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
    for (const [uid, down, at] of hands.splice(0)) grab(uid, down, at);
    if (controller) {
      for (const tag of inTags) ctlIo[tag] = io[tag];
      controller.scan(ctlIo, plant.t);
      for (const tag of outTags) if (ctlIo[tag] !== plc[tag]) applyOut(tag, ctlIo[tag]);
    }
    // 2. component models
    for (const r of comps) {
      if (!r.t.step) continue;
      for (const [key, tag] of r.outs) r.cio[key] = io[tag];
      r.t.step(r.s, r.p, r.cio, dt);
      for (const [key, tag] of r.ins) raw[tag] = r.cio[key];
    }
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
        pt.body.setNextKinematicTranslation({ x: pt.pin.p[0], y: pt.pin.p[1], z: pt.pin.p[2] });
        pt.body.setNextKinematicRotation({ x: pt.pin.q[0], y: pt.pin.q[1], z: pt.pin.q[2], w: pt.pin.q[3] });
        continue;
      }
      if (!pt.held) continue;
      const P = compose(W[pt.held.id][pt.held.link], pt.rel);
      pt.body.setNextKinematicTranslation({ x: P.p[0] * SK, y: P.p[1] * SK, z: P.p[2] * SK });
      pt.body.setNextKinematicRotation({ x: P.q[0], y: P.q[1], z: P.q[2], w: P.q[3] });
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
    // 6a. material flow: emitters spawn at their frame once the spot is clear; removers take
    // every part whose centre is inside their box.
    for (const em of emitters) {
      const r = em.r;
      if (r.s.req <= r.s.done) continue;
      if (!em.next) {
        const j = r.p.jitterMm;
        em.next = compose(W[r.id][defs.get(r.id).root], pose(j > 0 ? [(em.rand() * 2 - 1) * j, (em.rand() * 2 - 1) * j, 0] : undefined));
      }
      if (!spawnClear(defs.get(r.p.template), em.next, r.p.dropOnto)) continue;
      r.s.done++;
      spawnPart(r.p.template, em.next, r.id + '.' + r.s.done, r.id);
      em.next = null;
    }
    for (const r of removers) {
      const F = invert(W[r.id][defs.get(r.id).root]), [sx, sy, sz] = r.p.size;
      for (const pt of [...parts.values()]) {
        const l = apply(F, partCentre(pt));
        if (Math.abs(l[0]) <= sx / 2 && Math.abs(l[1]) <= sy / 2 && l[2] >= 0 && l[2] <= sz) { removePart(pt.uid, 'remove'); r.s.n++; }
      }
    }
    // 6b. part sensors: Rapier queries against PARTS only (PART_RAYS). The component model
    // turns s.hit into its output at the next step (NO/NC, off-delay).
    for (const r of sensors) {
      const F = W[r.id][defs.get(r.id).root], o = F.p, ax = qrot(F.q, [1, 0, 0]);
      let hit = false;
      if (r.t.sense === 'ray') {
        hit = !!world.castRay(new R.Ray({ x: o[0] * SK, y: o[1] * SK, z: o[2] * SK }, { x: ax[0], y: ax[1], z: ax[2] }), r.p.range * SK, true, undefined, PART_RAYS);
      } else {
        const c = apply(F, [r.p.range / 2, 0, 0]);
        world.intersectionsWithShape({ x: c[0] * SK, y: c[1] * SK, z: c[2] * SK }, { x: 0, y: 0, z: 0, w: 1 }, new R.Ball(r.p.range / 2 * SK), (/** @type {any} */ col) => {
          const pt = colPart.get(col.handle);
          if (pt && (!r.p.metalOnly || METAL.has(defs.get(pt.tpl).p.material))) { hit = true; return false; }
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
          pt.held = { id: r.id, link };
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
    for (const tag of inTags) publish(tag, forced.has(tag) ? forced.get(tag) : raw[tag]);
  }

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
      if (at) pt.pin.p = [at[0] * SK, at[1] * SK, at[2] * SK];
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
      const t = pt.body.translation(), q = pt.body.rotation();
      pt.pin = { p: [t.x, t.y, t.z], q: [q.x, q.y, q.z, q.w] };     // metres: written straight back to Rapier
      pt.body.setBodyType(R.RigidBodyType.KinematicPositionBased, true);
    } else {
      pt.pin = null;
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
    hands.length = 0;                                          // a hand edge for a part that is about to go
    for (const uid of [...parts.keys()]) removePart(uid, 'reset');
    spawnSceneParts();
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
  let timer = null, last = 0, acc = 0;
  function tick() {
    const now = clock();
    acc += now - last;
    last = now;
    let n = Math.floor(acc / dtMs);
    if (n > MAX_STEPS) {
      // Sim time falls behind wall time here. Say when and how long, so a stall can be traced
      // to what blocked the event loop (a synchronous require, a browse, GC).
      warn('plant stalled ' + Math.round(n * dtMs) + ' ms: ' + (n - MAX_STEPS) + ' steps dropped (overruns)');
      plant.overruns += n - MAX_STEPS; n = MAX_STEPS; acc = 0;
    } else acc -= n * dtMs;
    const t0 = performance.now();
    for (let i = 0; i < n; i++) step();
    if (n) plant.stepUs = Math.round(plant.stepUs * 0.9 + (performance.now() - t0) / n * 1000 * 0.1);
    exchange();
  }
  function start() {
    if (timer) return;
    last = clock(); acc = 0;
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
      plant: { mode: plant.mode, t: plant.t, overruns: plant.overruns, stepUs: plant.stepUs, parts: parts.size },
    };
  }
  async function close() {
    stop();
    await driver?.close?.();
    world.free();
  }

  return plant;
}
