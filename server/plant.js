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
import { compile, worldPoses, tags as sceneTags, validate, stringify } from '../lib/scene.js';
import { qmul, qaxis, qeuler } from '../lib/math.js';

/** mm -> m. The ONLY unit conversion between the scene and Rapier. */
export const SK = 0.001;
const MAX_STEPS = 50;             // per tick; past it the step is dropped and counted as an overrun
const RING = 200000;
/** Rapier cylinders run along Y; scene cylinders run along Z. */
const Y_TO_Z = qaxis([1, 0, 0], 90);

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
  const W0 = worldPoses(scene, dofOf());
  /** @type {Array<{id: string, link: string, body: any, kinematic: boolean}>} */
  const bodies = [];
  for (const c of order) {
    const d = defs.get(c.id);
    for (const l of d.links) {
      const shapes = d.shapes.filter((/** @type {any} */ s) => s.link === l.name && s.collide !== false);
      if (!shapes.length) continue;
      const kinematic = linkMoves(c, l.name), P = W0[c.id][l.name];
      const desc = (kinematic ? R.RigidBodyDesc.kinematicPositionBased() : R.RigidBodyDesc.fixed())
        .setTranslation(P.p[0] * SK, P.p[1] * SK, P.p[2] * SK)
        .setRotation({ x: P.q[0], y: P.q[1], z: P.q[2], w: P.q[3] });
      const body = world.createRigidBody(desc);
      for (const s of shapes) {
        let q = qeuler(s.rot), cd;
        if (s.kind === 'box') cd = R.ColliderDesc.cuboid(s.size[0] / 2 * SK, s.size[1] / 2 * SK, s.size[2] / 2 * SK);
        else if (s.kind === 'cyl') { cd = R.ColliderDesc.cylinder(s.h / 2 * SK, s.r * SK); q = qmul(q, Y_TO_Z); }
        else cd = R.ColliderDesc.ball(s.r * SK);
        cd.setTranslation(s.at[0] * SK, s.at[1] * SK, s.at[2] * SK).setRotation({ x: q[0], y: q[1], z: q[2], w: q[3] });
        world.createCollider(cd, body);
      }
      bodies.push({ id: c.id, link: l.name, body, kinematic });
    }
  }
  const kin = bodies.filter(b => b.kinematic);

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
  const ctlIo = /** @type {Record<string, any>} */ ({});
  const events = [];
  const warned = new Set();

  const plant = {
    scene, world, bodies, io, dtMs, inTags, outTags,
    t: 0, mode: 'stop', overruns: 0, stepUs: 0, events,
    /** @type {Array<(msg: string, t: number) => void>} */
    warnListeners: [],
    dof: dofOf(),
    step, exchange, start, stop, reset, press, force, fromPlc, plcSaw, driverUp, snapshot, status, close, warn,
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
    }
    // 5. physics
    world.step();
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
    return { t: plant.t, dof: plant.dof, io: { ...io }, forced: Object.fromEntries(forced), parts: [] };
  }
  function status() {
    return {
      io: driver ? driver.status() : { driver: 'internal', ok: true, msg: 'INTERNAL CONTROLLER - not a PLC', heartbeat: null },
      plant: { mode: plant.mode, t: plant.t, overruns: plant.overruns, stepUs: plant.stepUs },
    };
  }
  async function close() {
    stop();
    await driver?.close?.();
    world.free();
  }

  return plant;
}
