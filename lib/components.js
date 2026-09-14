// @ts-check
// Component types: plain objects in ONE map. No classes, no registry. Geometry is plain data
// (box / cyl / sphere), so Node builds colliders and the browser builds meshes from the SAME
// numbers. Imported by both sides: never import three, Rapier or node: modules here.
//
// A type has:
//   params            descriptors, which also drive the editor's property panel
//   io(p)             key -> { dir: 'out' (PLC -> plant) | 'in' (plant -> PLC), type, dev, words, hold? }
//                     hold: false exempts a handshake reply from the minPulseMs hold (see servo `done`)
//   links(p)          name -> { parent?, at?, rot?, dof?: 'prismatic'|'revolute', axis?, scale? };
//                     the FIRST link is the root, and parents come before children
//   sockets(p)        name -> { link, at?, rot? }: mount points for children, and snap targets
//   shapes(p)         [{ link, kind: 'box'|'cyl'|'sphere', size | r,h, at, rot?, mat, color?, glow?, collide? }]
//                     box `at` is the centre; cyl runs along local Z, centred on `at`;
//                     glow: an io key whose value lights the shape (lamps, reed LEDs)
//   init(p)           state; a numeric `x` is the component's DOF, streamed to the browser
//   step(s, p, io, dt) dt in seconds; reads `out` keys from io, writes `in` keys into it
//   press(s, p, key, down)  for parts clicked in 3D (the browser sends edges only)
//   check(p)          extra parameter errors, for validate()
import { clamp } from './math.js';

/**
 * One step of the trapezoid motion model, ported from rb4axis web/kin.js `langkahSumbu`
 * (the profile PRG_SIM_ROBOT.st runs). THE single copy: servo, index-table servo drive and
 * anything else that moves "like an axis" call this. A second copy disagrees one day.
 * @param {number} pos @param {number} cmd @param {number} vel @param {number} vmax
 * @param {number} acc @param {number} dt seconds
 */
export function trapStep(pos, cmd, vel, vmax, acc, dt) {
  const d = cmd - pos;
  let vt = Math.min(vmax, Math.sqrt(2 * acc * Math.abs(d)));
  if (d < 0) vt = -vt;
  const dv = acc * dt;
  if (Math.abs(vt - vel) <= dv) vel = vt;
  else vel += (vt > vel ? dv : -dv);
  const s = vel * dt;
  if (Math.abs(d) <= Math.abs(s)) return { pos: cmd, vel: 0, moving: false };
  return { pos: pos + s, vel, moving: true };
}

export const COLORS = { green: '#27b045', red: '#e0322c', yellow: '#f2c21b', blue: '#2a6fdb',
                        white: '#eeeeee', amber: '#ff9a1f', black: '#222222' };
const COLOR_NAMES = Object.keys(COLORS);

/** @param {{def?: any}} d */
const clone = d => (d.def && typeof d.def === 'object' ? JSON.parse(JSON.stringify(d.def)) : d.def);

/**
 * Params with defaults filled in. A `def` that is a function is evaluated after the plain
 * ones, so it can depend on them (a cylinder's rod follows its bore).
 * @param {{params: any[]}} t @param {Record<string, any>} [given]
 */
export function withDefaults(t, given = {}) {
  /** @type {Record<string, any>} */
  const p = {};
  for (const d of t.params) if (typeof d.def !== 'function') p[d.k] = given[d.k] ?? clone(d);
  for (const d of t.params) if (typeof d.def === 'function') p[d.k] = given[d.k] ?? d.def(p);
  return p;
}

// ---------------------------------------------------------------------------- structure

const frame = {
  label: 'Machine frame', group: 'structure',
  params: [{ k: 'size', type: 'vec3', def: [900, 600, 800], unit: 'mm' }],
  links: () => ({ body: {} }),
  sockets: p => ({ top: { link: 'body', at: [0, 0, p.size[2]] } }),
  shapes: p => {
    const [sx, sy, sz] = p.size, leg = 40, top = 20;
    const s = [{ link: 'body', kind: 'box', size: [sx, sy, top], at: [0, 0, sz - top / 2], mat: 'alu' }];
    for (const [ix, iy] of [[-1, -1], [1, -1], [-1, 1], [1, 1]]) {
      s.push({ link: 'body', kind: 'box', size: [leg, leg, sz - top], at: [ix * (sx / 2 - leg / 2), iy * (sy / 2 - leg / 2), (sz - top) / 2], mat: 'profile' });
    }
    return s;
  },
};

const plate = {
  label: 'Plate / bracket', group: 'structure',
  params: [{ k: 'size', type: 'vec3', def: [200, 100, 12], unit: 'mm' }],
  links: () => ({ body: {} }),
  sockets: p => ({ top: { link: 'body', at: [0, 0, p.size[2]] }, bottom: { link: 'body', at: [0, 0, 0], rot: [180, 0, 0] } }),
  shapes: p => [{ link: 'body', kind: 'box', size: p.size, at: [0, 0, p.size[2] / 2], mat: 'alu' }],
};

// ---------------------------------------------------------------------------- pneumatics

const ROD = /** @type {Record<number, number>} */ ({ 6: 3, 10: 4, 16: 6, 20: 8, 25: 10, 32: 12, 40: 16, 50: 20, 63: 20, 80: 25, 100: 30 });
const DOUBLE = ['5/2-double', '5/3-closed'];

/** Tube radius, piston face at x = 0 (z0), body length, cap length. @param {Record<string, any>} p */
export function cylGeometry(p) {
  return { R: p.bore / 2 + Math.max(3, p.bore * 0.15), z0: p.bore * 0.5 + 10,
           Lb: p.stroke + p.bore + 20, cap: Math.min(p.bore * 0.6 + 8, 40) };
}

/**
 * Speeds that reproduce the stroke times measured with a stopwatch:
 *   T = valveMs + (stroke - c) / v + c / (k v)   =>   v = ((stroke - c) + c / k) / (T - valveMs)
 * c = cushion length, run at k * v. The measured times are the calibration knob.
 * @param {Record<string, any>} p @returns {{ext: number, ret: number, c: number, k: number}} mm per ms
 */
export function cylSpeeds(p) {
  const c = Math.min(p.cushionMm, p.stroke), k = p.cushionK;
  const v = (/** @type {number} */ T) => ((p.stroke - c) + c / k) / (T - p.valveMs);
  return { ext: v(p.extendMs), ret: v(p.retractMs), c, k };
}

/** Exact piecewise travel for `t` ms: full speed, then cushion speed near the end. */
function travel(x, dir, t, stroke, v, c, k) {
  if (dir > 0) {
    const b = stroke - c;
    if (x < b) { const need = (b - x) / v; if (need >= t) return x + v * t; t -= need; x = b; }
    return Math.min(stroke, x + k * v * t);
  }
  if (x > c) { const need = (x - c) / v; if (need >= t) return x - v * t; t -= need; x = c; }
  return Math.max(0, x - k * v * t);
}

const cylinder = {
  label: 'Air cylinder', group: 'pneumatics',
  params: [
    { k: 'bore', type: 'enum', of: [6, 10, 16, 20, 25, 32, 40, 50, 63, 80, 100], def: 32, unit: 'mm' },
    { k: 'stroke', type: 'num', def: 100, min: 1, unit: 'mm' },
    { k: 'rod', type: 'num', def: (/** @type {any} */ p) => ROD[p.bore] || Math.round(p.bore * 0.35), min: 1, unit: 'mm' },
    { k: 'valve', type: 'enum', of: ['5/2-single', '5/2-double', '5/3-closed', 'single-acting'], def: '5/2-double' },
    { k: 'extendMs', type: 'num', def: 500, min: 1, unit: 'ms' },
    { k: 'retractMs', type: 'num', def: 450, min: 1, unit: 'ms' },
    { k: 'cushionMm', type: 'num', def: 5, min: 0, unit: 'mm' },
    { k: 'cushionK', type: 'num', def: 0.3, min: 0.01, max: 1 },
    { k: 'valveMs', type: 'num', def: 15, min: 0, unit: 'ms' },
    { k: 'extWord', type: 'str', def: 'EXT' },
    { k: 'retWord', type: 'str', def: 'RET' },
    { k: 'reedBand', type: 'num', def: 6, min: 0.1, unit: 'mm' },
    { k: 'reedHyst', type: 'num', def: 0.5, min: 0, unit: 'mm' },
    { k: 'switches', type: 'switches', def: (/** @type {any} */ p) => [{ id: 'ret', pos: 1 }, { id: 'ext', pos: p.stroke - 1 }] },
    // A tool on the rod end that really touches parts (a kinematic collider): a pusher plate or
    // a stopper pin. headSize is [x, y, length along the rod].
    { k: 'head', type: 'enum', of: ['none', 'plate', 'pin'], def: 'none' },
    { k: 'headSize', type: 'vec3', def: [60, 40, 8], unit: 'mm' },
  ],
  // Presets are parameter bundles, not new code (docs/PLAN.md §3).
  presets: [
    // A square stopper block, not a round pin: a cylinder collider over a part's top edge keeps
    // blocking below ~12 mm clearance even with the plant's contact refresh (measured).
    { label: 'Stopper', params: { bore: 16, stroke: 30, valve: '5/2-single', extendMs: 120, retractMs: 120, head: 'plate', headSize: [12, 12, 25] } },
    { label: 'Pusher', params: { bore: 25, stroke: 150, extendMs: 400, retractMs: 350, head: 'plate', headSize: [40, 80, 8] } },
    { label: 'Lifter', params: { bore: 32, stroke: 50, extendMs: 300, retractMs: 300, head: 'plate', headSize: [120, 120, 10] } },
  ],
  io: (/** @type {any} */ p) => {
    /** @type {Record<string, any>} */
    const io = { solExt: { dir: 'out', type: 'BOOL', dev: 'SOL', words: p.extWord } };
    if (DOUBLE.includes(p.valve)) io.solRet = { dir: 'out', type: 'BOOL', dev: 'SOL', words: p.retWord };
    for (const w of p.switches) {
      io['sw.' + w.id] = { dir: 'in', type: 'BOOL', dev: 'AS',
                           words: w.words || (w.id === 'ext' ? p.extWord : w.id === 'ret' ? p.retWord : String(w.id).toUpperCase()) };
    }
    return io;
  },
  links: (/** @type {any} */ p) => ({ body: {}, rod: { at: [0, 0, cylGeometry(p).Lb], dof: 'prismatic', axis: [0, 0, 1] } }),
  sockets: (/** @type {any} */ p) => {
    const g = cylGeometry(p);
    return { foot: { link: 'body', at: [0, 0, 0] }, head: { link: 'body', at: [0, 0, g.Lb] }, rodEnd: { link: 'rod', at: [0, 0, 27] } };
  },
  shapes: (/** @type {any} */ p) => {
    const g = cylGeometry(p), sq = 2 * g.R + 4;
    const rodIn = g.Lb - g.z0;                       // rod length inside the tube at x = 0
    const s = [
      { link: 'body', kind: 'cyl', r: g.R, h: g.Lb - 2 * g.cap, at: [0, 0, g.Lb / 2], mat: 'tube' },
      { link: 'body', kind: 'box', size: [sq, sq, g.cap], at: [0, 0, g.cap / 2], mat: 'alu' },
      { link: 'body', kind: 'box', size: [sq, sq, g.cap], at: [0, 0, g.Lb - g.cap / 2], mat: 'alu' },
      { link: 'rod', kind: 'cyl', r: p.rod / 2, h: rodIn + 15, at: [0, 0, (15 - rodIn) / 2], mat: 'rod' },
      { link: 'rod', kind: 'box', size: [p.rod * 1.6, p.rod * 1.6, 12], at: [0, 0, 21], mat: 'steel' },
    ];
    for (const w of p.switches) {
      s.push({ link: 'body', kind: 'box', size: [6, 8, 18], at: [g.R + 3, 0, g.z0 + w.pos], mat: 'reed', glow: 'sw.' + w.id, collide: false });
    }
    const [hx, hy, hz] = p.headSize;                  // starts at the rodEnd socket (z 27 on the rod)
    if (p.head === 'plate') s.push({ link: 'rod', kind: 'box', size: [hx, hy, hz], at: [0, 0, 27 + hz / 2], mat: 'steel' });
    if (p.head === 'pin') s.push({ link: 'rod', kind: 'cyl', r: hx / 2, h: hz, at: [0, 0, 27 + hz / 2], mat: 'steel' });
    return s;
  },
  states: ['retracted', 'extending', 'extended', 'retracting'],
  init: (/** @type {any} */ p) => {
    const rest = p.valve === '5/3-closed' ? 0 : -1;
    return { x: 0, spool: rest, pend: rest, tv: 0, sw: /** @type {Record<string, boolean>} */ ({}) };
  },
  step(/** @type {any} */ s, /** @type {any} */ p, /** @type {any} */ io, /** @type {number} */ dt) {
    const ext = !!io.solExt, ret = !!io.solRet;
    let want;
    if (p.valve === '5/2-double') want = ext && !ret ? 1 : ret && !ext ? -1 : s.pend;   // both or neither: holds
    else if (p.valve === '5/3-closed') want = ext && !ret ? 1 : ret && !ext ? -1 : 0;   // centre closed: stops
    else want = ext ? 1 : -1;                                                           // spring return
    if (want !== s.pend) { s.pend = want; s.tv = p.valveMs; }
    let t = dt * 1000;
    if (s.spool !== s.pend) {
      const u = Math.min(s.tv, t);
      s.tv -= u; t -= u;
      if (s.tv <= 0) s.spool = s.pend;
    }
    if (t > 0 && s.spool) {
      const v = cylSpeeds(p);
      s.x = travel(s.x, s.spool, t, p.stroke, s.spool > 0 ? v.ext : v.ret, v.c, v.k);
    }
    for (const w of p.switches) {
      const half = (w.band ?? p.reedBand) / 2, d = Math.abs(s.x - w.pos);
      s.sw[w.id] = s.sw[w.id] ? d <= half + p.reedHyst : d <= half;
      io['sw.' + w.id] = s.sw[w.id];
    }
  },
  check(/** @type {any} */ p) {
    const e = [];
    if (!(p.extendMs > p.valveMs)) e.push('extendMs must be longer than valveMs');
    if (!(p.retractMs > p.valveMs)) e.push('retractMs must be longer than valveMs');
    if (!(p.rod < p.bore)) e.push('rod must be thinner than the bore');
    const ids = new Set();
    for (const w of p.switches || []) {
      if (!w || typeof w.id !== 'string' || !/^[a-z0-9_]+$/i.test(w.id)) { e.push('switch id must be a word: ' + JSON.stringify(w && w.id)); continue; }
      if (ids.has(w.id)) e.push('duplicate switch id ' + w.id);
      ids.add(w.id);
      if (!(w.pos >= 0 && w.pos <= p.stroke)) e.push('switch ' + w.id + ' pos ' + w.pos + ' outside the stroke 0..' + p.stroke);
    }
    return e;
  },
};

// ---------------------------------------------------------------------------- motion

const servoLinear = {
  label: 'Servo linear axis', group: 'motion',
  params: [
    // ponytail: only `plant` mode; `positions` and `mirror` (docs/PLAN.md §3) arrive with P2/P5.
    { k: 'mode', type: 'enum', of: ['plant'], def: 'plant' },
    { k: 'stroke', type: 'num', def: 500, min: 1, unit: 'mm' },
    { k: 'vmax', type: 'num', def: 300, min: 0.1, unit: 'mm/s' },
    { k: 'acc', type: 'num', def: 1500, min: 0.1, unit: 'mm/s²' },
    { k: 'body', type: 'vec3', def: [640, 90, 70], unit: 'mm' },
    { k: 'inPosBand', type: 'num', def: 0.1, min: 0, unit: 'mm' },
  ],
  // Execute is a LEVEL held until Done (MC_MoveAbsolute semantics): its rising edge latches
  // the target, Done stays on while Execute is held, and drops with it. A one-scan pulse
  // would fall between two OPC UA samples. The PLC must see Done drop before the next Execute.
  // Done, Busy and InPos are exempt from the minPulseMs hold. They are replies to PLC commands,
  // and every short pulse of theirs is caused by the PLC: Done falls because Execute dropped,
  // InPos falls because the next move started. Stretching them would report "in position"
  // while the axis already moves. The hold is for physical events the PLC does not cause.
  io: () => ({
    target: { dir: 'out', type: 'LREAL' }, exec: { dir: 'out', type: 'BOOL' },
    done: { dir: 'in', type: 'BOOL', hold: false }, busy: { dir: 'in', type: 'BOOL', hold: false },
    actPos: { dir: 'in', type: 'LREAL' }, inPos: { dir: 'in', type: 'BOOL', hold: false },
  }),
  links: (/** @type {any} */ p) => {
    const [L, , H] = p.body, Lc = L - p.stroke;
    return { body: {}, carriage: { at: [-L / 2 + Lc / 2, 0, 0.6 * H], dof: 'prismatic', axis: [1, 0, 0] } };
  },
  sockets: (/** @type {any} */ p) => ({ carriage: { link: 'carriage', at: [0, 0, 0.4 * p.body[2]] }, base: { link: 'body', at: [0, 0, 0] } }),
  shapes: (/** @type {any} */ p) => {
    const [L, W, H] = p.body, Lc = L - p.stroke;
    return [
      { link: 'body', kind: 'box', size: [L, W, 0.6 * H], at: [0, 0, 0.3 * H], mat: 'alu' },
      { link: 'body', kind: 'box', size: [70, 0.8 * W, 0.8 * W], at: [L / 2 + 35, 0, 0.3 * H], mat: 'dark' },
      { link: 'carriage', kind: 'box', size: [Lc - 10, W + 10, 0.4 * H], at: [0, 0, 0.2 * H], mat: 'steel' },
    ];
  },
  init: () => ({ x: 0, v: 0, tgt: 0, exec: false, moving: false, arrived: false }),
  step(/** @type {any} */ s, /** @type {any} */ p, /** @type {any} */ io, /** @type {number} */ dt) {
    const e = !!io.exec;
    if (e && !s.exec) { s.tgt = clamp(Number(io.target) || 0, 0, p.stroke); s.moving = true; s.arrived = false; }
    s.exec = e;
    if (s.moving) {
      const r = trapStep(s.x, s.tgt, s.v, p.vmax, p.acc, dt);
      s.x = r.pos; s.v = r.vel;
      if (!r.moving) { s.moving = false; s.arrived = true; }
    }
    io.done = e && s.arrived;
    io.busy = s.moving;
    io.actPos = Math.round(s.x * 1000) / 1000;
    io.inPos = !s.moving && Math.abs(s.x - s.tgt) <= p.inPosBand;
  },
  check(/** @type {any} */ p) {
    return p.body[0] - p.stroke >= 30 ? [] : ['body length must exceed the stroke by at least 30 mm (carriage)'];
  },
};

/**
 * Cycloidal displacement, the profile a cam indexer follows: s(0) = 0, s(1) = 1 exactly, and
 * the speed is zero at both ends, so a station is reached without a jolt.
 * @param {number} tau 0..1
 */
export const cycloid = tau => tau - Math.sin(2 * Math.PI * tau) / (2 * Math.PI);

const indexTable = {
  label: 'Index table', group: 'motion',
  // Cam drive (docs/PLAN.md §3): while `run` is held the camshaft turns, and one revolution is
  // one index plus one dwell. Dropping `run` mid-index leaves the table where it stopped, as a
  // real indexer does, and `inPos` stays off because it is between stations.
  params: [
    { k: 'stations', type: 'num', def: 6, min: 2, max: 48 },
    { k: 'camMs', type: 'num', def: 1200, min: 10, unit: 'ms' },
    { k: 'indexFrac', type: 'num', def: 0.5, min: 0.05, max: 0.95 },
    { k: 'diameter', type: 'num', def: 600, min: 50, unit: 'mm' },
    { k: 'height', type: 'num', def: 120, min: 10, unit: 'mm' },
  ],
  io: () => ({
    run: { dir: 'out', type: 'BOOL', dev: 'MC', words: 'INDEX' },
    inPos: { dir: 'in', type: 'BOOL', dev: 'AS', words: 'INPOS' },
    origin: { dir: 'in', type: 'BOOL', dev: 'AS', words: 'ORIGIN' },
    station: { dir: 'in', type: 'INT' },
  }),
  links: () => ({ body: {}, table: { dof: 'revolute', axis: [0, 0, 1] } }),
  sockets: (/** @type {any} */ p) => {
    /** @type {Record<string, any>} */
    const s = {};
    const r = p.diameter * 0.38;
    for (let i = 0; i < p.stations; i++) {
      const a = i * 2 * Math.PI / p.stations;
      s['s' + i] = { link: 'table', at: [r * Math.cos(a), r * Math.sin(a), p.height + 20] };
    }
    return s;
  },
  shapes: (/** @type {any} */ p) => {
    const r = p.diameter / 2;
    /** @type {any[]} */
    const s = [
      { link: 'body', kind: 'cyl', r: r * 0.22, h: p.height, at: [0, 0, p.height / 2], mat: 'dark' },
      { link: 'table', kind: 'cyl', r, h: 20, at: [0, 0, p.height + 10], mat: 'alu' },
    ];
    for (let i = 0; i < p.stations; i++) {
      const a = i * 2 * Math.PI / p.stations, rr = p.diameter * 0.38;
      s.push({ link: 'table', kind: 'box', size: [40, 40, 4], at: [rr * Math.cos(a), rr * Math.sin(a), p.height + 22],
               mat: i === 0 ? 'reed' : 'steel', collide: false });
    }
    return s;
  },
  init: () => ({ x: 0, cam: 0, k: 0 }),
  step(/** @type {any} */ s, /** @type {any} */ p, /** @type {any} */ io, /** @type {number} */ dt) {
    const pitch = 360 / p.stations;
    if (io.run) {
      const before = s.cam;
      s.cam += dt * 1000 / p.camMs;
      if (s.cam >= 1) s.cam -= 1;
      // leaving the moving arc completes one index
      if (before < p.indexFrac && (s.cam >= p.indexFrac || s.cam < before)) s.k += 1;
    }
    const moving = s.cam > 0 && s.cam < p.indexFrac;
    const tau = moving ? s.cam / p.indexFrac : 0;
    s.x = pitch * (s.k + cycloid(tau));
    io.inPos = !moving;                              // false while between stations, run or not
    io.station = ((s.k % p.stations) + p.stations) % p.stations;
    io.origin = io.inPos && io.station === 0;
  },
};

// ---------------------------------------------------------------------------- operator

const pushbutton = {
  label: 'Pushbutton', group: 'operator',
  params: [
    { k: 'kind', type: 'enum', of: ['momentary', 'alternate'], def: 'momentary' },
    { k: 'color', type: 'enum', of: COLOR_NAMES, def: 'green' },
    { k: 'lamp', type: 'bool', def: true },
  ],
  io: (/** @type {any} */ p) => {
    /** @type {Record<string, any>} */
    const io = { pb: { dir: 'in', type: 'BOOL', dev: 'PB', words: 'PB' } };
    if (p.lamp) io.lamp = { dir: 'out', type: 'BOOL', dev: 'PL', words: 'LAMP' };
    return io;
  },
  links: () => ({ body: {}, cap: { at: [0, 0, 20], dof: 'prismatic', axis: [0, 0, -1] } }),
  sockets: () => ({}),
  shapes: (/** @type {any} */ p) => [
    { link: 'body', kind: 'cyl', r: 16, h: 20, at: [0, 0, 10], mat: 'dark' },
    { link: 'cap', kind: 'cyl', r: 11, h: 8, at: [0, 0, 4], mat: 'paint', color: COLORS[p.color], glow: p.lamp ? 'lamp' : undefined },
  ],
  pressKey: 'pb',
  init: () => ({ x: 0, on: false }),
  press(/** @type {any} */ s, /** @type {any} */ p, /** @type {string} */ key, /** @type {boolean} */ down) {
    if (key !== 'pb') return;
    if (p.kind === 'alternate') { if (down) s.on = !s.on; } else s.on = !!down;
  },
  step(/** @type {any} */ s, /** @type {any} */ p, /** @type {any} */ io) {
    io.pb = s.on;
    s.x = s.on ? 3 : 0;
  },
};

const lamp = {
  label: 'Pilot lamp', group: 'operator',
  params: [{ k: 'color', type: 'enum', of: COLOR_NAMES, def: 'green' }],
  io: () => ({ lamp: { dir: 'out', type: 'BOOL', dev: 'PL', words: 'LAMP' } }),
  links: () => ({ body: {} }),
  sockets: () => ({}),
  shapes: (/** @type {any} */ p) => [
    { link: 'body', kind: 'cyl', r: 15, h: 14, at: [0, 0, 7], mat: 'dark' },
    { link: 'body', kind: 'sphere', r: 11, at: [0, 0, 16], mat: 'paint', color: COLORS[p.color], glow: 'lamp', collide: false },
  ],
};

// ---------------------------------------------------------------------------- items

/** kg/m³ by workpiece material: Rapier mass comes from collider volume × density. */
export const DENSITY = /** @type {Record<string, number>} */ ({ steel: 7850, alu: 2700, plastic: 1200 });

const workpiece = {
  label: 'Workpiece', group: 'items',
  // dynamic: a loose Rapier body that falls, slides and gets pushed. Static (the default) is a
  // fixed body, e.g. a part that only sits in a press.
  params: [
    { k: 'kind', type: 'enum', of: ['box', 'cyl'], def: 'box' },
    { k: 'size', type: 'vec3', def: [60, 40, 30], unit: 'mm' },
    { k: 'color', type: 'str', def: '#c79a52' },
    { k: 'material', type: 'enum', of: ['steel', 'alu', 'plastic'], def: 'steel' },
    { k: 'dynamic', type: 'bool', def: false },
    { k: 'friction', type: 'num', def: 0.5, min: 0 },
    { k: 'restitution', type: 'num', def: 0.1, min: 0, max: 1 },
  ],
  part: true,
  links: () => ({ body: {} }),
  sockets: (/** @type {any} */ p) => ({ top: { link: 'body', at: [0, 0, p.size[2]] } }),
  shapes: (/** @type {any} */ p) => [p.kind === 'cyl'
    ? { link: 'body', kind: 'cyl', r: p.size[0] / 2, h: p.size[2], at: [0, 0, p.size[2] / 2], mat: 'part', color: p.color }
    : { link: 'body', kind: 'box', size: p.size, at: [0, 0, p.size[2] / 2], mat: 'part', color: p.color }],
};

// ---------------------------------------------------------------------------- material flow
// These types are pure models like the rest; the plant does their Rapier work (belt drive,
// spawning, removing), keyed by `flow`.

const STRIPE = 100;                      // mm between belt stripes

const conveyor = {
  label: 'Belt conveyor', group: 'material flow', flow: 'belt',
  params: [
    { k: 'length', type: 'num', def: 1500, min: 100, unit: 'mm' },
    { k: 'width', type: 'num', def: 200, min: 20, unit: 'mm' },
    { k: 'height', type: 'num', def: 800, min: 20, unit: 'mm' },
    { k: 'speed', type: 'num', def: 300, min: 0, unit: 'mm/s' },
    { k: 'accelMs', type: 'num', def: 100, min: 0, unit: 'ms' },
    { k: 'mu', type: 'num', def: 0.6, min: 0 },
    { k: 'guides', type: 'num', def: 30, min: 0, unit: 'mm' },
  ],
  io: () => ({ run: { dir: 'out', type: 'BOOL' }, rev: { dir: 'out', type: 'BOOL' } }),
  // x = belt travel in mm (unbounded); the stripes show it, wrapped every STRIPE mm.
  links: (/** @type {any} */ p) => ({ body: {}, stripes: { at: [0, 0, p.height], dof: 'prismatic', axis: [1, 0, 0], wrap: STRIPE } }),
  sockets: (/** @type {any} */ p) => ({
    top: { link: 'body', at: [0, 0, p.height] },
    start: { link: 'body', at: [-p.length / 2, 0, p.height] }, end: { link: 'body', at: [p.length / 2, 0, p.height] },
  }),
  shapes: (/** @type {any} */ p) => {
    const L = p.length, W = p.width, H = p.height, T = 12, leg = 40;
    /** @type {any[]} */
    const s = [{ link: 'body', kind: 'box', size: [L, W, T], at: [0, 0, H - T / 2], mat: 'dark', belt: true }];
    for (const iy of [-1, 1]) {
      s.push({ link: 'body', kind: 'box', size: [L, 30, 60], at: [0, iy * (W / 2 + 15), H - 30], mat: 'profile' });
      if (p.guides > 0) s.push({ link: 'body', kind: 'box', size: [L, 6, p.guides], at: [0, iy * (W / 2 + 3), H + p.guides / 2], mat: 'alu' });
      for (const ix of [-1, 1]) s.push({ link: 'body', kind: 'box', size: [leg, leg, H - 60], at: [ix * (L / 2 - leg), iy * (W / 2 + 15), (H - 60) / 2], mat: 'profile' });
    }
    for (let i = 0; i < Math.floor((L - 20) / STRIPE); i++) {
      s.push({ link: 'stripes', kind: 'box', size: [8, W - 12, 1], at: [-L / 2 + i * STRIPE + 6, 0, 0.5], mat: 'paint', color: '#5b6068', collide: false });
    }
    return s;
  },
  init: () => ({ x: 0, v: 0 }),
  step(/** @type {any} */ s, /** @type {any} */ p, /** @type {any} */ io, /** @type {number} */ dt) {
    const want = io.run ? (io.rev ? -p.speed : p.speed) : 0;
    const a = p.accelMs > 0 ? p.speed / (p.accelMs / 1000) * dt : Infinity;
    s.v += clamp(want - s.v, -a, a);
    s.x += s.v * dt;
  },
};

const emitter = {
  label: 'Part emitter', group: 'material flow', flow: 'emitter',
  params: [
    // `of: 'part'` is a DESCRIPTOR, not a type name: any type carrying `part` can be emitted, so
    // a feeder can drop pallets as well as workpieces. partRoles() has always keyed off the same
    // descriptor; only the validator was matching the literal type name.
    { k: 'template', type: 'ref', of: 'part', template: true, def: '' },
    { k: 'mode', type: 'enum', of: ['interval', 'tag'], def: 'interval' },
    { k: 'intervalMs', type: 'num', def: 2000, min: 10, unit: 'ms' },
    { k: 'max', type: 'num', def: 0, min: 0 },
    { k: 'jitterMm', type: 'num', def: 0, min: 0, unit: 'mm' },
    // A part falls onto whatever is under the feeder, so by default the whole column below must
    // be free. A feeder that places one part ON another (a lid onto a base) sets dropOnto.
    { k: 'dropOnto', type: 'bool', def: false },
  ],
  // interval: one part every intervalMs while `enable` is on (or unbound); tag: one per rising
  // edge of `emit`. max 0 = no limit. A part waits while the spawn spot is still occupied.
  io: (/** @type {any} */ p) => (p.mode === 'tag'
    ? { emit: { dir: 'out', type: 'BOOL' }, count: { dir: 'in', type: 'UDINT' } }
    : { enable: { dir: 'out', type: 'BOOL' }, count: { dir: 'in', type: 'UDINT' } }),
  links: () => ({ body: {} }),
  sockets: () => ({}),
  shapes: () => [{ link: 'body', kind: 'box', size: [30, 30, 4], at: [0, 0, 2], mat: 'reed', collide: false }],
  init: (/** @type {any} */ p) => ({ req: 0, done: 0, t: p.intervalMs, last: false }),
  step(/** @type {any} */ s, /** @type {any} */ p, /** @type {any} */ io, /** @type {number} */ dt) {
    if (p.mode === 'tag') { const e = !!io.emit; if (e && !s.last) s.req++; s.last = e; }
    else if (io.enable !== false) { s.t += dt * 1000; if (s.t >= p.intervalMs) { s.t -= p.intervalMs; s.req++; } }
    if (p.max > 0) s.req = Math.min(s.req, p.max);
    io.count = s.done;
  },
  check: (/** @type {any} */ p) => (p.template ? [] : ['params.template must name the part to emit']),
};

const remover = {
  label: 'Part remover', group: 'material flow', flow: 'remover',
  params: [{ k: 'size', type: 'vec3', def: [150, 250, 200], unit: 'mm' }],
  io: () => ({ count: { dir: 'in', type: 'UDINT' } }),
  links: () => ({ body: {} }),
  sockets: () => ({}),
  // A part whose centre enters this box leaves the plant.
  shapes: (/** @type {any} */ p) => [{ link: 'body', kind: 'box', size: p.size, at: [0, 0, p.size[2] / 2], mat: 'reed', collide: false, ghost: true }],
  init: () => ({ n: 0 }),
  step(/** @type {any} */ s, /** @type {any} */ p, /** @type {any} */ io) { io.count = s.n; },
};

// ---------------------------------------------------------------------------- holding
// One mechanism (docs/PLAN.md §3): the plant makes a held part kinematic and moves it to
// holder x stored relative pose, then hands it back to physics with the holder's velocity.
// `hold` says which rule picks the part.

const vacuumCup = {
  label: 'Vacuum cup', group: 'pneumatics', hold: 'vacuum',
  // The suction face is the frame origin, looking along +Z of the cup, so a cup mounted on a
  // rod end that points down looks down too. The body is drawn behind the face.
  params: [
    { k: 'd', type: 'num', def: 20, min: 4, unit: 'mm' },
    { k: 'reach', type: 'num', def: 2, min: 0.5, unit: 'mm' },
    { k: 'buildMs', type: 'num', def: 80, min: 0, unit: 'ms' },
    { k: 'dropMs', type: 'num', def: 60, min: 0, unit: 'ms' },
  ],
  io: () => ({ on: { dir: 'out', type: 'BOOL', dev: 'SOL', words: 'VAC' }, vac: { dir: 'in', type: 'BOOL', dev: 'VS', words: 'VAC' } }),
  links: () => ({ body: {} }),
  sockets: () => ({}),
  shapes: (/** @type {any} */ p) => [
    { link: 'body', kind: 'cyl', r: p.d / 2, h: 6, at: [0, 0, -3], mat: 'dark', glow: 'vac', collide: false },
    { link: 'body', kind: 'cyl', r: Math.max(2, p.d * 0.25), h: 16, at: [0, 0, -14], mat: 'steel', collide: false },
  ],
  init: () => ({ uid: null, b: 0, d: 0, on: false }),
  // The switch lags the grip like a real one: it rises buildMs after the cup holds, and falls
  // dropMs after it lets go.
  step(/** @type {any} */ s, /** @type {any} */ p, /** @type {any} */ io, /** @type {number} */ dt) {
    const ms = dt * 1000;
    if (s.uid) {
      s.b = Math.min(p.buildMs, s.b + ms); s.d = p.dropMs;
      if (s.b >= p.buildMs) s.on = true;
    } else {
      s.b = 0; s.d = Math.max(0, s.d - ms);
      if (s.d <= 0) s.on = false;
    }
    io.vac = s.on;
  },
};

const gripper = {
  label: '2-finger gripper', group: 'pneumatics', hold: 'grip',
  // The fingers reach along +Z of the gripper, so one mounted on a rod end that points down
  // reaches down too. `x` is HALF the opening, so the gap between the fingers is 2x.
  params: [
    { k: 'span', type: 'num', def: 60, min: 5, unit: 'mm' },
    { k: 'fingerLen', type: 'num', def: 50, min: 5, unit: 'mm' },
    { k: 'fingerW', type: 'num', def: 10, min: 2, unit: 'mm' },
    { k: 'closeMs', type: 'num', def: 250, min: 1, unit: 'ms' },
    { k: 'openMs', type: 'num', def: 250, min: 1, unit: 'ms' },
    { k: 'band', type: 'num', def: 1, min: 0.1, unit: 'mm' },
  ],
  // `closed` sits at FULL close, so a missed grip reads in the PLC exactly as on a real machine
  // (rb4axis SIM_GRIP_TUTUP): with a part between the fingers it never comes on.
  io: () => ({
    close: { dir: 'out', type: 'BOOL', dev: 'SOL', words: 'GRIP' },
    open: { dir: 'in', type: 'BOOL', dev: 'AS', words: 'OPEN' },
    closed: { dir: 'in', type: 'BOOL', dev: 'AS', words: 'CLOSED' },
  }),
  links: () => ({ body: {}, fingerL: { dof: 'prismatic', axis: [0, -1, 0] }, fingerR: { dof: 'prismatic', axis: [0, 1, 0] } }),
  sockets: (/** @type {any} */ p) => ({ tip: { link: 'body', at: [0, 0, p.fingerLen] } }),
  shapes: (/** @type {any} */ p) => [
    { link: 'body', kind: 'box', size: [p.fingerW * 2 + 20, p.span + 26, 20], at: [0, 0, -10], mat: 'dark', collide: false },
    { link: 'fingerL', kind: 'box', size: [p.fingerW * 2, p.fingerW, p.fingerLen], at: [0, -p.fingerW / 2, p.fingerLen / 2], mat: 'steel' },
    { link: 'fingerR', kind: 'box', size: [p.fingerW * 2, p.fingerW, p.fingerLen], at: [0, p.fingerW / 2, p.fingerLen / 2], mat: 'steel' },
  ],
  /** Where a part has to be for the fingers to catch it, in the gripper's frame. */
  zone: (/** @type {any} */ p) => ({ at: [0, 0, p.fingerLen * 0.6], size: [p.fingerW * 2, p.span, p.fingerLen * 0.8] }),
  init: (/** @type {any} */ p) => ({ x: p.span / 2, blockAt: 0, grip: false, uid: null }),
  step(/** @type {any} */ s, /** @type {any} */ p, /** @type {any} */ io, /** @type {number} */ dt) {
    const ms = dt * 1000, closing = !!io.close;
    const v = (p.span / 2) / (closing ? p.closeMs : p.openMs);          // mm per ms
    // blockAt is the half-width of the part in the grip zone, measured by the plant.
    const target = closing ? Math.max(0, s.blockAt) : p.span / 2;
    if (s.x > target) s.x = Math.max(target, s.x - v * ms);
    else if (s.x < target) s.x = Math.min(target, s.x + v * ms);
    s.grip = closing && s.blockAt > 0 && s.x <= s.blockAt + 0.01;
    io.open = s.x >= p.span / 2 - p.band;
    io.closed = s.x <= p.band;
  },
  check: (/** @type {any} */ p) => (p.span / 2 > p.band ? [] : ['span must be wider than twice the switch band']),
};

const nest = {
  // snap: a nest locates the part it catches, square on the pocket floor, as a real one does.
  label: 'Nest / fixture', group: 'structure', hold: 'nest', snap: true,
  // A pocket that keeps a part still. With `clamp` bound it holds only while the clamp is on.
  params: [
    { k: 'size', type: 'vec3', def: [80, 60, 25], unit: 'mm' },
    { k: 'wall', type: 'num', def: 8, min: 1, unit: 'mm' },
  ],
  io: () => ({ clamp: { dir: 'out', type: 'BOOL', dev: 'SOL', words: 'CLAMP' }, present: { dir: 'in', type: 'BOOL', dev: 'PX', words: 'EXIST' } }),
  links: () => ({ body: {} }),
  sockets: (/** @type {any} */ p) => ({ top: { link: 'body', at: [0, 0, p.size[2]] }, base: { link: 'body', at: [0, 0, 0] } }),
  shapes: (/** @type {any} */ p) => {
    const [sx, sy, sz] = p.size, w = p.wall;
    /** @type {any[]} */
    // The walls are drawn but do NOT collide: a part dropped into a pocket with a few mm of
    // clearance hangs on speculative contacts at the wall top edges instead of landing
    // (measured; tests/rapier.test.js). The pocket catches parts by holding them, not by
    // bumping them, so only the floor is solid.
    const s = [{ link: 'body', kind: 'box', size: [sx + 2 * w, sy + 2 * w, 6], at: [0, 0, -3], mat: 'alu' }];
    for (const ix of [-1, 1]) s.push({ link: 'body', kind: 'box', size: [w, sy + 2 * w, sz], at: [ix * (sx + w) / 2, 0, sz / 2], mat: 'alu', collide: false });
    for (const iy of [-1, 1]) s.push({ link: 'body', kind: 'box', size: [sx, w, sz], at: [0, iy * (sy + w) / 2, sz / 2], mat: 'alu', collide: false });
    return s;
  },
  init: () => ({ uid: null }),
  step(/** @type {any} */ s, /** @type {any} */ p, /** @type {any} */ io) { io.present = !!s.uid; },
};

// ---------------------------------------------------------------------------- pallets

const pallet = {
  // `pallet: true` is what a pallet lift filters on, so it takes the carrier and not the load.
  label: 'Pallet', group: 'material flow', pallet: true,
  // A pallet is a PART, not a holder: it rides the belt on friction like any workpiece, and what
  // it carries rides IT the same way (the assembler already runs a lid on a base for 30 minutes
  // that way). Making it a holder would mean following a DYNAMIC body, which the holding
  // mechanism does not do - holders are machine links.
  params: [
    { k: 'size', type: 'vec3', def: [200, 160, 20], unit: 'mm' },
    { k: 'rail', type: 'num', def: 10, min: 0, unit: 'mm' },
    { k: 'color', type: 'str', def: '#4a5560' },
    { k: 'material', type: 'enum', of: ['steel', 'alu', 'plastic'], def: 'plastic' },
    { k: 'dynamic', type: 'bool', def: true },
    { k: 'friction', type: 'num', def: 0.7, min: 0 },
    { k: 'restitution', type: 'num', def: 0.05, min: 0, max: 1 },
  ],
  part: true,
  links: () => ({ body: {} }),
  sockets: (/** @type {any} */ p) => ({ top: { link: 'body', at: [0, 0, p.size[2]] } }),
  shapes: (/** @type {any} */ p) => {
    const [sx, sy, sz] = p.size, r = p.rail;
    /** @type {any[]} */
    const s = [{ link: 'body', kind: 'box', size: [sx, sy, sz], at: [0, 0, sz / 2], mat: 'part', color: p.color }];
    // The rails DO collide, and they must: friction alone does not hold a load when the pallet
    // hits a stopper. Measured on this type before the rails were solid - the pallet stopped dead
    // and its part slid 70 mm along the deck. They are allowed to collide because they are
    // SHORTER than the part they retain, which is the measured exception to the pocket rule
    // (a part that has to DROP between guides needs 12 mm a side; see CLAUDE.md).
    if (r > 0) {
      for (const iy of [-1, 1]) s.push({ link: 'body', kind: 'box', size: [sx, 6, r], at: [0, iy * (sy / 2 - 3), sz + r / 2], mat: 'part', color: p.color });
      for (const ix of [-1, 1]) s.push({ link: 'body', kind: 'box', size: [6, sy - 12, r], at: [ix * (sx / 2 - 3), 0, sz + r / 2], mat: 'part', color: p.color });
    }
    return s;
  },
};

const palletLift = {
  // NO snap, unlike a nest. A nest snaps the part square onto its pocket floor, which means the
  // part is teleported to the holder's frame - and this holder's deck has to sit BELOW the belt
  // to stay out of the path, so snapping dropped the pallet 14 mm the instant the lift took it.
  // Measured: the pallet fell out of the station beam and back into it, and the plant warned
  // "pulse stretched PE_STN 8 -> 20 ms" once per cycle. Raising the deck flush instead blocks the
  // path outright (the pallet stopped dead with its front edge on the deck edge). A pallet that
  // arrived flat on a belt is already square, so the lift keeps its pose and just raises it.
  label: 'Pallet lift & locate', group: 'material flow', hold: 'nest', holdLink: 'lift',
  // It must take PALLETS only: the hold zone otherwise catches the workpiece that is dropped onto
  // the pallet, and the lift ends up holding the load instead of the carrier (measured).
  holdOnly: 'pallet',
  // A stopper cylinder (the Stopper preset) blocks the pallet; this lifts it off the belt and
  // locates it square on the pins. It is the SAME take/follow/release mechanism as a nest, with
  // one difference the plant knows about: the holder frame is the `lift` link, so the pallet
  // rises with it instead of staying where it was caught.
  //
  // One DOF per component (the plant keeps one `x` per component), so the stop pin is a separate
  // cylinder, as on a real machine.
  params: [
    { k: 'size', type: 'vec3', def: [240, 200, 80], unit: 'mm' },
    { k: 'stroke', type: 'num', def: 40, min: 1, unit: 'mm' },
    { k: 'upMs', type: 'num', def: 300, min: 1, unit: 'ms' },
    { k: 'dnMs', type: 'num', def: 300, min: 1, unit: 'ms' },
    { k: 'band', type: 'num', def: 2, min: 0.1, unit: 'mm' },
  ],
  // `clamp` is the holding contract's name for "hold now"; here it is the lift solenoid.
  io: () => ({
    clamp: { dir: 'out', type: 'BOOL', dev: 'SOL', words: 'LIFT' },
    up: { dir: 'in', type: 'BOOL', dev: 'AS', words: 'UP' },
    dn: { dir: 'in', type: 'BOOL', dev: 'AS', words: 'DOWN' },
    present: { dir: 'in', type: 'BOOL', dev: 'PX', words: 'EXIST' },
  }),
  links: () => ({ body: {}, lift: { dof: 'prismatic', axis: [0, 0, 1] } }),
  sockets: (/** @type {any} */ p) => ({ deck: { link: 'lift', at: [0, 0, 0] }, base: { link: 'body', at: [0, 0, -p.size[2]] } }),
  shapes: (/** @type {any} */ p) => {
    const [sx, sy] = p.size;
    /** @type {any[]} */
    const s = [{ link: 'lift', kind: 'box', size: [sx * 0.5, sy * 0.8, 12], at: [0, 0, -6], mat: 'alu' }];
    for (const ix of [-1, 1]) s.push({ link: 'lift', kind: 'cyl', r: 6, h: 20, at: [ix * sx * 0.2, 0, 10], mat: 'steel', collide: false });
    return s;
  },
  init: () => ({ x: 0, uid: null }),
  step(/** @type {any} */ s, /** @type {any} */ p, /** @type {any} */ io, /** @type {number} */ dt) {
    const ms = dt * 1000, up = !!io.clamp;
    const target = up ? p.stroke : 0, v = p.stroke / (up ? p.upMs : p.dnMs);
    if (s.x < target) s.x = Math.min(target, s.x + v * ms);
    else if (s.x > target) s.x = Math.max(target, s.x - v * ms);
    io.up = s.x >= p.stroke - p.band;
    io.dn = s.x <= p.band;
    io.present = !!s.uid;
  },
  check: (/** @type {any} */ p) => (p.stroke > p.band ? [] : ['stroke must be longer than the switch band']),
};

// ---------------------------------------------------------------------------- part sensors
// About PARTS, so the plant answers them with Rapier queries against parts only (never the
// machine) and writes s.hit; the model below turns that into the output the PLC reads:
// NO/NC and an off-delay, like the setting on a real Omron/Keyence sensor. One step (2 ms) of
// lag, then the scene's minPulseMs hold as for any sensor.

/** @param {any} s @param {any} p @param {any} io @param {number} dt */
function senseStep(s, p, io, dt) {
  if (s.hit) { s.on = true; s.offT = p.offDelayMs; }
  else if (s.on) { s.offT -= dt * 1000; if (s.offT <= 0) s.on = false; }
  io.out = p.logic === 'NC' ? !s.on : s.on;
}
const SENSE = [
  { k: 'logic', type: 'enum', of: ['NO', 'NC'], def: 'NO' },
  { k: 'offDelayMs', type: 'num', def: 0, min: 0, unit: 'ms' },
];

const photoEye = {
  label: 'Photoelectric sensor', group: 'sensors', sense: 'ray',
  // The beam leaves the lens (frame origin) along +X. Diffuse or through-beam are the same ray
  // here: `range` is the reach, or the distance to the receiver.
  params: [{ k: 'range', type: 'num', def: 300, min: 1, unit: 'mm' }, ...SENSE],
  io: () => ({ out: { dir: 'in', type: 'BOOL', dev: 'PH', words: 'EXIST' } }),
  links: () => ({ body: {} }),
  sockets: () => ({}),
  shapes: (/** @type {any} */ p) => [
    { link: 'body', kind: 'box', size: [18, 30, 22], at: [-9, 0, 0], mat: 'dark', collide: false },
    { link: 'body', kind: 'cyl', r: 1.5, h: p.range, at: [p.range / 2, 0, 0], rot: [0, 90, 0], mat: 'reed', glow: 'out', collide: false },
  ],
  init: () => ({ hit: false, on: false, offT: 0 }),
  step: senseStep,
};

const proximity = {
  label: 'Proximity sensor', group: 'sensors', sense: 'near',
  // Senses a part within `range` of its face (+X). Inductive = metalOnly (steel, alu).
  params: [{ k: 'range', type: 'num', def: 8, min: 0.5, unit: 'mm' }, { k: 'metalOnly', type: 'bool', def: true }, ...SENSE],
  io: () => ({ out: { dir: 'in', type: 'BOOL', dev: 'PX', words: 'EXIST' } }),
  links: () => ({ body: {} }),
  sockets: () => ({}),
  shapes: () => [
    { link: 'body', kind: 'cyl', r: 6, h: 40, at: [-20, 0, 0], rot: [0, 90, 0], mat: 'steel', collide: false },
    { link: 'body', kind: 'cyl', r: 6.5, h: 3, at: [-1.5, 0, 0], rot: [0, 90, 0], mat: 'reed', glow: 'out', collide: false },
  ],
  init: () => ({ hit: false, on: false, offT: 0 }),
  step: senseStep,
};

export const TYPES = /** @type {Record<string, any>} */ ({ frame, plate, cylinder, servoLinear, pushbutton, lamp, workpiece, conveyor, emitter, remover, photoEye, proximity, vacuumCup, gripper, nest, indexTable, pallet, palletLift });
