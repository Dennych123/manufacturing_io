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

export const TYPES = /** @type {Record<string, any>} */ ({ frame, plate, cylinder, servoLinear, pushbutton, lamp, workpiece });
