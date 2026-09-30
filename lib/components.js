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

/**
 * Speed override, as the percentage dial on a cell panel: it scales the axes' SPEED, never their
 * acceleration and never the pneumatics (a cylinder's speed is set by its flow regulator, and no
 * dial on the panel changes it). An unbound or zero `ovr` means full speed, so a scene that does
 * not have a dial behaves exactly as before.
 * @param {any} v @returns {number} 0.01 .. 1
 */
export const ovrK = v => (typeof v === 'number' && v > 0 ? clamp(v, 1, 100) : 100) / 100;

export const COLORS = /** @type {Record<string, string>} */ ({ green: '#27b045', red: '#e0322c', yellow: '#f2c21b', blue: '#2a6fdb',
                        white: '#eeeeee', amber: '#ff9a1f', black: '#222222' });
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
  params: [
    { k: 'size', type: 'vec3', def: [900, 600, 800], unit: 'mm' },
    // solid: a closed cabinet instead of an open table. A cell of six machines drawn as tables is
    // a thicket of legs, and the thing that has to be readable in it is WHICH machine the arm is
    // over. rb4axis draws its stations as one coloured box for that reason.
    { k: 'style', type: 'enum', of: ['table', 'solid'], def: 'table' },
    { k: 'color', type: 'str', def: '' },
  ],
  links: () => ({ body: {} }),
  sockets: (/** @type {any} */ p) => ({ top: { link: 'body', at: [0, 0, p.size[2]] } }),
  shapes: (/** @type {any} */ p) => {
    const [sx, sy, sz] = p.size, leg = 40, top = 20;
    if (p.style === 'solid') {
      // The top plate overhangs by 10 mm a side and is counted INSIDE sz, so `top` stays at sz:
      // whatever is mounted on it (a nest, a cover hinge) sits at the same height as on a table.
      const plate = Math.min(16, sz / 2);          // a frame shorter than the plate would give a negative box
      return [
        { link: 'body', kind: 'box', size: [sx, sy, sz - plate], at: [0, 0, (sz - plate) / 2], mat: 'paint', ...(p.color ? { color: p.color } : {}) },
        { link: 'body', kind: 'box', size: [sx + 20, sy + 20, plate], at: [0, 0, sz - plate / 2], mat: 'alu', color: '#cbd5e1' },
      ];
    }
    const s = [{ link: 'body', kind: 'box', size: [sx, sy, top], at: [0, 0, sz - top / 2], mat: 'alu', ...(p.color ? { color: p.color } : {}) }];
    for (const [ix, iy] of [[-1, -1], [1, -1], [-1, 1], [1, 1]]) {
      s.push({ link: 'body', kind: 'box', size: [leg, leg, sz - top], at: [ix * (sx / 2 - leg / 2), iy * (sy / 2 - leg / 2), (sz - top) / 2], mat: 'profile' });
    }
    return s;
  },
};

const plate = {
  label: 'Plate / bracket', group: 'structure',
  // hidden: the maker's CAD draws it (a chute sheet, say) and this box is only its collider.
  params: [{ k: 'size', type: 'vec3', def: [200, 100, 12], unit: 'mm' }, { k: 'hidden', type: 'bool', def: false }],
  links: () => ({ body: {} }),
  sockets: (/** @type {any} */ p) => ({ top: { link: 'body', at: [0, 0, p.size[2]] }, bottom: { link: 'body', at: [0, 0, 0], rot: [180, 0, 0] } }),
  shapes: (/** @type {any} */ p) => [{ link: 'body', kind: 'box', size: p.size, at: [0, 0, p.size[2] / 2], mat: 'alu', ...(p.hidden ? { draw: false } : {}) }],
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

/**
 * Exact piecewise travel for `t` ms: full speed, then cushion speed near the end.
 * @param {number} x @param {number} dir @param {number} t @param {number} stroke
 * @param {number} v @param {number} c @param {number} k @returns {number}
 */
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
    // `knife` is an escapement blade: a flat blade with a ROUND nose, which is what lets it go
    // in between two parts that are touching and come out again while one presses on it. A
    // square blade cannot: it meets both parts at once on a flat face, and on the way out the
    // part it is holding jams against its corner. The nose is a half-round of the blade's own
    // thickness, so the wedge is the same on the way in and the way out.
    { k: 'head', type: 'enum', of: ['none', 'plate', 'pin', 'knife'], def: 'none' },
    { k: 'headSize', type: 'vec3', def: [60, 40, 8], unit: 'mm' },
    // hidden: the maker's CAD draws this cylinder (a shell riding its rod), so its primitives are
    // colliders and switch logic only. They still collide exactly as before.
    { k: 'hidden', type: 'bool', def: false },
    // Where the cylinder is at power-up. A lift whose HOME is up starts up: the machine was left
    // homed, and a homing routine that never commands it (the add-on's tray) would wait for ever.
    { k: 'start', type: 'enum', of: ['ret', 'ext'], def: 'ret' },
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
    /** @type {any[]} */
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
    if (p.head === 'plate') s.push({ link: 'rod', kind: 'box', size: [hx, hy, hz], at: [0, 0, 27 + hz / 2], mat: 'tool' });
    if (p.head === 'pin') s.push({ link: 'rod', kind: 'cyl', r: hx / 2, h: hz, at: [0, 0, 27 + hz / 2], mat: 'tool' });
    // knife: headSize is [thickness, height across the lane, reach along the rod]. The blade
    // stops hx/2 short and a round nose of that radius finishes it, so the tip is a wedge.
    if (p.head === 'knife') {
      s.push({ link: 'rod', kind: 'box', size: [hx, hy, hz - hx / 2], at: [0, 0, 27 + (hz - hx / 2) / 2], mat: 'tool' });
      s.push({ link: 'rod', kind: 'cyl', r: hx / 2, h: hy, at: [0, 0, 27 + hz - hx / 2], rot: [90, 0, 0], mat: 'tool' });
    }
    return p.hidden ? s.map(x => ({ ...x, draw: false })) : s;
  },
  states: ['retracted', 'extending', 'extended', 'retracting'],
  init: (/** @type {any} */ p) => {
    const rest = p.start === 'ext' ? 1 : p.valve === '5/3-closed' ? 0 : -1;
    return { x: p.start === 'ext' ? p.stroke : 0, spool: rest, pend: rest, tv: 0, sw: /** @type {Record<string, boolean>} */ ({}) };
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
    // Jog speed as a percentage of vmax. A pendant does not jog at the rate a cycle runs at:
    // 300 mm/s crosses a 500 mm stroke in under two seconds, which cannot be placed by hand.
    { k: 'jogPct', type: 'num', def: 10, min: 0.1, max: 100, unit: '%' },
    // hidden: drawn by the maker's CAD instead (see cylinder).
    { k: 'hidden', type: 'bool', def: false },
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
    // Speed override, in percent. Unbound means full speed.
    ovr: { dir: 'out', type: 'LREAL' },
    // Jog: the axis creeps while one of these is held, at the override speed (MC_Jog).
    jogP: { dir: 'out', type: 'BOOL', dev: 'MC', words: 'JOG_P' },
    jogN: { dir: 'out', type: 'BOOL', dev: 'MC', words: 'JOG_N' },
  }),
  links: (/** @type {any} */ p) => {
    const [L, , H] = p.body, Lc = L - p.stroke;
    return { body: {}, carriage: { at: [-L / 2 + Lc / 2, 0, 0.6 * H], dof: 'prismatic', axis: [1, 0, 0] } };
  },
  sockets: (/** @type {any} */ p) => ({ carriage: { link: 'carriage', at: [0, 0, 0.4 * p.body[2]] }, base: { link: 'body', at: [0, 0, 0] } }),
  shapes: (/** @type {any} */ p) => {
    const [L, W, H] = p.body, Lc = L - p.stroke;
    const s = [
      { link: 'body', kind: 'box', size: [L, W, 0.6 * H], at: [0, 0, 0.3 * H], mat: 'motion' },
      { link: 'body', kind: 'box', size: [70, 0.8 * W, 0.8 * W], at: [L / 2 + 35, 0, 0.3 * H], mat: 'dark' },
      { link: 'carriage', kind: 'box', size: [Lc - 10, W + 10, 0.4 * H], at: [0, 0, 0.2 * H], mat: 'steel' },
    ];
    return p.hidden ? s.map(x => ({ ...x, draw: false })) : s;
  },
  init: () => ({ x: 0, v: 0, tgt: 0, exec: false, moving: false, arrived: false }),
  step(/** @type {any} */ s, /** @type {any} */ p, /** @type {any} */ io, /** @type {number} */ dt) {
    const e = !!io.exec;
    if (e && !s.exec) { s.tgt = clamp(Number(io.target) || 0, 0, p.stroke); s.moving = true; s.arrived = false; }
    s.exec = e;
    // Jog, and only while no move is running: two sources of motion for one axis is how a machine
    // gets broken. Both buttons at once is a stop, as on a real pendant.
    const jp = !!io.jogP, jn = !!io.jogN;
    if (!e && jp !== jn) {
      s.moving = false; s.arrived = false; s.v = 0;
      s.x = clamp(s.x + (jp ? 1 : -1) * p.vmax * (p.jogPct / 100) * ovrK(io.ovr) * dt, 0, p.stroke);
    }
    if (s.moving) {
      const r = trapStep(s.x, s.tgt, s.v, p.vmax * ovrK(io.ovr), p.acc, dt);
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

/** Axis letters to unit vectors, for the generic joint. Signed, because a URDF says `-1 0 0`. */
const AX = /** @type {Record<string, number[]>} */ ({ x: [1, 0, 0], y: [0, 1, 0], z: [0, 0, 1], '-x': [-1, 0, 0], '-y': [0, -1, 0], '-z': [0, 0, -1] });

/**
 * Euler [rx, 0, rz] (degrees, the lib's X-then-Y-then-Z fixed-axis rule) that turns +Y into the
 * direction of v. A joint draws its arm along +Y; a real robot's next joint is rarely there.
 * @param {number[]} v
 */
function eulerToY(v) {
  const n = Math.hypot(v[0], v[1], v[2]) || 1, u = v.map(x => x / n);
  const rx = Math.asin(Math.max(-1, Math.min(1, u[2]))), c = Math.cos(rx);
  const rz = Math.abs(c) < 1e-9 ? 0 : Math.atan2(-u[0], u[1]);
  return [rx * 180 / Math.PI, 0, rz * 180 / Math.PI];
}

/** Where a revolute joint's arm ends: `to` when given, else `len` along +Y. @param {any} p */
const jointTo = p => (p.to && p.to.some((/** @type {number} */ v) => v !== 0) ? p.to : [0, p.len, 0]);

const joint = {
  label: 'Joint (servo axis)', group: 'motion',
  // The plant keeps ONE dof per component, so a robot arm is a CHAIN of joints, each mounted on
  // the previous one's `end` socket. That is also how rb4axis composes its arm: every angle is
  // relative to its parent (a2 = a1 + pos[2] in chainPoints), which a parent/child mount gives
  // for nothing. Each joint carries the arm that hangs off it, drawn along +Y.
  //
  // Revolute values are DEGREES: worldPoses() turns the dof straight into qaxis(axis, x), and it
  // does not clamp - the limits live here, as they do on a real drive.
  params: [
    { k: 'kind', type: 'enum', of: ['revolute', 'prismatic'], def: 'revolute' },
    { k: 'axis', type: 'enum', of: ['x', 'y', 'z', '-x', '-y', '-z'], def: 'x' },
    { k: 'len', type: 'num', def: 300, min: 0, unit: 'mm' },
    // Where the NEXT joint sits, in this joint's moving frame - a URDF child origin. A 6-axis
    // arm's links are not laid end to end along one axis: the LR Mate's J2 sits 50 mm out from
    // J1, its J4 35 mm above J3. Zero means "use len along +Y", as the rb4axis chain does.
    { k: 'to', type: 'vec3', def: [0, 0, 0], unit: 'mm' },
    { k: 'min', type: 'num', def: -120 },
    { k: 'max', type: 'num', def: 120 },
    { k: 'home', type: 'num', def: 0 },
    { k: 'vmax', type: 'num', def: 120, min: 0.1, unit: 'deg/s or mm/s' },
    { k: 'acc', type: 'num', def: 320, min: 0.1, unit: 'deg/s² or mm/s²' },
    { k: 'band', type: 'num', def: 0.05, min: 0 },
    { k: 'width', type: 'num', def: 90, min: 5, unit: 'mm' },
    // plate: the same revolute axis drawn as a hinged door - a flat leaf the full `width` across
    // the hinge, `len` deep and `thick` thick, hanging off a hinge rod. A machine cover IS a
    // servo axis with an angle the sequence waits on, so it is this type and not a new one.
    { k: 'arm', type: 'enum', of: ['bar', 'plate'], def: 'bar' },
    { k: 'thick', type: 'num', def: 18, min: 1, unit: 'mm' },
    { k: 'color', type: 'str', def: '' },
    // The maker's own shell for this link, drawn instead of the primitive: an asset path under
    // assets/ (an STL from the robot's URDF package). It is DRAWN only - the primitive stays as
    // the collider, because a trimesh does not collide with a trimesh in Rapier and the part
    // sensors would have to ray-trace a 5000-triangle shell to answer "is something there".
    { k: 'mesh', type: 'str', def: '' },
    // Mesh units to mm. A URDF is in metres, so 1000; a maker's STEP export is already mm, so 1.
    { k: 'meshScale', type: 'num', def: 1000, min: 1e-6 },
    // Where the shell sits in the LINK's frame. A URDF visual needs none - it is authored in the
    // link frame - but a maker ships one file per axis in ITS OWN frame, which is rarely the frame
    // the kinematics were built in. These two put it right without touching the kinematics.
    { k: 'meshAt', type: 'vec3', def: [0, 0, 0], unit: 'mm' },
    { k: 'meshRot', type: 'vec3', def: [0, 0, 0], unit: 'deg' },
    // Jog speed as a percentage of vmax. A teach pendant does not jog at full rate: J1 at 450
    // deg/s crosses its whole range in 0.75 s, which is unusable by hand and looks like a fault.
    { k: 'jogPct', type: 'num', def: 10, min: 0.1, max: 100, unit: '%' },
  ],
  // Execute is a LEVEL held until Done, as on the servo: a one-scan pulse falls between two OPC UA
  // samples. Done/Busy/InPos are replies, so they are exempt from the minPulseMs hold.
  // jogP/jogN/ovr as on the servo, so a joint can be jogged from the panel and dragged by hand.
  io: () => ({
    target: { dir: 'out', type: 'LREAL' }, exec: { dir: 'out', type: 'BOOL' },
    done: { dir: 'in', type: 'BOOL', hold: false }, busy: { dir: 'in', type: 'BOOL', hold: false },
    actPos: { dir: 'in', type: 'LREAL' }, inPos: { dir: 'in', type: 'BOOL', hold: false },
    ovr: { dir: 'out', type: 'LREAL', dev: 'MC', words: 'OVR' },
    jogP: { dir: 'out', type: 'BOOL', dev: 'MC', words: 'JOG_P' },
    jogN: { dir: 'out', type: 'BOOL', dev: 'MC', words: 'JOG_N' },
  }),
  links: (/** @type {any} */ p) => ({ base: {}, arm: { dof: p.kind, axis: AX[p.axis] } }),
  sockets: (/** @type {any} */ p) => ({
    // A revolute joint hands the next one the far end of its arm; a prismatic one hands over its
    // carriage, which is the moving link's own origin.
    end: { link: 'arm', at: p.kind === 'revolute' ? jointTo(p) : [0, 0, 0] },
    base: { link: 'base', at: [0, 0, 0] },
  }),
  shapes: (/** @type {any} */ p) => {
    const w = p.width, a = AX[p.axis];
    /** @type {any[]} */
    const s = [];
    if (p.kind === 'revolute') {
      // the hub turns about `axis`; scene cylinders run along Z, so lie it down for x or y
      const ax = p.axis.replace('-', '');
      const rot = ax === 'x' ? [0, 90, 0] : ax === 'y' ? [90, 0, 0] : [0, 0, 0];
      // The maker's shell for this link, in the link's OWN frame - a URDF's visual mesh is
      // expressed exactly there, so it needs no offset. Drawn, never collided (see the param).
      // Hoisted above every arm style on purpose: the last joint of a wrist has `to` [0,0,0] and
      // no arm to draw, and a mesh emitted inside the arm branch would silently skip that link.
      if (p.mesh) s.push({ link: 'arm', kind: 'mesh', asset: p.mesh, scale: p.meshScale, at: p.meshAt, rot: p.meshRot, mat: 'alu', ...(p.color ? { color: p.color } : {}), collide: false });
      const to = jointTo(p), L = Math.hypot(to[0], to[1], to[2]);
      if (p.to.some((/** @type {number} */ v) => v !== 0)) {
        // the arm runs from the hub to wherever the next joint is
        s.push({ link: 'base', kind: 'cyl', r: w * 0.42, h: w * 0.7, at: [0, 0, 0], rot, mat: 'dark', collide: false, draw: !p.mesh });
        if (L > 0) s.push({ link: 'arm', kind: 'box', size: [w * 0.62, L, w * 0.62], at: to.map((/** @type {number} */ v) => v / 2), rot: eulerToY(to), mat: 'alu', ...(p.color ? { color: p.color } : {}), draw: !p.mesh });
        return s;
      }
      if (p.arm === 'plate') {
        // The hinge rod runs the full span; the leaf hangs off it and lies ON it when shut, so it
        // is offset by half its thickness and never straddles the hinge line.
        //
        // The leaf does NOT collide. It sweeps the whole machine top, which is where the nest and
        // the part it holds are: a solid leaf would make physics answer a question the interlock
        // already answers (the sequence keeps the arm out of a shut machine), and a kinematic leaf
        // closing onto a part on a kinematic nest is the eject case in CLAUDE.md.
        s.push({ link: 'base', kind: 'cyl', r: p.thick * 0.8, h: w, at: [0, 0, 0], rot, mat: 'dark', collide: false });
        if (p.len > 0) s.push({ link: 'arm', kind: 'box', size: [w, p.len, p.thick], at: [0, p.len / 2, p.thick / 2],
                                mat: 'paint', color: p.color || '#93c5fd', collide: false, opacity: 0.55 });
        return s;
      }
      s.push({ link: 'base', kind: 'cyl', r: w * 0.42, h: w * 0.7, at: [0, 0, 0], rot, mat: 'dark', collide: false, draw: !p.mesh });
      if (p.len > 0) s.push({ link: 'arm', kind: 'box', size: [w * 0.62, p.len, w * 0.62], at: [0, p.len / 2, 0], mat: 'alu', ...(p.color ? { color: p.color } : {}), draw: !p.mesh });
    } else {
      const L = Math.abs(p.max - p.min) + p.len;
      s.push({ link: 'base', kind: 'box', size: [a[0] ? L : w, a[1] ? L : w, a[2] ? L : w * 0.5],
               at: [a[0] * (p.min + p.max) / 2, a[1] * (p.min + p.max) / 2, a[2] * (p.min + p.max) / 2 - (a[2] ? 0 : w * 0.35)],
               mat: 'profile' });
      s.push({ link: 'arm', kind: 'box', size: [w, w, w * 0.6], at: [0, 0, w * 0.3], mat: 'steel' });
    }
    return s;
  },
  init: (/** @type {any} */ p) => ({ x: p.home, v: 0, tgt: p.home, exec: false, moving: false, arrived: true }),
  step(/** @type {any} */ s, /** @type {any} */ p, /** @type {any} */ io, /** @type {number} */ dt) {
    const e = !!io.exec;
    if (e && !s.exec) { s.tgt = clamp(Number(io.target) || 0, p.min, p.max); s.moving = true; s.arrived = false; }
    s.exec = e;
    // Jog, only while no move runs (two sources of motion for one axis is how a machine gets
    // broken); both buttons at once is a stop, as on a real pendant. The limits still apply.
    const jp = !!io.jogP, jn = !!io.jogN;
    if (!e && jp !== jn) {
      s.moving = false; s.arrived = false; s.v = 0;
      // clamped to the axis limits, exactly as a move is: a pendant cannot jog past a soft limit.
      s.x = clamp(s.x + (jp ? 1 : -1) * p.vmax * (p.jogPct / 100) * ovrK(io.ovr) * dt, p.min, p.max);
    }
    if (s.moving) {
      const r = trapStep(s.x, s.tgt, s.v, p.vmax * ovrK(io.ovr), p.acc, dt);
      s.x = r.pos; s.v = r.vel;
      if (!r.moving) { s.moving = false; s.arrived = true; }
    }
    io.done = e && s.arrived;
    io.busy = s.moving;
    io.actPos = Math.round(s.x * 1000) / 1000;
    io.inPos = !s.moving && Math.abs(s.x - s.tgt) <= p.band;
  },
  check(/** @type {any} */ p) {
    const e = [];
    if (p.min >= p.max) e.push('min must be below max');
    if (p.home < p.min || p.home > p.max) e.push('home must lie between min and max');
    return e;
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
    ovr: { dir: 'out', type: 'LREAL' },
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
      { link: 'table', kind: 'cyl', r, h: 20, at: [0, 0, p.height + 10], mat: 'motion' },
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
      s.cam += dt * 1000 / p.camMs * ovrK(io.ovr);
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

// A 2-position selector switch, AUTO / MANUAL, as on a Ndeso-style operator panel (rb4axis).
// The knob turns (revolute DOF), so the mode is visible in 3D. It starts on AUTO. The PLC
// enforces what each mode allows: START only in AUTO, the individual buttons only in MANUAL,
// and a change while the sequence runs stops it (FAULT, START acknowledges).
const selector = {
  label: 'Selector AUTO/MANUAL', group: 'operator',
  params: [{ k: 'color', type: 'enum', of: COLOR_NAMES, def: 'black' }],
  io: () => ({ sel: { dir: 'in', type: 'BOOL', dev: 'SS', words: 'AUTO' } }),
  links: () => ({ body: {}, knob: { at: [0, 0, 20], dof: 'revolute', axis: [0, 0, 1] } }),
  sockets: () => ({}),
  shapes: (/** @type {any} */ p) => [
    { link: 'body', kind: 'cyl', r: 16, h: 20, at: [0, 0, 10], mat: 'dark' },
    { link: 'knob', kind: 'cyl', r: 12, h: 6, at: [0, 0, 3], mat: 'paint', color: COLORS[p.color], collide: false },
    { link: 'knob', kind: 'box', size: [26, 6, 8], at: [0, 0, 8], mat: 'paint', color: COLORS[p.color], collide: false },
  ],
  pressKey: 'sel',
  init: () => ({ x: 45, on: true }),
  press(/** @type {any} */ s, /** @type {any} */ p, /** @type {string} */ key, /** @type {boolean} */ down) {
    if (key === 'sel' && down) s.on = !s.on;
  },
  step(/** @type {any} */ s, /** @type {any} */ p, /** @type {any} */ io) {
    io.sel = s.on;
    s.x = s.on ? 45 : -45;                             // knob to the right = AUTO, left = MANUAL
  },
};

// The percentage dial of a cell panel (rb4axis SIM_OVR). It is an operator INPUT: the plant tells
// the PLC what the dial says, and the PLC passes it on to the axes. The browser draws it as a
// slider in the operator panel and sends the value on change, never while dragging.
const speedDial = {
  label: 'Speed override', group: 'operator',
  params: [{ k: 'min', type: 'num', def: 10, min: 1, max: 100, unit: '%' }],
  io: () => ({ ovr: { dir: 'in', type: 'LREAL', dev: 'SS', words: 'OVR' } }),
  links: () => ({ body: {}, knob: { at: [0, 0, 20], dof: 'revolute', axis: [0, 0, 1] } }),
  sockets: () => ({}),
  shapes: () => [
    { link: 'body', kind: 'cyl', r: 18, h: 20, at: [0, 0, 10], mat: 'dark' },
    { link: 'knob', kind: 'cyl', r: 13, h: 6, at: [0, 0, 3], mat: 'paint', color: COLORS.white, collide: false },
    { link: 'knob', kind: 'box', size: [22, 5, 8], at: [7, 0, 8], mat: 'paint', color: COLORS.black, collide: false },
  ],
  /** The io key the browser's slider writes. */
  dialKey: 'ovr',
  init: () => ({ x: 0, v: 100 }),
  dial(/** @type {any} */ s, /** @type {any} */ p, /** @type {string} */ key, /** @type {number} */ v) {
    if (key === 'ovr') s.v = clamp(Number(v) || 0, p.min, 100);
  },
  step(/** @type {any} */ s, /** @type {any} */ p, /** @type {any} */ io) {
    io.ovr = s.v;
    s.x = (s.v - 100) * 2.4;                       // the knob sweeps 216 degrees over the range
  },
};

const lamp = {
  label: 'Pilot lamp', group: 'operator',
  // buzzer: the output is a buzzer - the viewer sounds it while it is on.
  params: [{ k: 'color', type: 'enum', of: COLOR_NAMES, def: 'green' }, { k: 'buzzer', type: 'bool', def: false }],
  io: () => ({ lamp: { dir: 'out', type: 'BOOL', dev: 'PL', words: 'LAMP' } }),
  links: () => ({ body: {} }),
  sockets: () => ({}),
  shapes: (/** @type {any} */ p) => [
    { link: 'body', kind: 'cyl', r: 15, h: 14, at: [0, 0, 7], mat: 'dark' },
    { link: 'body', kind: 'sphere', r: 11, at: [0, 0, 16], mat: 'paint', color: COLORS[p.color], glow: 'lamp', collide: false },
  ],
};

// ---------------------------------------------------------------------------- items

/**
 * kg/m³ by workpiece material: Rapier mass comes from collider volume × density.
 * `carton` is a filled cardboard box, which is mostly air: the Open Industry Project's standard
 * carton is 600 x 400 x 400 at 10 kg, so 104 kg/m³. Taking plastic for it would have made that
 * box weigh 115 kg, and a sorter is built around what a carton weighs.
 */
export const DENSITY = /** @type {Record<string, number>} */ ({ steel: 7850, alu: 2700, plastic: 1200, carton: 104 });

const workpiece = {
  label: 'Workpiece', group: 'items',
  // dynamic: a loose Rapier body that falls, slides and gets pushed. Static (the default) is a
  // fixed body, e.g. a part that only sits in a press.
  params: [
    // pin: a headed pin that HANGS by its head (a spark plug's centre electrode on a rail track).
    // size is [head diameter, shank diameter, overall length], and headH is the head's height.
    { k: 'kind', type: 'enum', of: ['box', 'cyl', 'pin'], def: 'box' },
    { k: 'size', type: 'vec3', def: [60, 40, 30], unit: 'mm' },
    { k: 'headH', type: 'num', def: 2, min: 0.1, unit: 'mm' },
    { k: 'color', type: 'str', def: '#c79a52' },
    { k: 'material', type: 'enum', of: ['steel', 'alu', 'plastic', 'carton'], def: 'steel' },
    { k: 'dynamic', type: 'bool', def: false },
    { k: 'friction', type: 'num', def: 0.5, min: 0 },
    { k: 'restitution', type: 'num', def: 0.1, min: 0, max: 1 },
    // The product as the maker's CAD draws it: [{asset, color}] in the part's own frame (origin at
    // its underside centre). Drawn only - the primitive above is still what collides, and what is
    // drawn when an asset does not load. An M&B drawn as a plain disc hides its connector, which is
    // the very thing the turntable is turning it to.
    { k: 'meshes', type: 'json', def: [] },
  ],
  part: true,
  links: () => ({ body: {} }),
  sockets: (/** @type {any} */ p) => ({ top: { link: 'body', at: [0, 0, p.size[2]] } }),
  shapes: (/** @type {any} */ p) => {
    const prim = workpiecePrim(p);
    const ms = Array.isArray(p.meshes) ? p.meshes.filter((/** @type {any} */ m) => m && m.asset) : [];
    if (!ms.length) return prim;
    return [...prim.map(s => ({ ...s, draw: false })),
      ...ms.map((/** @type {any} */ m) => ({ link: 'body', kind: 'mesh', asset: m.asset, at: [0, 0, 0], mat: 'part', color: m.color || p.color, collide: false }))];
  },
  // A zero-size part has no collider radius, and the emitter's column check then loops for ever.
  check: (/** @type {any} */ p) => {
    const e = p.size.every((/** @type {number} */ v) => v > 0) ? [] : ['size must be positive on every axis'];
    if (p.kind === 'pin' && !(p.size[1] < p.size[0])) e.push('a pin hangs by its head: size[1] (shank) must be thinner than size[0] (head)');
    if (p.kind === 'pin' && !(p.headH < p.size[2])) e.push('headH must be shorter than the pin');
    return e;
  },
};

/** A workpiece's own primitive shapes (what collides). @param {any} p */
function workpiecePrim(p) {
  {
    if (p.kind === 'pin') {
      // Origin at the tip, like every part's underside; the shank comes first so the part's
      // centre (the first solid shape) is the shank's middle.
      const [hd, sd, L] = p.size, hh = p.headH;
      return [
        { link: 'body', kind: 'cyl', r: sd / 2, h: L - hh, at: [0, 0, (L - hh) / 2], mat: 'part', color: p.color },
        { link: 'body', kind: 'cyl', r: hd / 2, h: hh, at: [0, 0, L - hh / 2], mat: 'part', color: p.color },
      ];
    }
    return [p.kind === 'cyl'
      ? { link: 'body', kind: 'cyl', r: p.size[0] / 2, h: p.size[2], at: [0, 0, p.size[2] / 2], mat: 'part', color: p.color }
      : { link: 'body', kind: 'box', size: p.size, at: [0, 0, p.size[2] / 2], mat: 'part', color: p.color }];
  }
}

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
    // hidden: the maker's CAD draws the conveyor; the belt, members and legs still collide.
    { k: 'hidden', type: 'bool', def: false },
    // centering: guides that funnel a part to the belt's centre line, at up to this lateral speed.
    { k: 'centering', type: 'num', def: 0, min: 0, unit: 'mm/s' },
  ],
  io: () => ({ run: { dir: 'out', type: 'BOOL' }, rev: { dir: 'out', type: 'BOOL' }, ovr: { dir: 'out', type: 'LREAL' } }),
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
      // The side members sit BELOW the belt surface, as on a real conveyor without guides. Flush
      // with it, a part pushed sideways slides belt -> rail top -> off the rail's far edge, and
      // Rapier 0.20 can keep the rail-top manifold after the part has left it: measured on
      // sort-by-material, steel #43 of 43 came to rest 12 mm BESIDE the rail at belt height, with
      // 4 contacts at -0.03 mm, normal +Z, and never fell (the other 42 fell at once). With the
      // top T below the belt, a part leaves the belt edge into free air and there is no edge to
      // hang on. tests/lib.test.js pins it.
      s.push({ link: 'body', kind: 'box', size: [L, 30, 60], at: [0, iy * (W / 2 + 15), H - T - 30], mat: 'profile' });
      if (p.guides > 0) s.push({ link: 'body', kind: 'box', size: [L, 6, p.guides], at: [0, iy * (W / 2 + 3), H + p.guides / 2], mat: 'alu' });
      for (const ix of [-1, 1]) s.push({ link: 'body', kind: 'box', size: [leg, leg, H - 60], at: [ix * (L / 2 - leg), iy * (W / 2 + 15), (H - 60) / 2], mat: 'profile' });
    }
    for (let i = 0; i < Math.floor((L - 20) / STRIPE); i++) {
      s.push({ link: 'stripes', kind: 'box', size: [8, W - 12, 1], at: [-L / 2 + i * STRIPE + 6, 0, 0.5], mat: 'paint', color: '#5b6068', collide: false });
    }
    return p.hidden ? s.map(x => ({ ...x, draw: false })) : s;
  },
  init: () => ({ x: 0, v: 0 }),
  step(/** @type {any} */ s, /** @type {any} */ p, /** @type {any} */ io, /** @type {number} */ dt) {
    const want = io.run ? (io.rev ? -p.speed : p.speed) * ovrK(io.ovr) : 0;
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
    // A part put down by hand lands at any angle: each copy is turned by up to this about Z.
    { k: 'jitterDeg', type: 'num', def: 0, min: 0, max: 180, unit: 'deg' },
    // A pallet loader: successive parts land on a grid around the emitter instead of all on one
    // spot, so one component fills a whole tray.
    { k: 'gridCols', type: 'num', def: 1, min: 1 },
    { k: 'gridRows', type: 'num', def: 1, min: 1 },
    { k: 'gridPitch', type: 'num', def: 0, min: 0, unit: 'mm' },
    // A pallet is not always square. 0 means "the same as gridPitch", so a square grid is
    // written the way it always was: the robot cell's pallet has 100 mm columns (the head's cup
    // spacing, which is fixed by the mechanism) and 40 mm rows (as many as the arm can reach).
    { k: 'gridPitchY', type: 'num', def: 0, min: 0, unit: 'mm' },
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
    // Switched off, nothing is pending: a feeder holding one last request kept the plant testing a
    // spawn spot every step for ever, long after the PLC had stopped asking for parts.
    else s.req = s.done;
    if (p.max > 0) s.req = Math.min(s.req, p.max);
    // A feeder that cannot drop a part does NOT remember the ones it missed: at most one is
    // pending. Measured on the palletizing loader, which banked a backlog while the tray was full
    // and then refilled every hole the machine emptied, for 1700 plugs and no empty pallet ever.
    if (p.mode !== 'tag') s.req = Math.min(s.req, s.done + 1);
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

// ---------------------------------------------------------------------------- tracks
// A rail track that tiny headed pins HANG in and slide down under gravity: a spark plug's centre
// electrode (shank 1.9 mm, head 2.73 mm) between two rails 2.5 mm apart, the head resting on a
// 0.4 mm ledge each side. Rapier cannot do that, and this is measured, not assumed: on Rapier 0.20
// such a pin falls straight through the rails (gone in 0.2 s at lengthUnit 1, in 0.8 s at 0.01),
// and even with the head collider widened until it holds, a queue of 30 flips its pins over (pitch
// 1.7 mm against a 2.73 mm head) at 2.4 ms a step against a 2 ms dt, where this machine carries
// several hundred. tests/rapier.test.js pins the trap. So a pin on a track is a KINEMATIC part the
// track moves along one coordinate per lane, as an Emulate3D track does: it is still a real part,
// so the photo-eyes see it by ray, the hand can take it and the viewer streams it like any other.
// It only becomes Rapier-dynamic when it leaves the track (the escapement drops it into a pipe).
//
// The frame's origin is the UPPER end of the rail tops, on the lane-offset origin; a lane runs
// along local +X, falling at `angle`, and lanes sit at `lanes` offsets along local Y.

/**
 * One step of one lane's queue: every pin slides down at a = g(sin - mu cos), capped at vmax, and
 * stops at whatever is ahead of it - the pin in front one pitch away, a closed gate, or the end.
 * Pins are ordered FRONT first. A pin the gate clamped (hold) does not move and is an obstacle.
 * Pure: the plant calls it, and tests/lib.test.js pins it.
 * @param {Array<{s: number, v: number, hold?: any}>} pins
 * @param {number} lim how far the front pin may go (its centre), mm
 * @param {number[]} gates positions of CLOSED gates on this lane, mm
 * @param {{a: number, vmax: number, dt: number, pitch: number}} k
 */
export function laneAdvance(pins, lim, gates, k) {
  let ahead = lim;
  for (const p of pins) {
    if (p.hold) { p.v = 0; ahead = p.s - k.pitch; continue; }
    let limit = ahead;
    // A gate stops a pin that is still upstream of it; one already past it (or under it when it
    // closed, which is `hold`) is not its business.
    for (const g of gates) if (p.s <= g - k.pitch / 2 + 1e-9) limit = Math.min(limit, g - k.pitch / 2);
    p.v = Math.min(k.vmax, p.v + k.a * k.dt);
    let ns = p.s + p.v * k.dt;
    if (ns >= limit) { ns = Math.max(p.s, limit); p.v = 0; }
    p.s = ns;
    ahead = p.s - k.pitch;
  }
}

/** mm/s² down a rail at `angle` with friction mu; 0 where friction wins. @param {any} p */
export const trackAccel = p => Math.max(0, 9810 * (Math.sin(p.angle * Math.PI / 180) - p.mu * Math.cos(p.angle * Math.PI / 180)));

/** Where lane i's rail top is at distance s down the lane, in the track frame. @param {any} p @param {number} i @param {number} s */
export function lanePoint(p, i, s) {
  const a = p.angle * Math.PI / 180;
  return [s * Math.cos(a), p.lanes[i], -s * Math.sin(a)];
}

const track = {
  label: 'Pin track (hanging rails)', group: 'material flow', flow: 'track',
  params: [
    { k: 'lanes', type: 'json', def: [0] },
    { k: 'length', type: 'num', def: 200, min: 1, unit: 'mm' },
    { k: 'angle', type: 'num', def: 26, min: 0, max: 89, unit: 'deg' },
    { k: 'gap', type: 'num', def: 2.5, min: 0.1, unit: 'mm' },
    { k: 'railW', type: 'num', def: 14, min: 1, unit: 'mm' },
    { k: 'railH', type: 'num', def: 10, min: 1, unit: 'mm' },
    // Centre to centre of two pins that touch: the head diameter.
    { k: 'pitch', type: 'num', def: 3, min: 0.1, unit: 'mm' },
    { k: 'vmax', type: 'num', def: 250, min: 1, unit: 'mm/s' },
    { k: 'mu', type: 'num', def: 0.3, min: 0 },
    // The track the pins run on to. A lane hands its pins over where its end meets one of that
    // track's lane starts (within 1.5 mm, in the world): a shuttle that has moved away is a wall.
    { k: 'next', type: 'ref', of: 'track', def: '' },
    // Bowl feeders: [{ lanes: [lane indices], template: 'part id', rate: pins/s }], each switched
    // by its own io `feed1`, `feed2`, ... A feeder that is blocked does not remember what it missed.
    { k: 'supply', type: 'json', def: [] },
    // Gates across lanes: [{ id: component, at: mm down the lane, lanes: [...], x: mm }]. Closed while
    // that component's dof is >= x. Closing clamps a pin that is under it, as a finger gripper does.
    { k: 'gates', type: 'json', def: [] },
    // The escapement at the lower end: { sep: cylinder, push: cylinder, pushAt: mm }. The separator
    // takes each lane's front pin out of the queue as it starts forward and carries it; when it
    // goes back the pin stays where it was left (the slot); the pusher then carries it and lets it
    // FALL, as a Rapier-dynamic part, once it is pushAt out.
    { k: 'escape', type: 'json', def: {} },
    { k: 'drawRails', type: 'bool', def: true },
  ],
  io: (/** @type {any} */ p) => {
    /** @type {Record<string, any>} */
    const io = { count: { dir: 'in', type: 'UDINT' } };
    (Array.isArray(p.supply) ? p.supply : []).forEach((/** @type {any} */ _, /** @type {number} */ i) => {
      io['feed' + (i + 1)] = { dir: 'out', type: 'BOOL', dev: 'CR', words: 'FEEDER' };
    });
    return io;
  },
  templates: (/** @type {any} */ p) => (Array.isArray(p.supply) ? p.supply.map((/** @type {any} */ x) => x?.template) : []),
  refs: (/** @type {any} */ p) => [
    ...(Array.isArray(p.gates) ? p.gates : []).map((/** @type {any} */ g, /** @type {number} */ i) => ({ id: g?.id, what: 'gates[' + i + '].id' })),
    ...(p.escape?.sep ? [{ id: p.escape.sep, of: 'cylinder', what: 'escape.sep' }] : []),
    ...(p.escape?.push ? [{ id: p.escape.push, of: 'cylinder', what: 'escape.push' }] : []),
    ...(Array.isArray(p.supply) ? p.supply : []).map((/** @type {any} */ x, /** @type {number} */ i) => ({ id: x?.template, of: 'workpiece', what: 'supply[' + i + '].template' })),
  ],
  links: () => ({ body: {} }),
  sockets: (/** @type {any} */ p) => ({ start: { link: 'body', at: [0, 0, 0] }, end: { link: 'body', at: lanePoint({ ...p, lanes: [0] }, 0, p.length) } }),
  shapes: (/** @type {any} */ p) => {
    if (!p.drawRails || !Array.isArray(p.lanes)) return [];
    const a = p.angle * Math.PI / 180, c = Math.cos(a), sn = Math.sin(a);
    /** @type {any[]} */
    const s = [];
    for (const y of p.lanes) {
      for (const side of [-1, 1]) {
        // The rail's centre: half way down the lane, and half its height below the rail top,
        // perpendicular to the slope. Drawn only: nothing loose ever lands on these.
        const m = p.length / 2, h = p.railH / 2;
        s.push({ link: 'body', kind: 'box', size: [p.length, p.railW, p.railH], rot: [0, p.angle, 0],
                 at: [m * c - h * sn, y + side * (p.gap + p.railW) / 2, -m * sn - h * c], mat: 'alu', collide: false });
      }
    }
    return s;
  },
  init: () => ({ n: 0, feed: /** @type {boolean[]} */ ([]) }),
  step(/** @type {any} */ s, /** @type {any} */ p, /** @type {any} */ io) {
    const n = Array.isArray(p.supply) ? p.supply.length : 0;
    for (let i = 0; i < n; i++) s.feed[i] = !!io['feed' + (i + 1)];
    io.count = s.n;
  },
  check(/** @type {any} */ p) {
    const e = [];
    const lanes = p.lanes, nL = Array.isArray(lanes) ? lanes.length : 0;
    if (!nL || !lanes.every((/** @type {any} */ y) => typeof y === 'number' && Number.isFinite(y))) e.push('lanes must be a list of lane offsets in mm');
    const laneList = (/** @type {any} */ l) => Array.isArray(l) && l.length > 0 && l.every(i => Number.isInteger(i) && i >= 0 && i < nL);
    if (!Array.isArray(p.supply)) e.push('supply must be a list');
    else p.supply.forEach((/** @type {any} */ x, /** @type {number} */ i) => {
      if (!x || typeof x.template !== 'string' || !x.template) e.push('supply[' + i + '] must name a template');
      if (!laneList(x?.lanes)) e.push('supply[' + i + '].lanes must be lane indices');
      if (!(x?.rate > 0)) e.push('supply[' + i + '].rate must be pins per second > 0');
    });
    if (!Array.isArray(p.gates)) e.push('gates must be a list');
    else p.gates.forEach((/** @type {any} */ g, /** @type {number} */ i) => {
      if (!g || typeof g.id !== 'string') e.push('gates[' + i + '].id must name a component');
      if (!(g?.at >= 0 && g.at <= p.length)) e.push('gates[' + i + '].at must be within the lane (0..' + p.length + ')');
      if (!laneList(g?.lanes)) e.push('gates[' + i + '].lanes must be lane indices');
      if (typeof g?.x !== 'number') e.push('gates[' + i + '].x must be the dof at which it closes');
    });
    const es = p.escape;
    if (!es || typeof es !== 'object' || Array.isArray(es)) e.push('escape must be an object ({} for none)');
    else if (Object.keys(es).length && !(typeof es.sep === 'string' && typeof es.push === 'string' && es.pushAt > 0)) e.push('escape needs sep, push and pushAt');
    if (p.pitch * 2 > p.length) e.push('a lane must hold at least two pins');
    return e;
  },
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
    // hidden: the maker's CAD draws the cup (or the magnet a cup stands in for).
    { k: 'hidden', type: 'bool', def: false },
  ],
  io: () => ({ on: { dir: 'out', type: 'BOOL', dev: 'SOL', words: 'VAC' }, vac: { dir: 'in', type: 'BOOL', dev: 'VS', words: 'VAC' } }),
  links: () => ({ body: {} }),
  sockets: () => ({}),
  shapes: (/** @type {any} */ p) => [
    { link: 'body', kind: 'cyl', r: p.d / 2, h: 6, at: [0, 0, -3], mat: 'holder', glow: 'vac', collide: false },
    { link: 'body', kind: 'cyl', r: Math.max(2, p.d * 0.25), h: 16, at: [0, 0, -14], mat: 'steel', collide: false },
  ].map(x => (p.hidden ? { ...x, draw: false } : x)),
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

/**
 * The half-angle of a `vee` jaw's notch, from the jaw's own face. A round part seats on the two
 * flanks at +-jawR*sin(VEE_DEG) off centre, so the flatter the vee the wider the jaw has to be to
 * reach them: at 30 degrees a 50 mm casting seats 12.5 mm either side of the middle, inside a jaw
 * the length below already covers.
 */
const VEE_DEG = 30;

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
    // `flat` is a plain pad. `vee` is the lathe-tending jaw: a prismatic notch cut for a part of
    // radius `jawR`, so a round casting seats on two flanks instead of balancing on one flat face.
    { k: 'jaw', type: 'enum', of: ['flat', 'vee'], def: 'flat' },
    { k: 'jawR', type: 'num', def: 25, min: 1, unit: 'mm' },
    { k: 'valve', type: 'enum', of: ['single', 'double'], def: 'single' },
    { k: 'confirm', type: 'enum', of: ['full', 'part'], def: 'full' },
    // hidden: drawn by the maker's CAD (see cylinder); the finger pads still collide.
    { k: 'hidden', type: 'bool', def: false },
    // A 3-jaw gripper (SMC MHSL3): three more finger links, radial at radialDeg + k*120 in the
    // gripper's XY, moving by the same half-opening. They carry the CAD's jaws so a chuck is SEEN;
    // the two pads above are still what collides.
    { k: 'fingers3', type: 'bool', def: false },
    { k: 'radialDeg', type: 'num', def: 90, unit: 'deg' },
  ],
  // `closed` sits at FULL close, so a missed grip reads in the PLC exactly as on a real machine
  // (rb4axis SIM_GRIP_TUTUP): with a part between the fingers it never comes on.
  // A machine builder may set its CHUCK switch at the GRIPPED position instead (the Denso add-on
  // waits for "CHUCK CONFIRM" with the diaphragm in the fingers): `confirm: 'part'` then turns
  // `closed` on when the fingers have stopped ON a part, and never on an empty close.
  // `valve: 'double'` is a 5/2 double-solenoid valve: CHUCK and UNCHUCK are separate coils and
  // the valve keeps the last one that was on - both or neither holds.
  io: (/** @type {any} */ p) => ({
    close: { dir: 'out', type: 'BOOL', dev: 'SOL', words: 'GRIP' },
    ...(p?.valve === 'double' ? { unclose: { dir: 'out', type: 'BOOL', dev: 'SOL', words: 'UNGRIP' } } : {}),
    open: { dir: 'in', type: 'BOOL', dev: 'AS', words: 'OPEN' },
    closed: { dir: 'in', type: 'BOOL', dev: 'AS', words: 'CLOSED' },
  }),
  links: (/** @type {any} */ p) => {
    /** @type {Record<string, any>} */
    const l = { body: {}, fingerL: { dof: 'prismatic', axis: [0, -1, 0] }, fingerR: { dof: 'prismatic', axis: [0, 1, 0] } };
    if (p?.fingers3) for (let k = 0; k < 3; k++) { const a = ((p.radialDeg ?? 90) + k * 120) * Math.PI / 180; l['f' + (k + 1)] = { dof: 'prismatic', axis: [Math.cos(a), Math.sin(a), 0] }; }
    return l;
  },
  // The finger sockets carry a maker's jaws (a shell riding a finger moves with the chuck).
  sockets: (/** @type {any} */ p) => ({ tip: { link: 'body', at: [0, 0, p.fingerLen] }, fingerL: { link: 'fingerL', at: [0, 0, 0] }, fingerR: { link: 'fingerR', at: [0, 0, 0] },
    ...(p.fingers3 ? { f1: { link: 'f1', at: [0, 0, 0] }, f2: { link: 'f2', at: [0, 0, 0] }, f3: { link: 'f3', at: [0, 0, 0] } } : {}) }),
  // A real parallel gripper is a ROUND body with a slide plate under it and two short fingers -
  // that is the shape in the video of the cell, and it is what tells a gripper apart from the
  // plate it is bolted to at cell scale. Only the finger PAD is a collider: the body, the slide
  // and the vee flanks are `collide: false`, so the shape reads better and the physics is
  // exactly what it was.
  shapes: (/** @type {any} */ p) => {
    /** @type {any[]} */
    const s = [
      { link: 'body', kind: 'cyl', r: (p.span + 26) / 2, h: 26, at: [0, 0, -13], mat: 'dark', collide: false },
      { link: 'body', kind: 'cyl', r: (p.span + 26) / 4, h: 14, at: [0, 0, -33], mat: 'alu', collide: false },
      { link: 'body', kind: 'box', size: [p.fingerW * 2 + 8, p.span + 20, 6], at: [0, 0, 3], mat: 'steel', collide: false },
    ];
    // `m` is the side the finger's material lies on, so its inner face is the plane y = 0 that
    // the model closes onto `blockAt`.
    for (const [link, m] of /** @type {Array<[string, number]>} */ ([['fingerL', -1], ['fingerR', 1]])) {
      s.push({ link, kind: 'box', size: [p.fingerW * 2, p.fingerW, p.fingerLen], at: [0, m * p.fingerW / 2, p.fingerLen / 2], mat: 'holder' });
      if (p.jaw !== 'vee') continue;
      // The vee's two flanks, DRAWN only: the pad above is still the one collider, so the
      // physics is exactly what a flat jaw's was and a flank can never close onto a part.
      // Each flank runs out from an apex `back` behind the inner face at VEE_DEG to it, which
      // is tangent to a cylinder of radius jawR seated between the pads: (jawR - y0) cos = jawR.
      const th = VEE_DEG * Math.PI / 180, c = Math.cos(th), sn = Math.sin(th);
      const back = p.jawR * (1 / c - 1), L = p.jawR * 1.2, t = p.fingerW;
      for (const ex of [-1, 1]) {
        s.push({ link, kind: 'box', size: [L, t, p.fingerLen], mat: 'holder', collide: false,
                 at: [ex * (L / 2 * c + t / 2 * sn), m * (back - L / 2 * sn + t / 2 * c), p.fingerLen / 2],
                 rot: [0, 0, -ex * m * VEE_DEG] });
      }
    }
    return p.hidden ? s.map(x => ({ ...x, draw: false })) : s;
  },
  /** Where a part has to be for the fingers to catch it, in the gripper's frame. */
  zone: (/** @type {any} */ p) => ({ at: [0, 0, p.fingerLen * 0.6], size: [p.fingerW * 2, p.span, p.fingerLen * 0.8] }),
  init: (/** @type {any} */ p) => ({ x: p.span / 2, blockAt: 0, grip: false, uid: null, spool: false }),
  step(/** @type {any} */ s, /** @type {any} */ p, /** @type {any} */ io, /** @type {number} */ dt) {
    const ms = dt * 1000;
    if (p.valve === 'double') { if (io.close && !io.unclose) s.spool = true; else if (io.unclose && !io.close) s.spool = false; }
    const closing = p.valve === 'double' ? s.spool : !!io.close;
    const v = (p.span / 2) / (closing ? p.closeMs : p.openMs);          // mm per ms
    // blockAt is the half-width of the part in the grip zone, measured by the plant.
    const target = closing ? Math.max(0, s.blockAt) : p.span / 2;
    if (s.x > target) s.x = Math.max(target, s.x - v * ms);
    else if (s.x < target) s.x = Math.min(target, s.x + v * ms);
    s.grip = closing && s.blockAt > 0 && s.x <= s.blockAt + 0.01;
    io.open = s.x >= p.span / 2 - p.band;
    io.closed = p.confirm === 'part' ? s.grip : s.x <= p.band;
  },
  check: (/** @type {any} */ p) => {
    const e = p.span / 2 > p.band ? [] : ['span must be wider than twice the switch band'];
    // The notch is cut INTO the jaw, so its apex has to stay inside the finger's own thickness.
    if (p.jaw === 'vee' && p.jawR * (1 / Math.cos(VEE_DEG * Math.PI / 180) - 1) >= p.fingerW) {
      e.push('a vee jaw for jawR ' + p.jawR + ' needs a finger thicker than ' + p.fingerW + ' mm');
    }
    return e;
  },
};

/**
 * A pitch-change head: `n` cup slots on a bar whose spacing is set by a camshaft. The pallet
 * is picked at one pitch and the jig is loaded at another, and on a real head that is a cam
 * groove per slot, so one motor changes every pitch at once. That is ONE dof - the cam angle -
 * with a different `scale` per slot link: slot i sits at (i - c) * pitchMax and slides by
 * (i - c) * (pitchMin - pitchMax) / camDeg per degree of cam, so at cam 0 the pitch is pitchMax
 * and at camDeg it is pitchMin. The cam link turns by the same dof, so the picture shows the
 * shaft turning as the cups close up. Cups mount on the slot sockets s0..s(n-1).
 * Linear in the cam angle: a real groove is shaped, but the pitch at both ends is what matters.
 */
const pitchBar = {
  label: 'Pitch-change head (camshaft)', group: 'motion',
  params: [
    { k: 'n', type: 'num', def: 5, min: 2 },
    { k: 'pitchMin', type: 'num', def: 60, min: 1, unit: 'mm' },
    { k: 'pitchMax', type: 'num', def: 100, min: 1, unit: 'mm' },
    { k: 'camDeg', type: 'num', def: 90, min: 1, unit: 'deg' },
    { k: 'home', type: 'num', def: 0 },
    { k: 'vmax', type: 'num', def: 180, min: 0.1, unit: 'deg/s' },
    { k: 'acc', type: 'num', def: 720, min: 0.1, unit: 'deg/s²' },
    { k: 'band', type: 'num', def: 0.2, min: 0 },
    { k: 'bar', type: 'vec3', def: [0, 60, 12], unit: 'mm' },
  ],
  io: () => ({
    target: { dir: 'out', type: 'LREAL' }, exec: { dir: 'out', type: 'BOOL' },
    done: { dir: 'in', type: 'BOOL', hold: false }, busy: { dir: 'in', type: 'BOOL', hold: false },
    actPos: { dir: 'in', type: 'LREAL' }, inPos: { dir: 'in', type: 'BOOL', hold: false },
  }),
  links: (/** @type {any} */ p) => {
    const c = (p.n - 1) / 2;
    /** @type {Record<string, any>} */
    const L = { body: {}, cam: { at: [0, 0, p.bar[2] + 14], dof: 'revolute', axis: [1, 0, 0] } };
    for (let i = 0; i < p.n; i++) L['s' + i] = { at: [(i - c) * p.pitchMax, 0, 0], dof: 'prismatic', axis: [1, 0, 0], scale: (i - c) * (p.pitchMin - p.pitchMax) / p.camDeg };
    return L;
  },
  sockets: (/** @type {any} */ p) => {
    /** @type {Record<string, any>} */
    const S = { flange: { link: 'body', at: [0, 0, p.bar[2]] } };
    for (let i = 0; i < p.n; i++) S['s' + i] = { link: 's' + i, at: [0, 0, 0] };
    return S;
  },
  shapes: (/** @type {any} */ p) => {
    // Nothing here collides: the bar sweeps over parts the cups are about to take, and a solid
    // bar closing onto a part held by a kinematic nest is the eject case in CLAUDE.md.
    const L = (p.n - 1) * p.pitchMax + 40, [, W, T] = p.bar;
    /** @type {any[]} */
    const s = [
      { link: 'body', kind: 'box', size: [L, W, T], at: [0, 0, T / 2], mat: 'steel', collide: false },
      { link: 'cam', kind: 'cyl', r: 8, h: L - 20, at: [0, 0, 0], rot: [0, 90, 0], mat: 'dark', collide: false },
      { link: 'cam', kind: 'box', size: [12, 30, 6], at: [0, 8, 0], mat: 'motion', collide: false },   // a lobe, so the turn is visible
    ];
    for (let i = 0; i < p.n; i++) s.push({ link: 's' + i, kind: 'box', size: [16, W + 8, T + 4], at: [0, 0, T / 2], mat: 'motion', collide: false });
    return s;
  },
  init: (/** @type {any} */ p) => ({ x: p.home, v: 0, tgt: p.home, exec: false, moving: false, arrived: true }),
  step(/** @type {any} */ s, /** @type {any} */ p, /** @type {any} */ io, /** @type {number} */ dt) {
    const e = !!io.exec;
    if (e && !s.exec) { s.tgt = clamp(Number(io.target) || 0, 0, p.camDeg); s.moving = true; s.arrived = false; }
    s.exec = e;
    if (s.moving) {
      const r = trapStep(s.x, s.tgt, s.v, p.vmax, p.acc, dt);
      s.x = r.pos; s.v = r.vel;
      if (!r.moving) { s.moving = false; s.arrived = true; }
    }
    io.done = e && s.arrived;
    io.busy = s.moving;
    io.actPos = Math.round(s.x * 1000) / 1000;
    io.inPos = !s.moving && Math.abs(s.x - s.tgt) <= p.band;
  },
  check: (/** @type {any} */ p) => (p.pitchMax > p.pitchMin ? [] : ['pitchMax must exceed pitchMin']),
};

/**
 * A 3D model with no kinematics: the maker's own shell, mounted like any other component. Used
 * for the parts of a machine that only have to LOOK right - a robot's base casting, a guard, a
 * frame imported from CAD. It is drawn and never collided: a trimesh does not collide with a
 * trimesh in Rapier, and a part sensor would have to ray-trace thousands of triangles to answer
 * "is something there". Where a shell needs to stop a part, put a `plate` inside it.
 */
const shell = {
  label: 'Shell (3D model)', group: 'structure',
  params: [
    { k: 'asset', type: 'str', def: '' },
    // Model units to mm: a URDF mesh is in metres, so 1000.
    { k: 'scale', type: 'num', def: 1000, min: 1e-6 },
    { k: 'color', type: 'str', def: '' },
    // What to draw when the asset is not there: [cx, cy, cz, sx, sy, sz] boxes in the shell's frame.
    // A maker's CAD is often not ours to ship (it is gitignored), and a machine that silently
    // draws as nothing is the one failure a viewer must never show quietly.
    { k: 'boxes', type: 'json', def: [] },
  ],
  links: () => ({ body: {} }),
  sockets: () => ({ top: { link: 'body', at: [0, 0, 0] } }),
  shapes: (/** @type {any} */ p) => (p.asset
    ? [{ link: 'body', kind: 'mesh', asset: p.asset, scale: p.scale, at: [0, 0, 0], mat: 'alu', ...(p.color ? { color: p.color } : {}), collide: false },
       ...(Array.isArray(p.boxes) ? p.boxes : []).map((/** @type {number[]} */ b) => ({ link: 'body', kind: 'box', size: b.slice(3, 6), at: b.slice(0, 3),
         mat: 'alu', ...(p.color ? { color: p.color } : {}), collide: false, draw: false }))]
    : []),
  check: (/** @type {any} */ p) => {
    const e = p.asset ? [] : ['asset must name a model under assets/'];
    if (!Array.isArray(p.boxes) || !p.boxes.every((/** @type {any} */ b) => Array.isArray(b) && b.length === 6 && b.every(n => typeof n === 'number' && Number.isFinite(n))
        && b[3] > 0 && b[4] > 0 && b[5] > 0)) e.push('boxes must be a list of [cx, cy, cz, sx, sy, sz] with positive sizes');
    return e;
  },
};

const nest = {
  // snap: a nest locates the part it catches, square on the pocket floor, as a real one does.
  label: 'Nest / fixture', group: 'structure', hold: 'nest', snap: true,
  // A pocket that keeps a part still. With `clamp` bound it holds only while the clamp is on.
  params: [
    { k: 'size', type: 'vec3', def: [80, 60, 25], unit: 'mm' },
    { k: 'wall', type: 'num', def: 8, min: 1, unit: 'mm' },
    // hidden: the maker's CAD draws the jig; the floor still collides and the pocket still holds.
    { k: 'hidden', type: 'bool', def: false },
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
    const s = [{ link: 'body', kind: 'box', size: [sx + 2 * w, sy + 2 * w, 6], at: [0, 0, -3], mat: 'holder' }];
    for (const ix of [-1, 1]) s.push({ link: 'body', kind: 'box', size: [w, sy + 2 * w, sz], at: [ix * (sx + w) / 2, 0, sz / 2], mat: 'holder', collide: false });
    for (const iy of [-1, 1]) s.push({ link: 'body', kind: 'box', size: [sx, w, sz], at: [0, iy * (sy + w) / 2, sz / 2], mat: 'holder', collide: false });
    return p.hidden ? s.map(x => ({ ...x, draw: false })) : s;
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
  // A retro-reflective sensor works on the light a part sends BACK, and a matt black part sends
  // almost none: Festo's MPS sorting station tells black from red with exactly this - a fork
  // light barrier sees every workpiece, the retro-reflective one sees only the ones that are not
  // black, and the inductive one only the metal. `seesDark: false` is that sensor.
  { k: 'seesDark', type: 'bool', def: true },
];

/** Relative luminance of a '#rrggbb' colour, 0..1. Below DARK a retro-reflective sensor is blind. @param {unknown} hex */
export function luminance(hex) {
  const m = /^#?([0-9a-f]{6})$/i.exec(String(hex || ''));
  if (!m) return 1;
  const n = parseInt(m[1], 16), r = (n >> 16 & 255) / 255, g = (n >> 8 & 255) / 255, b = (n & 255) / 255;
  return 0.2126 * r + 0.7152 * g + 0.0722 * b;
}
export const DARK = 0.2;

const photoEye = {
  label: 'Photoelectric sensor', group: 'sensors', sense: 'ray',
  // The beam leaves the lens (frame origin) along +X. Diffuse or through-beam are the same ray
  // here: `range` is the reach, or the distance to the receiver.
  params: [{ k: 'range', type: 'num', def: 300, min: 1, unit: 'mm' }, ...SENSE],
  io: () => ({ out: { dir: 'in', type: 'BOOL', dev: 'PH', words: 'EXIST' } }),
  links: () => ({ body: {} }),
  sockets: () => ({}),
  shapes: (/** @type {any} */ p) => [
    { link: 'body', kind: 'box', size: [18, 30, 22], at: [-9, 0, 0], mat: 'sensor', collide: false },
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
    { link: 'body', kind: 'cyl', r: 6, h: 40, at: [-20, 0, 0], rot: [0, 90, 0], mat: 'sensor', collide: false },
    { link: 'body', kind: 'cyl', r: 6.5, h: 3, at: [-1.5, 0, 0], rot: [0, 90, 0], mat: 'reed', glow: 'out', collide: false },
  ],
  init: () => ({ hit: false, on: false, offT: 0 }),
  step: senseStep,
};

// ---------------------------------------------------------------------------- electric actuators

/**
 * An IAI ELECYLINDER (or any electric actuator run from its controller's point I/O): the PLC
 * holds FWD START or BWD START (levels) and reads FORWARD / BACKWARD COMPLETE, which are ON at the
 * position whatever moved it there. A 3-point model has a MIDDLE START too. Motion is the one
 * trapezoid (trapStep). `kind: 'rotary'` turns a table about local Z (EC-RT..): stroke and mid in
 * degrees. Both or neither of FWD/BWD holds the last target, as the controller does.
 */
const elecylinder = {
  label: 'Electric cylinder (point I/O)', group: 'motion',
  params: [
    { k: 'kind', type: 'enum', of: ['linear', 'rotary'], def: 'linear' },
    { k: 'stroke', type: 'num', def: 100, min: 1, unit: 'mm or deg' },
    // -1: a 2-point model with no MIDDLE.
    { k: 'mid', type: 'num', def: -1, min: -1, unit: 'mm or deg' },
    { k: 'vmax', type: 'num', def: 200, min: 0.1, unit: 'mm/s or deg/s' },
    { k: 'acc', type: 'num', def: 2000, min: 0.1, unit: 'mm/s² or deg/s²' },
    { k: 'band', type: 'num', def: 0.5, min: 0, unit: 'mm or deg' },
    { k: 'body', type: 'vec3', def: [300, 60, 60], unit: 'mm' },
    { k: 'hidden', type: 'bool', def: false },
    // Where it is at power-up: the add-on's loader beam rests FORWARD, and its first cycle needs it there.
    { k: 'start', type: 'enum', of: ['bwd', 'fwd'], def: 'bwd' },
  ],
  io: (/** @type {any} */ p) => ({
    fwd: { dir: 'out', type: 'BOOL', dev: 'CR', words: 'FWD' }, bwd: { dir: 'out', type: 'BOOL', dev: 'CR', words: 'BWD' },
    ...(p?.mid >= 0 ? { mid: { dir: 'out', type: 'BOOL', dev: 'CR', words: 'MID' }, midEnd: { dir: 'in', type: 'BOOL', dev: 'LS', words: 'MID' } } : {}),
    fwdEnd: { dir: 'in', type: 'BOOL', dev: 'LS', words: 'FWD' }, bwdEnd: { dir: 'in', type: 'BOOL', dev: 'LS', words: 'BWD' },
    alarm: { dir: 'in', type: 'BOOL', dev: 'AL', words: 'ALARM' },
  }),
  links: (/** @type {any} */ p) => (p.kind === 'rotary'
    ? { body: {}, slider: { at: [0, 0, p.body[2]], dof: 'revolute', axis: [0, 0, 1] } }
    : { body: {}, slider: { at: [-p.body[0] / 2 + 30, 0, p.body[2]], dof: 'prismatic', axis: [1, 0, 0] } }),
  sockets: () => ({ slider: { link: 'slider', at: [0, 0, 0] } }),
  shapes: (/** @type {any} */ p) => {
    const [bx, by, bz] = p.body;
    const s = p.kind === 'rotary'
      ? [{ link: 'body', kind: 'box', size: [bx, by, bz], at: [0, 0, bz / 2], mat: 'motion' },
         { link: 'slider', kind: 'cyl', r: Math.min(bx, by) / 2 - 4, h: 12, at: [0, 0, 6], mat: 'alu' }]
      : [{ link: 'body', kind: 'box', size: [bx, by, bz], at: [0, 0, bz / 2], mat: 'motion' },
         { link: 'slider', kind: 'box', size: [60, by - 8, 14], at: [0, 0, 7], mat: 'alu' }];
    return p.hidden ? s.map(x => ({ ...x, draw: false })) : s;
  },
  init: (/** @type {any} */ p) => ({ x: p.start === 'fwd' ? p.stroke : 0, v: 0, cmd: p.start === 'fwd' ? p.stroke : 0 }),
  step(/** @type {any} */ s, /** @type {any} */ p, /** @type {any} */ io, /** @type {number} */ dt) {
    const want = p.mid >= 0 && io.mid ? p.mid : io.fwd && !io.bwd ? p.stroke : io.bwd && !io.fwd ? 0 : null;
    if (want !== null) s.cmd = want;
    const r = trapStep(s.x, s.cmd, s.v, p.vmax, p.acc, dt);
    s.x = r.pos; s.v = r.vel;
    io.fwdEnd = Math.abs(s.x - p.stroke) <= p.band;
    io.bwdEnd = Math.abs(s.x) <= p.band;
    // MIDDLE is a position the axis STOPS at: passing through it on the way to an end is not it.
    if (p.mid >= 0) io.midEnd = Math.abs(s.x - p.mid) <= p.band && Math.abs(s.cmd - p.mid) <= p.band && !r.moving;
    // *ALM, as IAI wires it: ON while the controller is healthy, so a cut cable reads as an alarm.
    io.alarm = true;
  },
  check: (/** @type {any} */ p) => (p.mid >= p.stroke ? ['mid must be inside the stroke'] : []),
};

/**
 * A motor-driven turntable that ORIENTS the part on it: the part is put down at any angle, the
 * motor turns (FWD / REV levels) until a sensor sees the part's feature - a terminal, a flat - and
 * the PLC stops it. The part rides the table (a nest-type holder that does NOT square it up, so
 * its angle is kept), and the two sensors are analytic, from the table angle plus the angle the
 * part was caught at: PRE-END comes on `preDeg` before the feature reaches the sensor, IN POSITION
 * within `band` of it. Turning FWD brings the feature on from the PRE side.
 * `lock` is a locating pin raised into the part's path: the feature cannot turn past it, so the
 * motor stalls with the part IN POSITION (the add-on raises its pin at PRE-END and keeps the motor
 * on until the sensor holds - measured: with no pin the part went round and round).
 */
const turntable = {
  label: 'Orienting turntable (motor)', group: 'motion', hold: 'nest', holdLink: 'table', snap: 'yaw',
  params: [
    { k: 'size', type: 'vec3', def: [110, 110, 40], unit: 'mm' },
    { k: 'height', type: 'num', def: 60, min: 1, unit: 'mm' },
    { k: 'speed', type: 'num', def: 90, min: 0.1, unit: 'deg/s' },
    { k: 'accMs', type: 'num', def: 80, min: 0, unit: 'ms' },
    { k: 'feature', type: 'num', def: 0, unit: 'deg' },
    { k: 'preDeg', type: 'num', def: 30, min: 0, unit: 'deg' },
    { k: 'band', type: 'num', def: 2, min: 0.1, unit: 'deg' },
    { k: 'hidden', type: 'bool', def: false },
    // cw: FWD turns clockwise seen from above (the motor's wiring decides, not the model).
    { k: 'cw', type: 'bool', def: false },
  ],
  io: () => ({
    fwd: { dir: 'out', type: 'BOOL', dev: 'MTR', words: 'FWD' }, rev: { dir: 'out', type: 'BOOL', dev: 'MTR', words: 'REV' },
    lock: { dir: 'out', type: 'BOOL', dev: 'PIN', words: 'LOCK' },
    pre: { dir: 'in', type: 'BOOL', dev: 'PH', words: 'PRE' }, inPos: { dir: 'in', type: 'BOOL', dev: 'PH', words: 'INPOS' },
    present: { dir: 'in', type: 'BOOL', dev: 'PH', words: 'EXIST' },
  }),
  links: (/** @type {any} */ p) => ({ body: {}, table: { at: [0, 0, p.height], dof: 'revolute', axis: [0, 0, p.cw ? -1 : 1] } }),
  sockets: () => ({ top: { link: 'table', at: [0, 0, 0] } }),
  shapes: (/** @type {any} */ p) => {
    const d = Math.max(p.size[0], p.size[1]);
    const s = [
      { link: 'body', kind: 'cyl', r: d / 2 * 0.7, h: p.height - 6, at: [0, 0, (p.height - 6) / 2], mat: 'motion', collide: false },
      { link: 'table', kind: 'cyl', r: d / 2, h: 6, at: [0, 0, -3], mat: 'holder' },
      // A mark on the table, so a turn can be seen.
      { link: 'table', kind: 'box', size: [d / 2 - 8, 6, 2], at: [d / 4, 0, 1], mat: 'tool', collide: false },
    ];
    return p.hidden ? s.map(x => ({ ...x, draw: false })) : s;
  },
  init: () => ({ x: 0, v: 0, uid: null, relYaw: null }),
  step(/** @type {any} */ s, /** @type {any} */ p, /** @type {any} */ io, /** @type {number} */ dt) {
    const want = io.fwd && !io.rev ? p.speed : io.rev && !io.fwd ? -p.speed : 0;
    const dv = p.accMs > 0 ? p.speed / (p.accMs / 1000) * dt : Infinity;
    s.v = Math.abs(want - s.v) <= dv ? want : s.v + Math.sign(want - s.v) * dv;
    const held = s.uid != null && s.relYaw != null;
    // Progress of the feature towards the sensor, -180..180, 0 = at it, rising as FWD turns (the
    // table's own angle runs the other way round when it turns clockwise).
    const sg = p.cw ? -1 : 1;
    const fOf = (/** @type {number} */ x) => sg * (((((sg * x + s.relYaw - p.feature) % 360) + 540) % 360) - 180);
    let nx = s.x + s.v * dt;
    // The pin stops the feature at the sensor: turning FWD it cannot cross from - to +.
    if (held && io.lock && s.v > 0) { const f0 = fOf(s.x), f1 = f0 + (nx - s.x); if (f0 <= 0 && f1 > 0) { nx = s.x - f0; s.v = 0; } }
    s.x = nx;
    if (Math.abs(s.x) > 3600) s.x %= 360;                   // an unbounded angle, kept small
    const f = held ? fOf(s.x) : NaN;
    io.inPos = held && Math.abs(f) <= p.band;
    io.pre = held && f < -p.band && f >= -p.band - p.preDeg;
    io.present = s.uid != null;
  },
};

/**
 * An assembly or process station that changes what a part IS: the parts in its zone are
 * replaced by one part of `template` (or `templateAlt` while `alt` is on) at the pose of the
 * lowest of them. Two parts stacked in a nest become the assembly the next gripper carries as
 * one; a caulked unit becomes a finished OK or NG part. `mode: 'auto'` joins as soon as `n`
 * parts are in the zone and have been still for `settleMs`; `tag` joins on each rising edge of
 * `join`, whatever is there. A part a gripper or cup is carrying is never joined: it is passing.
 */
const joiner = {
  label: 'Joiner (assemble / process)', group: 'material flow', flow: 'joiner',
  params: [
    { k: 'size', type: 'vec3', def: [120, 120, 80], unit: 'mm' },
    { k: 'template', type: 'ref', of: 'part', template: true, def: '' },
    { k: 'templateAlt', type: 'ref', of: 'part', template: true, def: '' },
    { k: 'mode', type: 'enum', of: ['auto', 'tag'], def: 'auto' },
    { k: 'n', type: 'num', def: 2, min: 1 },
    { k: 'settleMs', type: 'num', def: 150, min: 0, unit: 'ms' },
  ],
  io: (/** @type {any} */ p) => (p?.mode === 'tag'
    ? { join: { dir: 'out', type: 'BOOL' }, alt: { dir: 'out', type: 'BOOL' }, count: { dir: 'in', type: 'UDINT' } }
    : { count: { dir: 'in', type: 'UDINT' } }),
  links: () => ({ body: {} }),
  sockets: () => ({}),
  shapes: (/** @type {any} */ p) => [{ link: 'body', kind: 'box', size: p.size, at: [0, 0, p.size[2] / 2], mat: 'reed', collide: false, ghost: true }],
  init: () => ({ n: 0, req: 0, last: false, alt: false, calm: 0 }),
  step(/** @type {any} */ s, /** @type {any} */ p, /** @type {any} */ io) {
    if (p.mode === 'tag') { const e = !!io.join; if (e && !s.last) s.req++; s.last = e; s.alt = !!io.alt; }
    io.count = s.n;
  },
  check: (/** @type {any} */ p) => (p.template ? [] : ['params.template must name the part the station makes']),
};

export const TYPES = /** @type {Record<string, any>} */ ({ frame, plate, cylinder, servoLinear, pushbutton, selector, speedDial, lamp, workpiece, conveyor, emitter, remover, track, photoEye, proximity, vacuumCup, gripper, nest, indexTable, pallet, palletLift, joint, pitchBar, shell, elecylinder, turntable, joiner });
