// @ts-check
// Scene model: a FLAT list of component instances, each naming its parent. The scene JSON is
// the only place tag names, dimensions and stroke times live.
//
// worldPoses() is THE pose function. Node and the browser both call it with the same scene and
// the same DOF values, so the picture cannot disagree with the plant.
import { TYPES, withDefaults } from './components.js';
import { pose, compose, invert, qaxis, eulerOf, IDENTITY } from './math.js';

export const FORMAT = 'mio-scene/1';
export const NAME_RE = /^[a-z0-9_-]+$/;
export const TAG_RE = /^[A-Za-z_][A-Za-z0-9_]*$/;
const ID_RE = /^[A-Za-z_][A-Za-z0-9_-]*$/;

/** @typedef {import('./math.js').Pose} Pose */
/** @typedef {{id: string, type: string, parent?: string, socket?: string, at?: number[], rot?: number[],
 *             params?: Record<string, any>, io?: Record<string, string>, label?: string, station?: string}} Comp */
/** @typedef {{format: string, name: string, components: Comp[], sim?: any, io?: any, stations?: any[], cycle?: any}} Scene */

/** Instance params with the type's defaults filled in. @param {Comp} c */
export function params(c) {
  return withDefaults(TYPES[c.type], c.params || {});
}

/** @type {WeakMap<object, any>} */
const cache = new WeakMap();

/**
 * Everything derived from the scene once: parent-first order and each component's links,
 * sockets, shapes and io schema. Cached per scene OBJECT, so an edited scene must be a new
 * object (the editor's snapshots are). Components with a missing parent or in a cycle are
 * left out of `order`; validate() names them.
 * @param {Scene} scene
 */
export function compile(scene) {
  let c = cache.get(scene);
  if (c) return c;
  const byId = new Map(scene.components.map(x => [x.id, x]));
  const mark = new Map();                        // 1 visiting, 2 ok, 3 broken
  /** @type {Comp[]} */
  const order = [];
  /** @param {Comp} x @returns {boolean} */
  const visit = x => {
    const m = mark.get(x.id);
    if (m) return m === 2;
    mark.set(x.id, 1);
    const par = x.parent == null ? null : byId.get(x.parent);
    const ok = !!TYPES[x.type] && (x.parent == null || (!!par && visit(par)));
    mark.set(x.id, ok ? 2 : 3);
    if (ok) order.push(x);
    return ok;
  };
  scene.components.forEach(visit);

  const defs = new Map();
  for (const x of order) {
    const t = TYPES[x.type], p = params(x);
    const links = Object.entries(t.links(p)).map(([name, l]) => ({ name, ...l, local: pose(l.at, l.rot) }));
    const init = t.init ? t.init(p) : {};
    defs.set(x.id, {
      c: x, t, p, links, root: links[0].name,
      // The mount offset never changes while a scene object lives, and worldPoses() was rebuilding
      // it from three Euler angles for every moving component on every 2 ms step.
      mount: pose(x.at, x.rot),
      sockets: t.sockets ? t.sockets(p) : {},
      shapes: t.shapes ? t.shapes(p) : [],
      io: t.io ? t.io(p) : {},
      x0: typeof init.x === 'number' ? init.x : 0,
    });
  }
  // Which components can never move? A scene is mostly furniture - frames, plates, pallet pockets,
  // the parts standing in them - and recomputing their poses every 2 ms costs more than the
  // machine does. Measured on `palletizing` (265 components): 2276 -> 735 us a step.
  /** @type {Map<string, boolean>} */
  const stat = new Map();
  /** Does this link of this component sit still for ever? @param {string} id @param {string} link @returns {boolean} */
  const linkStill = (id, link) => {
    const d = defs.get(id);
    for (let l = d.links.find((/** @type {any} */ x) => x.name === link); l; l = l.parent ? d.links.find((/** @type {any} */ x) => x.name === l.parent) : null) {
      if (l.dof) return false;
    }
    return baseStill(id);
  };
  /** @param {string} id @returns {boolean} */
  const baseStill = id => {
    if (stat.has(id)) return /** @type {boolean} */ (stat.get(id));
    const d = defs.get(id), x = d.c;
    stat.set(id, false);                                  // a cycle cannot be still; validate() names it
    const still = x.parent == null || !defs.has(x.parent) ? x.parent == null
      : linkStill(x.parent, x.socket != null ? defs.get(x.parent).sockets[x.socket].link : defs.get(x.parent).root);
    stat.set(id, still);
    return still;
  };
  /** Components whose every link is still: their poses are computed once and reused. */
  const still = new Set();
  for (const x of order) {
    const d = defs.get(x.id);
    if (baseStill(x.id) && d.links.every((/** @type {any} */ l) => !l.dof)) still.add(x.id);
  }
  c = { order, defs, byId, still, poses: new Map() };
  cache.set(scene, c);
  return c;
}

/**
 * World pose of every link of every component, parent first.
 * frame  = parent link (via socket) x mount offset (at, rot)
 * link   = its parent link (or the frame) x (at, rot) x DOF motion
 * @param {Scene} scene @param {Record<string, number>} [dof] one number per moving component
 * @returns {Record<string, Record<string, Pose>>}
 */
export function worldPoses(scene, dof) {
  const { order, defs, still, poses } = compile(scene);
  /** @type {Record<string, Record<string, Pose>>} */
  const out = {};
  for (const c of order) {
    // Furniture: the same poses every time, so they are computed on the first call only. The
    // result is shared, which is safe because nothing writes to a pose it was given.
    const kept = poses.get(c.id);
    if (kept) { out[c.id] = kept; continue; }
    const d = defs.get(c.id);
    const frame = compose(baseOf(c, defs, out), d.mount);
    const x = dof && typeof dof[c.id] === 'number' ? dof[c.id] : d.x0;
    /** @type {Record<string, Pose>} */
    const L = {};
    for (const l of d.links) {
      let P = compose(l.parent ? L[l.parent] : frame, l.local);
      if (l.dof === 'prismatic') {
        // `wrap`: a repeating pattern (belt stripes). The DOF is the unbounded travel, so the
        // viewer interpolates it smoothly; only the pose wraps.
        const k = (l.wrap ? ((x % l.wrap) + l.wrap) % l.wrap : x) * (l.scale ?? 1);
        P = compose(P, { p: [l.axis[0] * k, l.axis[1] * k, l.axis[2] * k], q: [0, 0, 0, 1] });
      } else if (l.dof === 'revolute') {
        P = compose(P, { p: [0, 0, 0], q: qaxis(l.axis, x * (l.scale ?? 1)) });
      }
      L[l.name] = P;
    }
    out[c.id] = L;
    if (still.has(c.id)) poses.set(c.id, L);
  }
  return out;
}

/**
 * Components that are loose parts rather than machine:
 *   'free'     a dynamic workpiece placed in the scene: a Rapier-dynamic body from the start;
 *   'template' a workpiece an emitter copies: never simulated, never drawn, only its shapes
 *              and physics params are used.
 * worldPoses() gives a free part its starting pose only; after that the plant streams it.
 * @param {Scene} scene @returns {Map<string, 'free'|'template'>}
 */
export function partRoles(scene) {
  /** @type {Map<string, 'free'|'template'>} */
  const m = new Map();
  for (const c of scene.components || []) if (TYPES[c.type]?.part && params(c).dynamic) m.set(c.id, 'free');
  for (const c of scene.components || []) {
    for (const d of TYPES[c.type]?.params || []) {
      const v = c.params?.[d.k];
      if (d.type === 'ref' && d.template && typeof v === 'string' && v) m.set(v, 'template');
    }
  }
  return m;
}

/**
 * The world pose a component is mounted on: its parent's socket, the parent's root link, or the
 * world. @param {Comp} c @param {Map<string, any>} defs @param {Record<string, Record<string, Pose>>} out
 * @returns {Pose}
 */
function baseOf(c, defs, out) {
  if (c.parent == null) return IDENTITY;
  const pd = defs.get(c.parent), pl = out[c.parent];
  const s = c.socket != null ? pd.sockets[c.socket] : null;
  return s ? compose(pl[s.link], pose(s.at, s.rot)) : pl[pd.root];
}

/**
 * A component's mount for the editor: `base` is what it is mounted on and `frame` = base x
 * (at, rot). Null when the component is not placed (missing parent, cycle).
 * @param {Scene} scene @param {Record<string, number>} dof @param {string} id
 * @returns {{base: Pose, frame: Pose} | null}
 */
export function mountPoses(scene, dof, id) {
  const { defs } = compile(scene);
  const c = defs.get(id)?.c;
  if (!c) return null;
  const base = baseOf(c, defs, worldPoses(scene, dof));
  return { base, frame: compose(base, pose(c.at, c.rot)) };
}

/**
 * The `at` and `rot` that put a component's frame at world pose `w` on `base`: how a world-space
 * drag becomes a mount offset in the parent's frame. Rounded to 0.001 mm / 0.001°.
 * @param {Pose} base @param {Pose} w @returns {{at: number[], rot: number[]}}
 */
export function mountFrom(base, w) {
  const l = compose(invert(base), w);
  const r = (/** @type {number} */ v) => { const x = Math.round(v * 1000) / 1000; return x === 0 ? 0 : x === -180 ? 180 : x; };
  return { at: l.p.map(r), rot: eulerOf(l.q).map(r) };
}

/**
 * Every tag the scene binds, with direction and type from the component type's schema.
 * `out` = PLC -> plant (actuator command), `in` = plant -> PLC (sensor). Station step tags and
 * the cycle's auto/count tags are PLC outputs the plant only records.
 * @param {Scene} scene
 * @returns {Array<{tag: string, dir: 'in'|'out', type: string, comp?: string, key: string, dev?: string, words?: string, station?: string, unknown?: boolean}>}
 */
export function bindings(scene) {
  const out = [];
  for (const c of scene.components || []) {
    const t = TYPES[c.type];
    if (!t) continue;
    const schema = t.io ? t.io(params(c)) : {};
    for (const [key, tag] of Object.entries(c.io || {})) {
      const s = schema[key];
      out.push({ tag, key, comp: c.id, dir: s?.dir, type: s?.type, dev: s?.dev, words: s?.words, station: c.station, unknown: !s });
    }
  }
  for (const st of scene.stations || []) {
    if (st.stepTag) out.push({ tag: st.stepTag, key: 'step', dir: 'out', type: 'INT', station: st.id });
  }
  const cy = scene.cycle || {};
  if (cy.autoTag) out.push({ tag: cy.autoTag, key: 'auto', dir: 'out', type: 'BOOL' });
  if (cy.countTag) out.push({ tag: cy.countTag, key: 'count', dir: 'out', type: 'UDINT' });
  return /** @type {any} */ (out);
}

/** tag -> {dir, type}, one entry per tag. Run validate() first: conflicts are not reported here. @param {Scene} scene */
export function tags(scene) {
  /** @type {Map<string, {dir: 'in'|'out', type: string}>} */
  const m = new Map();
  for (const b of bindings(scene)) if (!b.unknown && !m.has(b.tag)) m.set(b.tag, { dir: b.dir, type: b.type });
  return m;
}

/** @param {any} d @param {any} v @returns {string|null} */
function paramError(d, v) {
  switch (d.type) {
    case 'num': return typeof v === 'number' && Number.isFinite(v) && !(d.min != null && v < d.min) && !(d.max != null && v > d.max)
      ? null : 'must be a number' + (d.min != null ? ' >= ' + d.min : '') + (d.max != null ? ' <= ' + d.max : '');
    case 'enum': return d.of.includes(v) ? null : 'must be one of ' + d.of.join(', ');
    case 'vec3': return Array.isArray(v) && v.length === 3 && v.every(n => typeof n === 'number' && Number.isFinite(n)) ? null : 'must be [x, y, z]';
    case 'str': return typeof v === 'string' ? null : 'must be text';
    case 'ref': return typeof v === 'string' ? null : 'must name a ' + d.of;
    case 'bool': return typeof v === 'boolean' ? null : 'must be true or false';
    case 'switches': return Array.isArray(v) && v.every(w => w && typeof w.pos === 'number') ? null : 'must be a list of { id, pos }';
    default: return 'unknown descriptor type ' + d.type;
  }
}

const vec = (/** @type {any} */ v) => v == null || (Array.isArray(v) && v.length === 3 && v.every(n => typeof n === 'number' && Number.isFinite(n)));

/**
 * Every reason this scene cannot run. The same function guards the server's load and the
 * editor's save.
 * @param {any} scene @returns {string[]}
 */
export function validate(scene) {
  const e = [];
  if (!scene || typeof scene !== 'object') return ['scene is not an object'];
  if (scene.format !== FORMAT) e.push('format must be "' + FORMAT + '"');
  if (!NAME_RE.test(scene.name || '')) e.push('name must match ' + NAME_RE);
  const dt = scene.sim?.dtMs;
  if (dt != null && !(dt >= 1 && dt <= 10)) e.push('sim.dtMs must be 1..10');
  const hand = scene.sim?.handMmS;
  if (hand != null && !(hand >= 50 && hand <= 20000)) e.push('sim.handMmS must be 50..20000 mm/s: how fast the hand may drag a part');
  if (scene.io?.minPulseMs != null && !(scene.io.minPulseMs >= 0)) e.push('io.minPulseMs must be >= 0');
  if (scene.io?.mode != null && !['sim', 'twin'].includes(scene.io.mode)) e.push('io.mode must be sim or twin');
  if (!Array.isArray(scene.components)) return [...e, 'components must be a list'];

  const ids = new Map();
  for (const c of scene.components) {
    const who = 'component ' + JSON.stringify(c?.id);
    if (!c || typeof c.id !== 'string' || !ID_RE.test(c.id)) { e.push(who + ': id must be a word'); continue; }
    if (ids.has(c.id)) e.push('duplicate id ' + c.id);
    ids.set(c.id, c);
    const t = TYPES[c.type];
    if (!t) { e.push(who + ': unknown type ' + JSON.stringify(c.type)); continue; }
    if (!vec(c.at)) e.push(who + ': at must be [x, y, z]');
    if (!vec(c.rot)) e.push(who + ': rot must be [rx, ry, rz] in degrees');
    let bad = false;
    for (const [k, v] of Object.entries(c.params || {})) {
      const d = t.params.find((/** @type {any} */ q) => q.k === k);
      const m = d ? paramError(d, v) : 'unknown parameter';
      if (m) { e.push(who + ': params.' + k + ' ' + m); bad = true; }
    }
    if (bad) continue;
    const p = params(c);
    for (const m of t.check ? t.check(p) : []) e.push(who + ': ' + m);
    const schema = t.io ? t.io(p) : {};
    for (const [k, tag] of Object.entries(c.io || {})) {
      if (!schema[k]) e.push(who + ': io.' + k + ' is not a ' + c.type + ' signal (has: ' + Object.keys(schema).join(', ') + ')');
      if (typeof tag !== 'string' || !TAG_RE.test(tag)) e.push(who + ': io.' + k + ' tag ' + JSON.stringify(tag) + ' is not a PLC identifier');
    }
  }
  for (const c of scene.components) {
    if (!c || !ids.has(c.id) || c.parent == null) continue;
    const par = ids.get(c.parent);
    if (!par) { e.push('component ' + c.id + ': parent ' + JSON.stringify(c.parent) + ' does not exist'); continue; }
    if (c.socket != null && TYPES[par.type]) {
      const socks = TYPES[par.type].sockets ? TYPES[par.type].sockets(params(par)) : {};
      if (!socks[c.socket]) e.push('component ' + c.id + ': ' + par.type + ' ' + par.id + ' has no socket "' + c.socket + '" (has: ' + Object.keys(socks).join(', ') + ')');
    }
    // Only the members of a cycle report it; their descendants are merely unreachable.
    const seen = new Set();
    for (let x = par; x && !seen.has(x.id); x = x.parent == null ? undefined : ids.get(x.parent)) {
      if (x.id === c.id) { e.push('component ' + c.id + ': parent chain has a cycle'); break; }
      seen.add(x.id);
    }
  }

  // `ref` params name another component of a given type (an emitter's template workpiece).
  for (const c of scene.components) {
    const t = c && TYPES[c.type];
    if (!t) continue;
    for (const d of t.params) {
      const v = d.type === 'ref' ? c.params?.[d.k] : null;
      if (typeof v !== 'string' || !v) continue;
      const tgt = ids.get(v);
      if (!tgt) e.push('component ' + c.id + ': params.' + d.k + ' names ' + JSON.stringify(v) + ', which does not exist');
      // `of` is normally a type name. `of: 'part'` is the descriptor instead, so any emittable
      // type qualifies (a workpiece or a pallet), which is what partRoles() already assumes.
      else if (d.of === 'part' ? !TYPES[tgt.type]?.part : tgt.type !== d.of) {
        e.push('component ' + c.id + ': params.' + d.k + ' must name a ' + d.of + ', not the ' + tgt.type + ' ' + v);
      }
    }
  }

  // One writer per tag. A tag the plant writes (in) must not also be a PLC output, and two
  // readers of one PLC output must agree on its type.
  /** @type {Map<string, any[]>} */
  const byTag = new Map();
  for (const b of bindings(scene)) {
    if (b.unknown) continue;
    if (!TAG_RE.test(b.tag)) { e.push('tag ' + JSON.stringify(b.tag) + ' is not a PLC identifier'); continue; }
    /** @type {any[]} */ (byTag.get(b.tag) || byTag.set(b.tag, []).get(b.tag)).push(b);
  }
  const who = (/** @type {any} */ b) => (b.comp ? b.comp + '.' + b.key : b.station ? 'station ' + b.station : 'cycle.' + b.key + 'Tag');
  for (const [tag, bs] of byTag) {
    const writers = bs.filter(b => b.dir === 'in');
    if (writers.length > 1) e.push('two components writing the same tag ' + tag + ': ' + writers.map(who).join(', '));
    if (writers.length && writers.length < bs.length) e.push('tag ' + tag + ' is written by the plant (' + who(writers[0]) + ') AND by the PLC (' + bs.filter(b => b.dir === 'out').map(who).join(', ') + ')');
    if (new Set(bs.map(b => b.type)).size > 1) e.push('tag ' + tag + ' is used with different types: ' + bs.map(b => who(b) + ' ' + b.type).join(', '));
  }

  const st = new Set();
  for (const s of scene.stations || []) {
    if (!s || typeof s.id !== 'string') { e.push('station without id'); continue; }
    if (st.has(s.id)) e.push('duplicate station ' + s.id);
    st.add(s.id);
    for (const m of s.members || []) if (!ids.has(m)) e.push('station ' + s.id + ': member ' + m + ' does not exist');
  }
  for (const c of scene.components) if (c && c.station != null && !st.has(c.station)) e.push('component ' + c.id + ': station ' + c.station + ' does not exist');
  if (scene.cycle?.exitTag && !byTag.has(scene.cycle.exitTag)) e.push('cycle.exitTag ' + scene.cycle.exitTag + ' is not bound to any component');
  return e;
}

// ---------------------------------------------------------------------------- saving

const TOP = ['format', 'name', 'sim', 'io', 'stations', 'cycle', 'components'];
const COMP = ['id', 'type', 'label', 'station', 'parent', 'socket', 'at', 'rot', 'params', 'io'];
const SCENE_IO = ['driver', 'endpoint', 'prefix', 'mode', 'minPulseMs'];

/** Known keys in `first` order, then the rest alphabetically. @param {any} o @param {string[]} first */
function ordered(o, first) {
  const keys = Object.keys(o).filter(k => o[k] !== undefined);
  /** @type {Record<string, any>} */
  const r = {};
  for (const k of [...first.filter(k => keys.includes(k)), ...keys.filter(k => !first.includes(k)).sort()]) r[k] = o[k];
  return r;
}

/** The scene with a stable key order: params in the type's own order, io in its schema order. @param {Scene} scene */
export function canonical(scene) {
  const s = ordered(scene, TOP);
  if (s.sim) s.sim = ordered(s.sim, []);
  if (s.io) s.io = ordered(s.io, SCENE_IO);
  if (s.stations) s.stations = s.stations.map((/** @type {any} */ x) => ordered(x, ['id', 'name', 'members', 'stepTag']));
  if (s.cycle) s.cycle = ordered(s.cycle, ['exitTag', 'autoTag', 'countTag', 'exclude', 'avgN']);
  if (s.components) {
    s.components = s.components.map((/** @type {Comp} */ c) => {
      const t = TYPES[c.type], o = ordered(c, COMP);
      if (o.params) {
        o.params = ordered(o.params, t ? t.params.map((/** @type {any} */ d) => d.k) : []);
        if (Array.isArray(o.params.switches)) o.params.switches = o.params.switches.map((/** @type {any} */ w) => ordered(w, ['id', 'pos', 'band', 'words']));
      }
      if (o.io) o.io = ordered(o.io, t && t.io ? Object.keys(t.io(params(c))) : []);
      return o;
    });
  }
  return s;
}

/** @param {any} v @returns {string} */
function inline(v) {
  if (v === null || typeof v !== 'object') return JSON.stringify(v);
  if (Array.isArray(v)) return '[' + v.map(inline).join(', ') + ']';
  const e = Object.entries(v).filter(([, x]) => x !== undefined);
  return e.length ? '{ ' + e.map(([k, x]) => JSON.stringify(k) + ': ' + inline(x)).join(', ') + ' }' : '{}';
}

/** Inline whatever fits in 100 columns, else one entry per line. @param {any} v @param {string} ind @param {number} [key] @returns {string} */
function fmt(v, ind, key = 0) {
  const one = inline(v);
  if (v === null || typeof v !== 'object' || ind.length + key + one.length <= 100) return one;
  const ni = ind + '  ';
  if (Array.isArray(v)) return '[\n' + v.map(x => ni + fmt(x, ni)).join(',\n') + '\n' + ind + ']';
  const e = Object.entries(v).filter(([, x]) => x !== undefined);
  return '{\n' + e.map(([k, x]) => { const kk = JSON.stringify(k) + ': '; return ni + kk + fmt(x, ni, kk.length); }).join(',\n') + '\n' + ind + '}';
}

/** Saving twice gives byte-identical files. @param {Scene} scene */
export function stringify(scene) {
  return fmt(canonical(scene), '') + '\n';
}
