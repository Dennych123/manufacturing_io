// lib/: math, scene validation, worldPoses, the actuator models, stable saving.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { pose, compose, invert, apply, qeuler, qrot, eulerOf } from '../lib/math.js';
import { TYPES, trapStep, withDefaults, cylSpeeds } from '../lib/components.js';
import { validate, worldPoses, bindings, tags, stringify } from '../lib/scene.js';

const ROOT = path.join(path.dirname(fileURLToPath(import.meta.url)), '..');
let fail = 0;
const chk = (l, c, x) => { if (!c) fail++; console.log((c ? '  OK  ' : '>>BAD ') + l + (x ? '   ' + x : '')); };
const near = (a, b, tol = 1e-9) => a.length === b.length && a.every((v, i) => Math.abs(v - b[i]) <= tol);
const fmt = v => '[' + v.map(n => +n.toFixed(4)).join(', ') + ']';
const clone = o => JSON.parse(JSON.stringify(o));

// ---------------------------------------------------------------- math
const a = pose([10, 20, 30], [30, -45, 60]), b = pose([-5, 7, 1], [0, 90, 10]);
const ab = compose(a, b);
chk('compose(a, invert(a)) = identity', near(compose(a, invert(a)).p, [0, 0, 0]) && near(compose(a, invert(a)).q.slice(0, 3), [0, 0, 0]));
chk('apply(compose(a, b), v) = apply(a, apply(b, v))', near(apply(ab, [1, 2, 3]), apply(a, apply(b, [1, 2, 3]))));
chk('rot [90,0,0] turns +Y into +Z', near(apply({ p: [0, 0, 0], q: qeuler([90, 0, 0]) }, [0, 1, 0]), [0, 0, 1]));
chk('rot is X THEN Y about fixed axes: [90,90,0] turns +Y into +X', near(apply({ p: [0, 0, 0], q: qeuler([90, 90, 0]) }, [0, 1, 0]), [1, 0, 0]),
  fmt(apply({ p: [0, 0, 0], q: qeuler([90, 90, 0]) }, [0, 1, 0])));

// ---------------------------------------------------------------- scene
const scene = JSON.parse(fs.readFileSync(path.join(ROOT, 'scenes', 'cyl-on-slide.json'), 'utf8'));
const errs = validate(scene);
chk('cyl-on-slide validates', errs.length === 0, errs.join(' | '));

// worldPoses caches the components that can never move. Most of a scene is furniture, and
// recomputing it every 2 ms cost more than the machine did (palletizing: 2276 -> 663 us a step).
{
  const a = worldPoses(scene, { cyl1: 0, slide1: 0 });
  const b = worldPoses(scene, { cyl1: 50, slide1: 300 });
  chk('worldPoses caches a component that never moves, and hands back the same poses', a.base === b.base && a.base.body.p.join() === b.base.body.p.join());
  chk('worldPoses still moves what has a DOF', a.cyl1.rod.p.join() !== b.cyl1.rod.p.join() && a.slide1.carriage.p.join() !== b.slide1.carriage.p.join(),
    a.cyl1.rod.p.join() + ' vs ' + b.cyl1.rod.p.join());
  // A component mounted on a moving link must NOT be cached: the cylinder rides the slide.
  chk('a component mounted on a moving link is never cached', a.cyl1.body.p.join() !== b.cyl1.body.p.join(),
    a.cyl1.body.p.join() + ' vs ' + b.cyl1.body.p.join());
  chk('a part standing on the frame is cached (nothing under it moves)', a.part1 === b.part1);
}

const broken = (f, re, label) => {
  const s = clone(scene); f(s);
  const e = validate(s);
  chk('validate rejects ' + label, e.some(m => re.test(m)), e.join(' | ') || 'no error');
};
const comp = (s, id) => s.components.find(c => c.id === id);
broken(s => { comp(s, 'cyl1').parent = 'nope'; }, /parent "nope" does not exist/, 'a missing parent');
broken(s => { comp(s, 'base').parent = 'cyl1'; }, /cycle/, 'a parent cycle');
broken(s => { comp(s, 'part1').type = 'unicorn'; }, /unknown type/, 'an unknown type');
broken(s => { comp(s, 'part1').id = 'base'; }, /duplicate id base/, 'duplicate ids');
broken(s => { comp(s, 'pbCstop').io.pb = 'PB_START'; }, /two components writing the same tag PB_START/, 'two writers of one tag');
broken(s => { comp(s, 'part1').params = { size: [60, 0, 30] }; }, /size must be positive/, 'a zero-size workpiece (the emitter column check would loop for ever)');
broken(s => { s.sim = { ...s.sim, handMmS: 0 }; }, /sim.handMmS must be/, 'a hand speed of zero (a drag that never arrives)');

// ---------------------------------------------------------------- the robot arm chain (rb4axis)
// The plant keeps ONE dof per component, so an arm is a CHAIN of joints and every angle is
// relative to its parent - which is exactly how rb4axis's chainPoints() composes them
// (a2 = a1 + pos[2]). Change the joint's mount convention and the arm lands somewhere else in
// silence, so the closed form is pinned here: L1 400 lifts the shoulder above the carriage, then
// L2 300, L3 250 and L4 100 turn about X in the Y-Z plane. Home is [0, 90, -90, -90].
{
  const L1 = 400, L2 = 300, L3 = 250, L4 = 100;
  const arm = {
    format: 'mio-scene/1', name: 'armtest',
    components: [
      { id: 'rail', type: 'joint', params: { kind: 'prismatic', axis: 'x', len: 0, min: -1500, max: 1500, home: 0 } },
      { id: 'j1', type: 'joint', parent: 'rail', socket: 'end', at: [0, 0, L1],
        params: { kind: 'revolute', axis: 'x', len: L2, min: -90, max: 180, home: 90 } },
      { id: 'j2', type: 'joint', parent: 'j1', socket: 'end',
        params: { kind: 'revolute', axis: 'x', len: L3, min: -150, max: 0, home: -90 } },
      { id: 'j3', type: 'joint', parent: 'j2', socket: 'end',
        params: { kind: 'revolute', axis: 'x', len: L4, min: -120, max: 120, home: -90 } },
    ],
  };
  chk('the robot arm chain validates', validate(arm).length === 0, validate(arm).join(' | '));
  const H = worldPoses(arm, undefined);                       // no dof: every joint at its home
  chk('arm at home: shoulder 400, elbow 700, wrist out 250',
    near(H.j1.base.p, [0, 0, L1], 1e-6) && near(H.j2.base.p, [0, 0, L1 + L2], 1e-6) && near(H.j3.base.p, [0, L3, L1 + L2], 1e-6),
    fmt(H.j3.base.p));
  chk('arm at home: the flange hangs L4 below the wrist', near(apply(H.j3.arm, [0, L4, 0]), [0, L3, L1 + L2 - L4], 1e-6),
    fmt(apply(H.j3.arm, [0, L4, 0])));
  // rail 500, angles 90 / -45 / 0: the last two links run out together at 45 degrees
  const P = worldPoses(arm, { rail: 500, j1: 90, j2: -45, j3: 0 }), k = Math.SQRT1_2;
  chk('arm angles are cumulative, as chainPoints() composes them',
    near(apply(P.j3.arm, [0, L4, 0]), [500, (L3 + L4) * k, L1 + L2 + (L3 + L4) * k], 1e-6), fmt(apply(P.j3.arm, [0, L4, 0])));
  // the limits live in the model, because worldPoses() turns the dof straight into a rotation
  const jt = TYPES.joint, jp = withDefaults(jt, { kind: 'revolute', min: -90, max: 180, home: 0, vmax: 90, acc: 240 });
  const st = jt.init(jp), jio = { target: 999, exec: true };
  for (let i = 0; i < 4000; i++) jt.step(st, jp, jio, 0.002);
  chk('a joint clamps its target to the axis limits', Math.abs(st.x - 180) < 1e-6, st.x.toFixed(3));
}
broken(s => { comp(s, 'lampAuto').io.lamp = 'PB_START'; }, /PB_START is written by the plant .* AND by the PLC/, 'a tag both plant- and PLC-written');
broken(s => { comp(s, 'cyl1').socket = 'nose'; }, /has no socket "nose"/, 'an unknown socket');
broken(s => { comp(s, 'cyl1').params.boreDiameter = 32; }, /unknown parameter/, 'an unknown parameter (a typo is silent otherwise)');
broken(s => { comp(s, 'cyl1').params.bore = 33; }, /params.bore must be one of/, 'a bore that is not a catalogue size');
broken(s => { comp(s, 'cyl1').io['sw.mid'] = 'AS_X'; }, /io.sw.mid is not a cylinder signal/, 'an io key the type does not have');
broken(s => { comp(s, 'cyl1').params.switches[1].pos = 140; }, /outside the stroke/, 'a reed switch past the stroke');
broken(s => { comp(s, 'cyl1').params.extendMs = 10; }, /extendMs must be longer than valveMs/, 'a stroke time shorter than the valve');
broken(s => { s.cycle.exitTag = 'AS_NOWHERE'; }, /exitTag AS_NOWHERE is not bound/, 'an unbound cycle.exitTag');

const tg = tags(scene);
chk('tag directions come from the type schema', tg.get('SOL_ST1_PRSS_CYL_DN').dir === 'out' && tg.get('AS_ST1_PRSS_CYL_UP').dir === 'in'
  && tg.get('SV1_TGT').type === 'LREAL' && tg.get('ST1_STEP').type === 'INT');
chk('bindings: every scene io entry is known to its type', bindings(scene).every(b => !b.unknown));

// Rod end of the cylinder on the slide, computed by hand:
//   base top z 800; slide origin (0,-100,800); carriage link x = -640/2 + 140/2 = -250, z 800+0.6*70
//   = 842; carriage socket +0.4*70 -> z 870; bracket at (0,-56,0) -> y -156; bracket top +250 -> z 1120;
//   cylinder at (0,-30,0), rot 180 about X (points DOWN) -> (-250,-186,1120);
//   rod link at local z Lb = 100+32+20 = 152 -> world z 968; rodEnd +27 -> z 941.
//   slide 400 mm -> x = 150; rod 100 mm -> z 841.
const rodEnd = dof => {
  const w = worldPoses(scene, dof).cyl1.rod;
  return { at: apply(w, [0, 0, 27]), dir: apply({ p: [0, 0, 0], q: w.q }, [0, 0, 1]) };
};
const r0 = rodEnd({}), r1 = rodEnd({ slide1: 400, cyl1: 100 });
chk('rod end at rest = hand-computed (-250, -186, 941)', near(r0.at, [-250, -186, 941], 1e-9), fmt(r0.at));
chk('rod end, slide 400 + rod 100 = (150, -186, 841)', near(r1.at, [150, -186, 841], 1e-9), fmt(r1.at));
chk('the cylinder points down (rot 180 about X)', near(r0.dir, [0, 0, -1], 1e-12), fmt(r0.dir));
chk('the rod end meets the workpiece top (z 840) within 1 mm', Math.abs(r1.at[2] - 840) <= 1);
chk('worldPoses covers every component', Object.keys(worldPoses(scene, {})).length === scene.components.length);

// ---------------------------------------------------------------- cylinder model
const DT = 0.002;
function cyl(over) {
  const t = TYPES.cylinder, p = withDefaults(t, { stroke: 100, extendMs: 450, retractMs: 380, cushionMm: 8, valveMs: 15, ...over });
  return { t, p, s: t.init(p) };
}
/** steps until pred, with io fixed; returns ms */
function until(c, io, pred, max = 5000) {
  for (let i = 1; i <= max; i++) { c.t.step(c.s, c.p, io, DT); if (pred(c.s)) return i * DT * 1000; }
  return Infinity;
}
for (const valve of ['5/2-double', '5/2-single', '5/3-closed', 'single-acting']) {
  const c = cyl({ valve });
  const te = until(c, { solExt: true }, s => s.x >= 100);
  const tr = until(c, { solRet: true, solExt: false }, s => s.x <= 0);
  chk(valve + ': extend time = extendMs 450 within 1 step', Math.abs(te - 450) <= 2, te + ' ms');
  chk(valve + ': retract time = retractMs 380 within 1 step', Math.abs(tr - 380) <= 2, tr + ' ms');
}
{
  const c = cyl({});
  const tv = until(c, { solExt: true }, s => s.x > 0);
  chk('valve delay: the rod starts moving after valveMs 15', tv === 16, tv + ' ms (first step with x > 0)');
  const v = cylSpeeds(c.p);
  let x0 = c.s.x; c.t.step(c.s, c.p, { solExt: true }, DT);
  chk('full speed mid-stroke = v', Math.abs((c.s.x - x0) - v.ext * 2) < 1e-9, ((c.s.x - x0) / 2).toFixed(4) + ' mm/ms');
  until(c, { solExt: true }, s => s.x > 95);
  x0 = c.s.x; c.t.step(c.s, c.p, { solExt: true }, DT);
  chk('cushion speed in the last cushionMm = k * v', Math.abs((c.s.x - x0) - v.k * v.ext * 2) < 1e-9);
}
{
  const c = cyl({ valve: '5/2-double' });
  until(c, { solExt: true }, s => s.x >= 100);
  until(c, {}, () => false, 100);
  chk('5/2-double: neither solenoid -> holds its last position', c.s.x === 100);
  until(c, { solExt: true, solRet: true }, () => false, 100);
  chk('5/2-double: both solenoids -> still holds', c.s.x === 100);
  const s = cyl({ valve: '5/2-single' });
  until(s, { solExt: true }, x => x.x >= 100);
  const back = until(s, {}, x => x.x <= 0);
  chk('5/2-single: solenoid off -> spring return', back < 1000, back + ' ms');
  const m = cyl({ valve: '5/3-closed' });
  until(m, { solExt: true }, () => false, 100);
  const mid = m.s.x;
  until(m, {}, () => false, 100);
  chk('5/3-closed: no solenoid -> stops mid-stroke', mid > 20 && mid < 80 && Math.abs(m.s.x - mid) < 1, mid.toFixed(2) + ' -> ' + m.s.x.toFixed(2));
}
{
  // reed switch `ext` at pos 97, band 6, hysteresis 0.5: ON when |x - 97| <= 3
  const c = cyl({ valve: '5/3-closed', switches: [{ id: 'ext', pos: 97 }] });
  const at = (x, from) => { c.s.x = x; c.s.sw.ext = from; const io = {}; c.t.step(c.s, c.p, io, DT); return io['sw.ext']; };
  chk('reed: ON at the band edge (x = 94)', at(94, false) === true);
  chk('reed: OFF just outside the band (x = 93.99)', at(93.99, false) === false);
  chk('reed: stays ON inside the hysteresis (x = 93.6)', at(93.6, true) === true);
  chk('reed: drops past band + hysteresis (x = 93.4)', at(93.4, true) === false);
}

// ---------------------------------------------------------------- servo trapezoid
// Vectors produced by rb4axis web/kin.js langkahSumbu itself (docs/PLAN.md §3): [from, to, vmax, acc, dt],
// total steps, and {step: [pos, vel]}.
const VECTORS = [
  [[0, 400, 300, 1500, 0.002], 757, { 1: [0.006, 3], 50: [7.65, 150], 100: [30.300000000000004, 300], 500: [270.2999999999988, 300] }],
  [[400, 0, 300, 1500, 0.002], 757, { 1: [399.994, -3], 100: [369.7000000000003, -300] }],
  [[0, 5, 300, 1500, 0.002], 53, { 1: [0.006, 3], 20: [1.2600000000000002, 60], 40: [4.142868755441456, 54.67585252006619] }],
  [[10, 10.05, 300, 1500, 0.002], 5, { 1: [10.006, 3] }],
];
for (const [[from, to, vmax, acc, dt], steps, marks] of VECTORS) {
  let pos = from, vel = 0, n = 0, same = true;
  for (; n < 1e6; n++) {
    const r = trapStep(pos, to, vel, vmax, acc, dt); pos = r.pos; vel = r.vel;
    const m = marks[n + 1];
    if (m && (m[0] !== pos || m[1] !== vel)) same = false;
    if (!r.moving) break;
  }
  chk(`trapStep ${from} -> ${to} matches langkahSumbu bit for bit`, same && n + 1 === steps && pos === to, (n + 1) + ' steps');
}
{
  const t = TYPES.servoLinear, p = withDefaults(t, { stroke: 500 }), s = t.init(p), io = { target: 400, exec: false };
  const step = n => { for (let i = 0; i < n; i++) t.step(s, p, io, DT); };
  step(5);
  chk('servo: no Execute, no motion', s.x === 0 && io.done === false);
  io.exec = true; step(1); io.target = 100; step(10);
  chk('servo: the target is latched on the Execute edge (later changes ignored)', s.tgt === 400 && io.busy === true);
  step(2000);
  chk('servo: Done while Execute is held, actPos = target', io.done === true && io.actPos === 400 && io.inPos === true);
  io.exec = false; step(1);
  chk('servo: Done drops with Execute', io.done === false);
}
{
  const t = TYPES.pushbutton, p = withDefaults(t, { kind: 'alternate' }), s = t.init(p), io = {};
  t.press(s, p, 'pb', true); t.press(s, p, 'pb', false); t.step(s, p, io, DT);
  chk('alternate pushbutton latches on the first press', io.pb === true);
  t.press(s, p, 'pb', true); t.step(s, p, io, DT);
  chk('alternate pushbutton releases on the second', io.pb === false);
}

// ---------------------------------------------------------------- saving
const once = stringify(scene), twice = stringify(JSON.parse(once));
chk('two saves are byte-identical', once === twice);
chk('scenes/cyl-on-slide.json is in canonical form', fs.readFileSync(path.join(ROOT, 'scenes', 'cyl-on-slide.json'), 'utf8') === once);
for (const f of fs.readdirSync(path.join(ROOT, 'scenes')).filter(f => /^[a-z0-9_-]+\.json$/.test(f))) {
  const txt = fs.readFileSync(path.join(ROOT, 'scenes', f), 'utf8'), sc = JSON.parse(txt), errs = validate(sc);
  chk('scenes/' + f + ' is valid and canonical', errs.length === 0 && txt === stringify(sc) && sc.name + '.json' === f, errs.join('; '));
  // The panel controls every scene owes its machine. A regenerated scene silently dropped its
  // speed dial, its `ovr` bindings and its jog buttons once, and nothing failed: the machine simply
  // ran with three fewer controls than the others.
  {
    const by = (/** @type {string} */ ty) => sc.components.filter((/** @type {any} */ c) => c.type === ty);
    const motors = [...by('servoLinear'), ...by('conveyor'), ...by('indexTable')];
    const servos = by('servoLinear');
    const tags = new Set(sc.components.flatMap((/** @type {any} */ c) => Object.values(c.io || {})));
    if (motors.length) {
      const unbound = motors.filter((/** @type {any} */ c) => !c.io?.ovr).map((/** @type {any} */ c) => c.id);
      chk(sc.name + ': every motor takes the speed override', unbound.length === 0, unbound.join(' '));
      chk(sc.name + ': the panel has the speed dial', by('speedDial').length === 1, by('speedDial').length + ' dials');
    }
    for (const ax of servos) {
      chk(sc.name + ': ' + ax.id + ' can be jogged', !!ax.io?.jogP && !!ax.io?.jogN, JSON.stringify(ax.io));
      chk(sc.name + ': ' + ax.id + ' has a jog button each way on the panel',
        tags.has(ax.io?.jogP) && tags.has(ax.io?.jogN) && sc.components.some((/** @type {any} */ c) => c.io?.lamp === ax.io?.jogP));
    }
  }
}
const shuffled = clone(scene);
shuffled.components[3] = Object.fromEntries(Object.entries(shuffled.components[3]).reverse());
chk('key order in the input does not change the output', stringify(shuffled) === once);

// ---------------------------------------------------------------- conveyor side members
// Nothing solid beside the belt may reach the belt surface. Measured on sort-by-material: with
// the side members flush with the belt, steel #43 slid belt -> rail top -> off the rail edge and
// came to rest 12 mm BESIDE the rail at belt height, held up by a stale rail-top manifold
// (4 contacts, -0.03 mm, normal +Z) with nothing under it; the sequence then faulted for good.
{
  const t = TYPES.conveyor, p = withDefaults(t, { length: 2400, width: 200, height: 800, speed: 300, guides: 0 });
  const top = (/** @type {any} */ s) => s.at[2] + s.size[2] / 2;
  const shapes = t.shapes(p), belt = shapes.find((/** @type {any} */ s) => s.belt);
  const flush = shapes.filter((/** @type {any} */ s) => !s.belt && s.collide !== false && s.kind === 'box'
    && Math.abs(s.at[1]) >= p.width / 2 && top(s) >= top(belt) - 1e-9);
  chk('conveyor: no side member reaches the belt surface (a part pushed off the edge hangs on it)', flush.length === 0, JSON.stringify(flush));
  chk('conveyor: the side members are still there, just lower', shapes.some((/** @type {any} */ s) => s.size?.[1] === 30 && s.size?.[2] === 60));
}

// ---------------------------------------------------------------- index table (cam drive)
{
  const { cycloid } = await import('../lib/components.js');
  chk('cycloid starts and ends exactly on the station (s(0) = 0, s(1) = 1)', cycloid(0) === 0 && Math.abs(cycloid(1) - 1) < 1e-12);
  chk('cycloid is still at both ends (no jolt onto the station)', Math.abs(cycloid(0.001) / 0.001) < 0.02 && Math.abs((1 - cycloid(0.999)) / 0.001) < 0.02);

  const t = TYPES.indexTable, p = withDefaults(t, { stations: 6, camMs: 1200, indexFrac: 0.5 });
  const s = t.init(p), io = { run: true };
  const run = (n, on = true) => { io.run = on; for (let i = 0; i < n; i++) t.step(s, p, io, DT); };
  run(150);                                     // 300 ms: a quarter of the camshaft, mid-index
  chk('index table: mid-index it is between stations and not in position', s.x > 0 && s.x < 60 && io.inPos === false, s.x.toFixed(2) + '°');
  const held = s.x;
  run(100, false);                              // run drops mid-index
  chk('index table: dropping run mid-index leaves it where it stopped, still not in position', s.x === held && io.inPos === false, s.x.toFixed(2) + '°');
  run(151);                                     // resume and finish the index (600 ms in total)
  chk('index table: one index turns exactly one pitch (60°) and reports in position', Math.abs(s.x - 60) < 1e-9 && io.inPos === true && io.station === 1,
    s.x.toFixed(6) + '°, station ' + io.station);
  chk('index table: origin is only at station 0', io.origin === false);
  // one camshaft revolution = 600 steps at dt 2 ms = one more index, so five more revolutions
  // take the table from station 1 back round to origin
  run(3000);
  chk('index table: six indexes come back to origin at 360°', Math.abs(s.x - 360) < 1e-9 && io.station === 0 && io.origin === true,
    s.x.toFixed(6) + '°, station ' + io.station);
}

// eulerOf inverts qeuler (same rotation, compared as rotated vectors), incl. gimbal lock
{
  const near = (a, b) => a.every((v, i) => Math.abs(v - b[i]) < 1e-9);
  const same = (r1, r2) => [[1, 0, 0], [0, 1, 0], [0, 0, 1]].every(v => near(qrot(qeuler(r1), v), qrot(qeuler(r2), v)));
  const cases = [[0, 0, 0], [180, 0, 0], [30, -45, 120], [-170, 89, 5], [10, 90, 30], [10, -90, 30], [0, 0, -179], [90, 0, 90]];
  const badE = cases.filter(r => !same(r, eulerOf(qeuler(r))));
  chk('eulerOf(qeuler(rot)) is the same rotation (' + cases.length + ' cases, gimbal included)', badE.length === 0, JSON.stringify(badE));
  chk('eulerOf keeps simple angles simple', near(eulerOf(qeuler([30, -45, 120])), [30, -45, 120]) && near(eulerOf(qeuler([0, 0, 0])), [0, 0, 0]));
}

// mountFrom undoes mountPoses: a drag to the current frame gives back the same at/rot
{
  const { mountPoses, mountFrom } = await import('../lib/scene.js');
  const W = worldPoses(scene, { slide1: 120 });
  const bad = [];
  for (const c of scene.components) {
    const m = mountPoses(scene, { slide1: 120 }, c.id);
    const back = mountFrom(m.base, m.frame);
    const want = pose(c.at, c.rot);
    const got = pose(back.at, back.rot);
    if (!want.p.every((v, i) => Math.abs(v - got.p[i]) < 1e-3) || ![[1, 0, 0], [0, 0, 1]].every(v => qrot(want.q, v).every((x, i) => Math.abs(x - qrot(got.q, v)[i]) < 1e-6))) bad.push(c.id);
  }
  chk('mountFrom(mountPoses) returns every component\'s own at/rot', bad.length === 0, bad.join(' '));
  const cm = mountPoses(scene, { slide1: 120 }, 'cyl1');
  const moved = { p: [cm.frame.p[0] + 10, cm.frame.p[1], cm.frame.p[2]], q: cm.frame.q };
  const cyl = scene.components.find(c => c.id === 'cyl1');
  const nm = mountFrom(cm.base, moved);
  chk('a +10 mm world X drag of cyl1 (on the carriage) moves its at by +10 in X', Math.abs(nm.at[0] - ((cyl.at?.[0] ?? 0) + 10)) < 1e-3 && Math.abs(nm.at[1] - (cyl.at?.[1] ?? 0)) < 1e-3, JSON.stringify(nm));
  chk('mountPoses of the base frame is the world', mountPoses(scene, {}, 'base').base.p.every(v => v === 0) && !!W.base);
  chk('mountPoses of an unknown id is null', mountPoses(scene, {}, 'nope') === null);
}

process.exit(fail ? 1 : 0);
