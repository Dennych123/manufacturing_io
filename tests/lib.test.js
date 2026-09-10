// lib/: math, scene validation, worldPoses, the actuator models, stable saving.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { pose, compose, invert, apply, qeuler } from '../lib/math.js';
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
broken(s => { comp(s, 'pbStop').io.pb = 'PB_START'; }, /two components writing the same tag PB_START/, 'two writers of one tag');
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
const shuffled = clone(scene);
shuffled.components[3] = Object.fromEntries(Object.entries(shuffled.components[3]).reverse());
chk('key order in the input does not change the output', stringify(shuffled) === once);

process.exit(fail ? 1 : 0);
