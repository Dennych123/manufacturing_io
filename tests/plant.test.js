// server/plant.js with the internal controller: the cyl-on-slide sequence end to end, stroke
// and reed timing as the PLC would see it, pulse stretching, the IO exchange, determinism.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { worldPoses } from '../lib/scene.js';
import { cylSpeeds, withDefaults, TYPES } from '../lib/components.js';

const ROOT = path.join(path.dirname(fileURLToPath(import.meta.url)), '..');
let fail = 0;
const chk = (l, c, x) => { if (!c) fail++; console.log((c ? '  OK  ' : '>>BAD ') + l + (x ? '   ' + x : '')); };

let P;
try {
  P = await import('../server/plant.js');
  await P.loadRapier();
} catch (e) {
  console.log('  SKIP  Rapier not loadable (' + String(e.message || e).split('\n')[0] + '): run npm install');
  process.exit(0);
}
const { createPlant, SK } = P;
const { create } = await import('../scenes/cyl-on-slide.ctl.js');
const scene = JSON.parse(fs.readFileSync(path.join(ROOT, 'scenes', 'cyl-on-slide.json'), 'utf8'));
const clone = o => JSON.parse(JSON.stringify(o));
const edges = (p, tag, v) => p.events.filter(e => (e.k === 'in' || e.k === 'out') && e.tag === tag && (v === undefined || e.v === v)).map(e => e.t);

async function cycle(sc = scene, ms = 9000) {
  const p = await createPlant(sc, { controller: create() });
  p.run(200);
  p.press('pbStart', 'pb', true); p.run(150); p.press('pbStart', 'pb', false);
  p.run(ms);
  return p;
}

const p = await cycle();
chk('the START pushbutton starts the sequence', p.events.some(e => e.k === 'step' && e.st === 'ST1' && e.v === 10));
chk('the sequence completes cycles (CYCLE_CNT counts)', p.io.CYCLE_CNT >= 1, 'CYCLE_CNT ' + p.io.CYCLE_CNT);
chk('the start lamp follows AUTO_RUN', p.io.PL_START === true && p.io.AUTO_RUN === true);
chk('servo replies (Done/Busy/InPos) are not pulse-stretched: no warnings in normal cycles', !p.events.some(e => e.k === 'warn'),
  p.events.filter(e => e.k === 'warn').map(e => e.msg).join(' | '));

// SOL -> AS as the PLC sees it. `ext` switch at 97 with band 6 turns ON at x = 94:
//   valveMs + 92 / v + 2 / (k v)   (full speed up to stroke - cushion = 92, then cushion speed)
const cp = withDefaults(TYPES.cylinder, scene.components.find(c => c.id === 'cyl1').params);
const v = cylSpeeds(cp);
const expect = cp.valveMs + 92 / v.ext + 2 / (v.k * v.ext);
const sol = edges(p, 'SOL_ST1_PRSS_CYL_DN', true)[0], as = edges(p, 'AS_ST1_PRSS_CYL_DN', true)[0];
chk('SOL -> AS delay = stroke model within 1 step', Math.abs((as - sol) - expect) <= p.dtMs, (as - sol) + ' ms vs ' + expect.toFixed(1));

const moved = clone(scene);
moved.components.find(c => c.id === 'cyl1').params.switches[1] = { id: 'ext', pos: 60 };
const q = await cycle(moved, 3000);
const d2 = edges(q, 'AS_ST1_PRSS_CYL_DN', true)[0] - edges(q, 'SOL_ST1_PRSS_CYL_DN', true)[0];
const expect2 = cp.valveMs + 57 / v.ext;
chk('moving the reed switch to 60 moves the edge the PLC sees', Math.abs(d2 - expect2) <= q.dtMs, d2 + ' ms vs ' + expect2.toFixed(1));

p.press('pbStop', 'pb', true); p.run(150); p.press('pbStop', 'pb', false);
p.run(6000);
chk('STOP ends the cycle at home', p.io.ST1_STEP === 0 && p.io.AUTO_RUN === false && p.dof.slide1 === 0 && p.dof.cyl1 === 0);

// Rapier bodies follow worldPoses (mid-motion, so it is not the initial pose by luck)
const k = await createPlant(scene, { controller: create() });
k.run(100); k.press('pbStart', 'pb', true); k.run(700);
const W = worldPoses(scene, k.dof);
const rod = k.bodies.find(b => b.id === 'cyl1' && b.link === 'rod');
const car = k.bodies.find(b => b.id === 'slide1' && b.link === 'carriage');
const err = b => { const t = b.body.translation(), w = W[b.id][b.link].p; return Math.max(Math.abs(t.x / SK - w[0]), Math.abs(t.y / SK - w[1]), Math.abs(t.z / SK - w[2])); };
chk('carriage body lines up with its link (mid-move)', k.dof.slide1 > 10 && err(car) < 0.01, 'slide ' + k.dof.slide1.toFixed(1) + ' mm, err ' + err(car).toExponential(1));
chk('rod body rides the carriage', rod.kinematic && err(rod) < 0.01, 'err ' + err(rod).toExponential(1));
chk('the frame is a fixed body', !k.bodies.find(b => b.id === 'base').kinematic);

// A blip shorter than minPulseMs is held for minPulseMs, plus a warning
const MIN = scene.io.minPulseMs, BLIP = MIN - 8;
const b = await createPlant(scene, {});
const warns = [];
b.warnListeners.push(m => warns.push(m));
b.run(500);
b.force('PB_STOP', true); b.run(BLIP); b.force('PB_STOP', null); b.run(300);
const on = edges(b, 'PB_STOP', true), off = edges(b, 'PB_STOP', false);
chk(BLIP + ' ms blip is held for minPulseMs ' + MIN, BLIP > 0 && on.length === 1 && off.length === 1 && off[0] - on[0] === MIN, on + ' -> ' + off);
chk('the stretch is recorded as a warn', warns.some(m => m.includes('pulse stretched PB_STOP ' + BLIP + ' -> ' + MIN + ' ms')), warns.join(' | '));
chk('the warn is in the event log too', b.events.some(e => e.k === 'warn' && /PB_STOP/.test(e.msg)));

// Determinism: same inputs -> identical event logs
const p2 = await cycle();
chk('two runs give identical event logs', JSON.stringify(p.events.slice(0, p2.events.length)) === JSON.stringify(p2.events) && p2.events.length > 50,
  p2.events.length + ' events');

// IO exchange through a fake driver
const writes = [];
let release;
const driver = { write: batch => { writes.push(batch); return new Promise(r => { release = r; }); }, status: () => ({ driver: 'fake', ok: true }) };
let now = 1e6;
const x = await createPlant(scene, { driver, wall: () => now });
x.driverUp();
x.exchange();
chk('driverUp writes every sensor once', writes.length === 1 && writes[0].length === x.inTags.length, writes[0]?.length + ' / ' + x.inTags.length);
x.force('PB_START', true); x.run(4);
x.exchange();
chk('at most one batch in flight', writes.length === 1);
release(); await new Promise(r => setTimeout(r, 0));
x.exchange();
chk('the next change goes out once the batch landed', writes.length === 2 && writes[1].some(w => w.name === 'PB_START' && w.value === true));
release(); await new Promise(r => setTimeout(r, 0));
const xw = [];
x.warnListeners.push(m => xw.push(m));
now += 40; x.plcSaw('PB_START', false);
chk('no overwrite warning while the write may still be in transit', xw.length === 0);
now += 200; x.plcSaw('PB_START', false);
chk('PLC holding another value -> "overwritten by the PLC"', xw.some(m => /PB_START overwritten by the PLC/.test(m)), xw.join(' | '));

now += 1000;
x.fromPlc('SOL_ST1_PRSS_CYL_DN', true, now - 30); x.run(2);
const ev = x.events.find(e => e.k === 'out' && e.tag === 'SOL_ST1_PRSS_CYL_DN');
chk('PLC output applied at the next step, stamped with its source time', x.io.SOL_ST1_PRSS_CYL_DN === true && ev && ev.tp === ev.t - 32, JSON.stringify(ev));
x.fromPlc('AS_ST1_PRSS_CYL_UP', true); x.run(2);
chk('the PLC cannot write a sensor tag through fromPlc', x.events.every(e => !(e.k === 'out' && e.tag === 'AS_ST1_PRSS_CYL_UP')));

// Real-time pacing: a stalled second is capped at 50 steps and counted
let clk = 0;
const r = await createPlant(scene, { clock: () => clk });
r.start(); clk = 1000;
await new Promise(res => setTimeout(res, 30));
r.stop();
chk('a 1 s stall runs 50 steps and counts the rest as overruns', r.t === 100 && r.overruns === 450, 't ' + r.t + ' ms, overruns ' + r.overruns);

// Static: physics never writes to the PLC. driver.write appears once, inside exchange().
const src = fs.readFileSync(path.join(ROOT, 'server', 'plant.js'), 'utf8');
const calls = src.match(/driver\.write\(/g) || [];
const ex = src.slice(src.indexOf('function exchange('), src.indexOf('function fromPlc('));
chk('driver.write is called only from exchange()', calls.length === 1 && ex.includes('driver.write('));

for (const pl of [p, p2, q, k, b, x, r]) await pl.close();
process.exit(fail ? 1 : 0);
