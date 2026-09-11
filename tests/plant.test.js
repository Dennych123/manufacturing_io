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

// ---------------------------------------------------------------- loose parts (P3)
{
  const sc = clone(scene);
  sc.components.push({ id: 'drop1', type: 'workpiece', parent: 'base', socket: 'top', at: [300, 150, 100], params: { dynamic: true, material: 'alu' } });
  sc.components.push({ id: 'lost1', type: 'workpiece', at: [3000, 0, 500], params: { dynamic: true } });
  const pl = await createPlant(sc, {});
  chk('a dynamic workpiece is a loose part, not a machine body', pl.parts.has('drop1') && !pl.bodies.some(b => b.id === 'drop1'));
  chk('the static press part stays a machine body', pl.bodies.some(b => b.id === 'part1') && !pl.parts.has('part1'));
  const b = pl.parts.get('drop1').body;
  chk('loose parts never sleep and use CCD', b.isSleeping() === false && b.isCcdEnabled());
  pl.run(1500);
  const z = b.translation().z / SK;
  chk('dropped 100 mm onto the frame top, it rests there (z 800 mm)', Math.abs(z - 800) < 0.5, z.toFixed(3) + ' mm');
  chk('a part that falls off the machine is removed and reported', !pl.parts.has('lost1') && pl.events.some(e => e.k === 'warn' && /part lost lost1/.test(e.msg))
    && pl.events.some(e => e.k === 'part' && e.uid === 'lost1' && e.ev === 'lost'));
  const snap = pl.snapshot();
  chk('snapshot streams loose-part transforms in mm with their template', Math.abs(snap.parts.drop1[2] - 800) < 0.5 && snap.ptpl.drop1 === 'drop1' && !('part1' in snap.parts));
  const rest = { ...b.translation() };                      // `b` is freed by reset(): read it first
  pl.reset();
  // Rapier stores f32: 0.9 m reads back as 899.99997 mm.
  chk('reset puts every scene part back at its start pose', pl.parts.has('lost1') && Math.abs(pl.parts.get('drop1').body.translation().z / SK - 900) < 0.01);
  const pl3 = await createPlant(sc, {}); pl3.run(1500);
  const a = pl3.parts.get('drop1').body.translation();
  chk('two runs give bit-identical part poses', a.x === rest.x && a.y === rest.y && a.z === rest.z);
  const bal = ev => pl3.events.filter(e => e.k === 'part' && e.ev === ev).length;
  chk('part balance: spawned = removed + lost + inside', bal('spawn') === bal('remove') + bal('lost') + bal('reset') + pl3.parts.size, [bal('spawn'), bal('lost'), pl3.parts.size].join(' '));
  await pl.close(); await pl3.close();
}

// ---------------------------------------------------------------- material flow: emitter -> belt -> remover
{
  const flow = (run = true) => ({
    format: 'mio-scene/1', name: 'flowtest',
    components: [
      { id: 'cv1', type: 'conveyor', params: { length: 1500, speed: 300 }, io: { run: 'CV1_RUN' } },
      { id: 'wp', type: 'workpiece', at: [0, 1000, 0], params: { material: 'alu' } },
      { id: 'em1', type: 'emitter', parent: 'cv1', socket: 'start', at: [100, 0, 60], params: { template: 'wp', intervalMs: 1500, max: 3 }, io: { count: 'EM1_CNT' } },
      { id: 'rm1', type: 'remover', parent: 'cv1', socket: 'end', at: [-75, 0, 0], io: { count: 'RM1_CNT' } },
    ],
  });
  const { validate: v2 } = await import('../lib/scene.js');
  const bad = flow(); bad.components[2].params.template = 'cv1';
  chk('an emitter template must name a workpiece', v2(flow()).length === 0 && v2(bad).some(e => /template must name a workpiece/.test(e)), v2(flow()).join('; '));
  const go = async () => { const pl = await createPlant(flow(), {}); pl.force('CV1_RUN', true); pl.run(10000); return pl; };
  const pl = await go();
  const ev = pl.events.filter(e => e.k === 'part');
  const spawns = ev.filter(e => e.ev === 'spawn'), removes = ev.filter(e => e.ev === 'remove');
  chk('the template is neither a machine body nor a part', !pl.bodies.some(b => b.id === 'wp') && !spawns.some(e => e.uid === 'wp'));
  // The first part leaves at the first step, then one every 1500 ms of sim time.
  chk('the emitter spawns max 3 parts, every 1.5 s', spawns.length === 3 && spawns.map(e => e.t).join() === '2,1500,3000', spawns.map(e => e.uid + '@' + e.t).join(' '));
  // spawn at x -650 (60 mm above the belt), removed once the centre passes x 600: 1250 mm at
  // 300 mm/s, plus the fall and the mu*g spin-up.
  const trip = removes[0] && removes[0].t - spawns[0].t;
  chk('the belt carries a part 1250 mm to the remover in 1250/300 s + fall + spin-up', trip > 4167 && trip < 4500, trip + ' ms');
  chk('every part reaches the remover; counters match', removes.length === 3 && pl.io.RM1_CNT === 3 && pl.io.EM1_CNT === 3 && pl.parts.size === 0, pl.io.RM1_CNT + ' / ' + pl.io.EM1_CNT);
  chk('no warnings on the way', !pl.events.some(e => e.k === 'warn'), pl.events.filter(e => e.k === 'warn').map(e => e.msg).join(' | '));
  chk('the stripes DOF is the belt travel (300 mm/s)', Math.abs(pl.dof.cv1 - 300 * 9.95) < 40, pl.dof.cv1.toFixed(0) + ' mm');
  const pl2 = await go();
  chk('two runs give identical event logs (parts included)', JSON.stringify(pl2.events) === JSON.stringify(pl.events), pl.events.length + ' events');
  const yaw = await createPlant(flow(), {}); yaw.force('CV1_RUN', true); yaw.run(4000);
  const yq = yaw.parts.get('em1.1').body.rotation(), yawDeg = 2 * Math.atan2(yq.z, yq.w) * 180 / Math.PI;
  chk('the belt\'s friction torque keeps a riding part square (|yaw| < 0.5° after 4 s)', Math.abs(yawDeg) < 0.5, yawDeg.toFixed(3) + '°');
  await yaw.close();
  const still = await createPlant(flow(), {});
  still.run(3000);
  const x = still.parts.get('em1.1').body.translation().x / SK;
  chk('a stopped belt holds the part where it landed', Math.abs(x - (-650)) < 2, x.toFixed(2) + ' mm');
  await pl.close(); await pl2.close(); await still.close();

  // ---------------------------------------------------------------- part sensors
  // A photo-eye 200 mm before the belt end, 15 mm above the belt, looking across it (+Y).
  const withEye = (params = {}) => {
    const sc = flow();
    sc.components.push({ id: 'pe1', type: 'photoEye', parent: 'cv1', socket: 'end', at: [-200, -140, 15], rot: [0, 0, 90], params: { range: 300, ...params }, io: { out: 'PE1' } });
    return sc;
  };
  const eye = async params => { const q = await createPlant(withEye(params), {}); q.force('CV1_RUN', true); q.run(9000); return q; };
  const pe = await eye();
  const on = edges(pe, 'PE1', true), off = edges(pe, 'PE1', false);
  // beam at x 550: the 60 mm part's front reaches it when its centre is at 520 (1170 mm from the spawn)
  chk('the photo-eye sees each part once, when its front reaches the beam', on.length === 3 && on[0] > 3900 && on[0] < 4300, on.join(' ') + ' ms');
  const dur = off[0] - on[0];
  chk('it stays on while the 60 mm part crosses at 300 mm/s (200 ms)', Math.abs(dur - 200) <= 8, dur + ' ms');
  chk('the beam sees parts only: the belt guides and frame never trip it', !edges(pe, 'PE1', true).some(t => t < 3900));
  const pd = await eye({ offDelayMs: 50 });
  const d2 = edges(pd, 'PE1', false)[0] - edges(pd, 'PE1', true)[0];
  chk('offDelayMs 50 holds the output 50 ms longer', Math.abs(d2 - 250) <= 8, d2 + ' ms');
  const pn = await eye({ logic: 'NC' });
  chk('NC: on while clear, off while a part is in the beam', pn.io.PE1 === true && edges(pn, 'PE1', false).length === 3);
  await pe.close(); await pd.close(); await pn.close();

  // Proximity: a steel and a plastic part at rest, each 4 mm in front of a sensor face.
  const prox = metalOnly => ({
    format: 'mio-scene/1', name: 'proxtest',
    components: [
      { id: 'base', type: 'frame', params: { size: [600, 400, 800] } },
      { id: 'steel', type: 'workpiece', parent: 'base', socket: 'top', at: [-100, 0, 0], params: { dynamic: true, material: 'steel' } },
      { id: 'plastic', type: 'workpiece', parent: 'base', socket: 'top', at: [100, 0, 0], params: { dynamic: true, material: 'plastic' } },
      { id: 'px1', type: 'proximity', parent: 'base', socket: 'top', at: [-134, 0, 15], params: { metalOnly }, io: { out: 'PX_STEEL' } },
      { id: 'px2', type: 'proximity', parent: 'base', socket: 'top', at: [66, 0, 15], params: { metalOnly }, io: { out: 'PX_PLASTIC' } },
    ],
  });
  const px = await createPlant(prox(true), {}); px.run(500);
  chk('inductive (metalOnly) proximity sees steel, not plastic', px.io.PX_STEEL === true && px.io.PX_PLASTIC === false);
  const pc = await createPlant(prox(false), {}); pc.run(500);
  chk('capacitive (metalOnly off) proximity sees both', pc.io.PX_STEEL === true && pc.io.PX_PLASTIC === true);
  await px.close(); await pc.close();
}

// ---------------------------------------------------------------- pusher and stopper presets
{
  const push = {
    format: 'mio-scene/1', name: 'pushtest',
    components: [
      { id: 'base', type: 'frame', params: { size: [1000, 600, 800] } },
      { id: 'part', type: 'workpiece', parent: 'base', socket: 'top', at: [30, 0, 0], params: { dynamic: true } },
      // horizontal along +X; retracted, the plate face sits 20 mm short of the part
      { id: 'push', type: 'cylinder', parent: 'base', socket: 'top', at: [-250, 0, 40], rot: [0, 90, 0],
        params: { ...TYPES.cylinder.presets.find(p => p.label === 'Pusher').params, headSize: [40, 60, 8] }, io: { solExt: 'SOL_PUSH' } },
    ],
  };
  const { validate: v3 } = await import('../lib/scene.js');
  chk('every cylinder preset is a valid parameter set', TYPES.cylinder.presets.every(pr => v3({ ...push, components: [{ id: 'c', type: 'cylinder', params: pr.params }] }).length === 0));
  const pp = await createPlant(push, {});
  pp.run(300);
  const x0 = pp.parts.get('part').body.translation().x / SK;
  pp.force('SOL_PUSH', true); pp.run(1500);
  const x1 = pp.parts.get('part').body.translation().x / SK;
  // Extended, the face is at -20 + 150 = 130, so the part (60 long, origin at its centre) is
  // pushed to at least 160. The plate slows into the cushion 5 mm before the end (part at 155);
  // the part leaves it at full speed and slides v^2/(2 mu g) on the frame (mu 0.5): real physics.
  const pv = cylSpeeds(withDefaults(TYPES.cylinder, push.components[2].params)).ext * 1000 / 1000;   // m/s
  const slide = pv * pv / (2 * 0.5 * 9.81) * 1000;
  chk('the pusher plate really pushes the part (kinematic head vs dynamic part)', x1 >= 159.5 && x1 <= 155 + slide + 1,
    x0.toFixed(1) + ' -> ' + x1.toFixed(1) + ' mm (160 .. ' + (155 + slide).toFixed(1) + ')');
  await pp.close();
}

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
