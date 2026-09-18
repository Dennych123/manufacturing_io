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

/**
 * The panel start-up, in the order an operator does it: MASTER ON energises the machine, HOME
 * drives every actuator to its home position, and only then does START run the cycle. A machine
 * that has not been homed refuses to start, which is the point of the Home button.
 * @param {any} p @param {number} [homeMs] time allowed for the home step
 */
function powerUp(p, homeMs = 3000) {
  const tap = (/** @type {string} */ id) => { p.press(id, 'pb', true); p.run(120); p.press(id, 'pb', false); p.run(120); };
  tap('pbMaster');
  tap('pbHome');
  p.run(homeMs);
  tap('pbStart');
}

async function cycle(sc = scene, ms = 9000) {
  const p = await createPlant(sc, { controller: create() });
  p.run(200);
  powerUp(p);
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

p.press('pbCstop', 'pb', true); p.run(150); p.press('pbCstop', 'pb', false);
p.run(6000);
chk('STOP ends the cycle at home', p.io.ST1_STEP === 0 && p.io.AUTO_RUN === false && p.dof.slide1 === 0 && p.dof.cyl1 === 0);

// Rapier bodies follow worldPoses (mid-motion, so it is not the initial pose by luck)
const k = await createPlant(scene, { controller: create() });
k.run(100); powerUp(k); k.run(400);
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
  // A feeder emits PARTS, which is a descriptor: a pallet is one too. The validator used to match
  // the literal type name and refused a pallet feeder.
  const pal = flow();
  pal.components.push({ id: 'plt', type: 'pallet', at: [0, 1400, 0], params: { dynamic: true } });
  pal.components[2].params.template = 'plt';
  chk('an emitter template must name a part, and a pallet is one',
    v2(flow()).length === 0 && v2(pal).length === 0 && v2(bad).some(e => /template must name a part/.test(e)),
    v2(flow()).concat(v2(pal)).join('; ') + ' | ' + v2(bad).join('; '));
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
  // A feeder must not drop a part onto one already under it: the spot is a column, not a point.
  const stack = await createPlant(flow(), {});
  stack.run(8000);                                        // belt stopped: the first part stays under the feeder
  const zs = [...stack.parts.values()].map(p => p.body.translation().z / SK);
  chk('the emitter waits while a part sits under it instead of stacking', stack.parts.size === 1 && zs.every(z => z < 830),
    stack.parts.size + ' part(s), z ' + zs.map(z => z.toFixed(0)).join(' '));
  await stack.close();

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

// ---------------------------------------------------------------- holding: vacuum cup and nest
{
  // A cylinder pointing down on the frame, a cup on its rod end. Geometry: Lb = 120+25+20 = 165,
  // so the cup face sits at 1141 - (165+27) - x: 949 retracted, 829 extended, on a part whose
  // top is at 830.
  const pick = {
    format: 'mio-scene/1', name: 'picktest',
    components: [
      { id: 'base', type: 'frame', params: { size: [1000, 600, 800] } },
      { id: 'p1', type: 'workpiece', parent: 'base', socket: 'top', at: [0, 0, 0], params: { dynamic: true, material: 'plastic' } },
      { id: 'p2', type: 'workpiece', parent: 'base', socket: 'top', at: [300, 0, 30], params: { dynamic: true, size: [70, 50, 20] } },
      { id: 'pick', type: 'cylinder', parent: 'base', socket: 'top', at: [0, 0, 341], rot: [180, 0, 0],
        params: { bore: 25, stroke: 120, valve: '5/2-double' }, io: { solExt: 'SOL_DN', solRet: 'SOL_UP', 'sw.ext': 'AS_DN', 'sw.ret': 'AS_UP' } },
      { id: 'cup1', type: 'vacuumCup', parent: 'pick', socket: 'rodEnd', at: [0, 0, 0], params: { d: 20, reach: 2, buildMs: 80, dropMs: 60 }, io: { on: 'VAC_ON', vac: 'VAC_SW' } },
      { id: 'nest1', type: 'nest', parent: 'base', socket: 'top', at: [300, 0, 0], params: { size: [80, 60, 25] }, io: { present: 'NEST_P' } },
    ],
  };
  const pk = await createPlant(pick, {});
  pk.run(800);
  chk('a nest holds a part dropped into its pocket and reports it', pk.io.NEST_P === true && pk.parts.get('p2').held?.id === 'nest1'
    && pk.events.some(e => e.k === 'part' && e.ev === 'hold' && e.by === 'nest1'));
  pk.force('SOL_DN', true);
  pk.run(700);
  chk('the cylinder reaches the part (AS_DN)', pk.io.AS_DN === true);
  chk('nothing is held before the vacuum is on', !pk.parts.get('p1').held && pk.io.VAC_SW === false);
  pk.force('VAC_ON', true);
  pk.run(40);
  chk('the cup takes the part at once; the switch lags by buildMs', !!pk.parts.get('p1').held && pk.io.VAC_SW === false);
  pk.run(80);
  chk('the vacuum switch comes on after buildMs', pk.io.VAC_SW === true);
  pk.force('SOL_DN', false); pk.force('SOL_UP', true);
  pk.run(700);
  const zUp = pk.parts.get('p1').body.translation().z / SK;
  chk('the part rides the rod up (its top stays on the cup face at 949)', Math.abs(zUp - 919) < 2, zUp.toFixed(2) + ' mm');
  pk.force('VAC_ON', false);
  pk.run(700);
  const zDown = pk.parts.get('p1').body.translation().z / SK;
  chk('releasing drops it back onto the frame', Math.abs(zDown - 800) < 1 && pk.io.VAC_SW === false && pk.events.some(e => e.k === 'part' && e.ev === 'release' && e.by === 'cup1'), zDown.toFixed(2) + ' mm');
  await pk.close();
}

// ---------------------------------------------------------------- the hand: holding a part still (jam testing)
// Forcing lies to the PLC; the hand stops a PART. A jam is what the sequence actually has to
// survive, so it is worth a test of its own.
{
  const jam = () => ({
    format: 'mio-scene/1', name: 'jamtest',
    components: [
      { id: 'cv1', type: 'conveyor', params: { length: 1500, speed: 300 }, io: { run: 'CV1_RUN' } },
      { id: 'wp', type: 'workpiece', at: [0, 1000, 0], params: { material: 'alu' } },
      { id: 'em1', type: 'emitter', parent: 'cv1', socket: 'start', at: [100, 0, 60], params: { template: 'wp', intervalMs: 1500, max: 3 }, io: { count: 'EM1_CNT' } },
      { id: 'rm1', type: 'remover', parent: 'cv1', socket: 'end', at: [-75, 0, 0], io: { count: 'RM1_CNT' } },
    ],
  });
  // Hold the first part at 2.5 s, let the belt run under it for 4 s, then let go.
  const script = async () => {
    const p = await createPlant(jam(), {});
    p.force('CV1_RUN', true);
    p.run(2500);
    const uid = [...p.parts.keys()][0];
    const x0 = p.parts.get(uid).body.translation().x / SK;
    p.holdPart(uid, true);
    p.run(4000);
    const x1 = p.parts.get(uid).body.translation().x / SK;
    const heldRemoves = p.events.filter(e => e.k === 'part' && e.ev === 'remove').length;
    const behind = p.parts.size;
    p.holdPart(uid, false);
    p.run(8000);
    return { p, uid, x0, x1, heldRemoves, behind };
  };
  const j = await script();
  chk('the hand holds a part still while the belt runs under it', Math.abs(j.x1 - j.x0) < 0.5, (j.x1 - j.x0).toFixed(3) + ' mm in 4 s at 300 mm/s');
  chk('the jam stops the line: nothing reaches the remover while the part is held', j.heldRemoves === 0 && j.p.io.CV1_RUN === true);
  chk('the parts behind it queue up instead of passing through', j.behind >= 2, j.behind + ' parts on the belt');
  chk('releasing it lets the belt carry every part away again', j.p.parts.size === 0 && j.p.io.RM1_CNT === 3, j.p.io.RM1_CNT + ' removed');
  chk('the hand is recorded as a holder, so a run can be read back', j.p.events.some(e => e.k === 'part' && e.ev === 'hold' && e.by === 'hand')
    && j.p.events.some(e => e.k === 'part' && e.ev === 'release' && e.by === 'hand'));
  const j2 = await script();
  chk('a run with the hand in it still replays identically', JSON.stringify(j2.p.events) === JSON.stringify(j.p.events), j.p.events.length + ' events');
  await j.p.close(); await j2.p.close();

  // Dragging is the same hold, put somewhere else.
  const d = await createPlant(jam(), {});
  d.force('CV1_RUN', true);
  d.run(2500);
  const duid = [...d.parts.keys()][0], dp = () => d.parts.get(duid).body.translation();
  const from = dp();
  d.holdPart(duid, true);
  d.run(50);
  d.holdPart(duid, true, [from.x / SK - 300, from.y / SK, from.z / SK + 120]);
  d.run(100);
  const mid = dp();
  // It walks there at a hand's speed rather than teleporting: a kinematic body dropped into a
  // queue of resting parts scatters them across the hall.
  chk('the hand walks a part toward the pointer instead of teleporting it',
    Math.abs((mid.x - from.x) / SK) > 5 && Math.abs((mid.x - from.x) / SK) < 295, ((mid.x - from.x) / SK).toFixed(1) + ' mm after 100 ms');
  d.run(600);
  const to = dp();
  chk('dragging a held part moves it there and it stays', Math.abs((to.x - from.x) / SK + 300) < 0.5 && Math.abs((to.z - from.z) / SK - 120) < 0.5,
    ((to.x - from.x) / SK).toFixed(1) + ', ' + ((to.z - from.z) / SK).toFixed(1) + ' mm');
  chk('the plant reports which parts the hand holds (the viewer highlights them)', d.snapshot().pins.join() === duid, d.snapshot().pins.join());
  d.holdPart(duid, false);
  d.run(1500);
  chk('a dragged part falls and carries on when it is let go', d.parts.get(duid) == null || d.parts.get(duid).body.translation().z < to.z - 0.05);
  await d.close();

  // Taking a part OUT of a machine holder is the point: the holder's own switch must go false, so
  // the PLC can raise the alarm instead of running the cycle with an empty gripper.
  const pp = JSON.parse(fs.readFileSync(path.join(ROOT, 'scenes', 'pick-place.json'), 'utf8'));
  const { create: createPP } = await import('../scenes/pick-place.ctl.js');
  const w = await createPlant(pp, { controller: createPP() });
  w.run(200); powerUp(w);
  let holdUid = null, holder = null;
  for (let i = 0; i < 200 && !holdUid; i++) {
    w.run(100);
    for (const [uid, pt] of w.parts) if (pt.held) { holdUid = uid; holder = pt.held.id; }
  }
  w.holdPart(holdUid, true);
  w.run(100);
  chk('the hand takes a part out of a machine holder (gripper, cup or nest)',
    holdUid !== null && w.parts.get(holdUid)?.pin != null && w.parts.get(holdUid)?.held == null, holdUid + ' from ' + holder);
  chk('the holder knows it lost the part, so the PLC can see it', w.parts.get(holdUid) != null && ![...w.parts.values()].some(pt => pt.held?.id === holder),
    holder);
  await w.close();
}

// ---------------------------------------------------------------- a pallet stop is structure, not clearance
// An overhead pin that a pallet must drive UNDER holds it through a positive-distance contact even
// after it has lifted clear and come to rest: measured from 7.9 and 15.1 mm, and the clearance
// sweep is non-monotonic (5.1 held, 8.1 free, 11.1 free, 14.1 HELD, 17.1+ free), so no clearance
// can be designed to. A stop that pops UP from under the belt leaves the path entirely.
{
  const line = kind => ({
    format: 'mio-scene/1', name: 'palletstop',
    components: [
      { id: 'cv1', type: 'conveyor', params: { length: 2000, width: 300, speed: 250 }, io: { run: 'CV1_RUN' } },
      { id: 'plt', type: 'pallet', parent: 'cv1', socket: 'top', at: [-600, 0, 30], params: { dynamic: true } },
      ...(kind === 'overhead' ? [
        { id: 'post1', type: 'plate', parent: 'cv1', socket: 'top', at: [330, 250, -800], params: { size: [40, 40, 953] } },
        { id: 'br1', type: 'plate', parent: 'cv1', socket: 'top', at: [300, 95, 153], params: { size: [60, 330, 12] } },
        { id: 'stp', type: 'cylinder', parent: 'br1', socket: 'bottom', at: [0, 95, 0],
          params: { bore: 16, stroke: 30, valve: '5/2-single', extendMs: 120, retractMs: 120,
                    extWord: 'DOWN', retWord: 'UP', head: 'plate', headSize: [12, 12, 25] }, io: { solExt: 'SOL_STOP' } },
      ] : [
        // foot = Lb(96) + rod end 27 + head 25 + 15 mm of sink, so the retracted head is below the belt
        { id: 'stp', type: 'cylinder', parent: 'cv1', socket: 'top', at: [300, 0, -163],
          params: { bore: 16, stroke: 60, valve: '5/2-single', extendMs: 150, retractMs: 150,
                    extWord: 'UP', retWord: 'DOWN', head: 'plate', headSize: [12, 120, 25] }, io: { solExt: 'SOL_STOP' } },
      ]),
    ],
  });
  const run = async kind => {
    const q = await createPlant(line(kind), {});
    const at = () => (q.parts.get('plt') ? q.parts.get('plt').body.translation().x / SK : null);
    q.force('SOL_STOP', true);
    q.run(500);                                                // the pin is UP before the pallet arrives
    q.force('CV1_RUN', true);
    q.run(8000);
    const blocked = at();
    q.force('SOL_STOP', false);
    q.run(500);
    const from = at();
    q.run(5000);
    const to = at();
    await q.close();
    return { blocked, moved: to === null ? 9999 : to - from };  // null: it left the belt, which is moving
  };
  const over = await run('overhead'), pop = await run('popup');
  chk('both stops actually block the pallet', over.blocked > 100 && over.blocked < 400 && pop.blocked > 100 && pop.blocked < 400,
    'overhead ' + over.blocked?.toFixed(0) + ', pop-up ' + pop.blocked?.toFixed(0));
  chk('trap: a RETRACTED overhead stop still holds the pallet it blocked', over.moved < 250, over.moved.toFixed(0) + ' mm in 5 s');
  chk('rule: a stop that pops up from under the belt lets it go', pop.moved > 400, pop.moved.toFixed(0) + ' mm in 5 s');
}

// ---------------------------------------------------------------- scene a-to-b with its internal controller
{
  const ab = JSON.parse(fs.readFileSync(path.join(ROOT, 'scenes', 'a-to-b.json'), 'utf8'));
  const { create: createAB } = await import('../scenes/a-to-b.ctl.js');
  const run = async () => {
    const q = await createPlant(ab, { controller: createAB() });
    q.run(200); powerUp(q);
    q.run(40000);
    return q;
  };
  const q = await run();
  const pev = ev => q.events.filter(e => e.k === 'part' && e.ev === ev).length;
  chk('a-to-b: the sequence completes cycles', q.io.CYCLE_CNT >= 5, 'CYCLE_CNT ' + q.io.CYCLE_CNT);
  chk('a-to-b: every loaded part is unloaded (in = out + inside), none lost', pev('spawn') === pev('remove') + q.parts.size && pev('lost') === 0 && q.parts.size <= 1,
    pev('spawn') + ' in, ' + pev('remove') + ' out, ' + q.parts.size + ' inside');
  chk('a-to-b: PLC-visible counters agree with the plant', q.io.EM1_CNT === pev('spawn') && q.io.RM1_CNT === pev('remove'));
  chk('a-to-b: the end sensor sees one part per cycle', Math.abs(edges(q, 'PE_END', true).length - q.io.CYCLE_CNT) <= 1);
  chk('a-to-b: no warnings', !q.events.some(e => e.k === 'warn'), q.events.filter(e => e.k === 'warn').map(e => e.msg).join(' | '));
  const q2 = await run();
  chk('a-to-b: two runs give identical event logs', JSON.stringify(q2.events) === JSON.stringify(q.events), q.events.length + ' events');
  // Reset zeroes the plant's counters; a controller holding the old ones waits for a part that
  // already "arrived" and the sequence stalls with the clock still running (measured).
  q.reset();
  powerUp(q);
  q.run(25000);
  chk('a-to-b: Reset then START runs again (the controller is reset too)', q.io.CYCLE_CNT >= 2, 'CYCLE_CNT ' + q.io.CYCLE_CNT + ', step ' + q.io.ST1_STEP);
  await q.close(); await q2.close();
}

// ---------------------------------------------------------------- scene buffer-queue (stop and go)
// Two stopper pins 300 mm apart: HOLD meters one part into the pocket, GATE releases it on demand.
// Each pin has its own up/down reed switches and its own photo-eye.
{
  const bq = JSON.parse(fs.readFileSync(path.join(ROOT, 'scenes', 'buffer-queue.json'), 'utf8'));
  const { create: createBQ } = await import('../scenes/buffer-queue.ctl.js');
  const run = async () => {
    const q = await createPlant(bq, { controller: createBQ() });
    q.run(200); powerUp(q);
    q.run(60000);
    return q;
  };
  const q = await run();
  const pev = ev => q.events.filter(e => e.k === 'part' && e.ev === ev).length;
  chk('buffer-queue: one part is metered through the escapement per cycle', q.io.CYCLE_CNT >= 4 && Math.abs(q.io.RM_CNT - q.io.CYCLE_CNT) <= 1,
    'CYCLE_CNT ' + q.io.CYCLE_CNT + ', removed ' + q.io.RM_CNT);
  // The discriminating measurement: both eyes see every part, each exactly once per cycle.
  const hold = edges(q, 'PE_HOLD', true).length, gate = edges(q, 'PE_GATE', true).length;
  chk('buffer-queue: each part is seen once at the hold pin and once at the gate', Math.abs(hold - q.io.EM_CNT) <= 1 && Math.abs(gate - q.io.CYCLE_CNT) <= 1,
    hold + ' hold, ' + gate + ' gate, ' + q.io.EM_CNT + ' fed, ' + q.io.CYCLE_CNT + ' cycles');
  // A pin cannot come down between parts that touch, so the line must never hold two at once.
  chk('buffer-queue: only one part is in the line at a time, so a pin always lands on free belt', q.parts.size <= 1,
    q.parts.size + ' on the belt');
  chk('buffer-queue: both pins really stroke (the reed switches see up and down)',
    edges(q, 'AS_HOLD_DN', true).length >= 4 && edges(q, 'AS_HOLD_UP', true).length >= 4
    && edges(q, 'AS_GATE_DN', true).length >= 4 && edges(q, 'AS_GATE_UP', true).length >= 4,
    'hold ' + edges(q, 'AS_HOLD_DN', true).length + '/' + edges(q, 'AS_HOLD_UP', true).length
    + ', gate ' + edges(q, 'AS_GATE_DN', true).length + '/' + edges(q, 'AS_GATE_UP', true).length);
  chk('buffer-queue: parts balance and none are lost', pev('spawn') === pev('remove') + q.parts.size && pev('lost') === 0,
    pev('spawn') + ' in, ' + pev('remove') + ' out, ' + q.parts.size + ' inside, ' + pev('lost') + ' lost');
  chk('buffer-queue: no warnings', !q.events.some(e => e.k === 'warn'), q.events.filter(e => e.k === 'warn').map(e => e.msg).slice(0, 3).join(' | '));
  const q2 = await run();
  chk('buffer-queue: two runs give identical event logs', JSON.stringify(q2.events) === JSON.stringify(q.events), q.events.length + ' events');
  // A seized machine still balances its parts and raises no warning, so ask for PROGRESS.
  const before = q.io.CYCLE_CNT;
  q.run(90000);
  chk('buffer-queue: it keeps cycling, it does not seize', q.io.CYCLE_CNT - before >= 5,
    (q.io.CYCLE_CNT - before) + ' more cycles in 90 s');
  await q.close(); await q2.close();
}

// ---------------------------------------------------------------- scene sort-by-height (two beams)
{
  const sbh = JSON.parse(fs.readFileSync(path.join(ROOT, 'scenes', 'sort-by-height.json'), 'utf8'));
  const { create: createSBH } = await import('../scenes/sort-by-height.ctl.js');
  const run = async () => {
    const q = await createPlant(sbh, { controller: createSBH() });
    q.run(200); powerUp(q);
    q.run(60000);
    return q;
  };
  const q = await run();
  const pev = ev => q.events.filter(e => e.k === 'part' && e.ev === ev).length;
  chk('sort-by-height: the cycle repeats, feeding tall and short by turns', q.io.CYCLE_CNT >= 6 && Math.abs(q.io.EM_T_CNT - q.io.EM_S_CNT) <= 1,
    'CYCLE_CNT ' + q.io.CYCLE_CNT + ', tall ' + q.io.EM_T_CNT + ' short ' + q.io.EM_S_CNT);
  // The discriminating measurement: the high beam breaks once per TALL part and never for a short one.
  chk('sort-by-height: only the tall parts reach the high beam', Math.abs(edges(q, 'PE_HIGH', true).length - q.io.EM_T_CNT) <= 1,
    edges(q, 'PE_HIGH', true).length + ' beam breaks for ' + q.io.EM_T_CNT + ' tall parts');
  chk('sort-by-height: every tall part ends in the reject bin', q.io.RM_T_CNT === q.io.EM_T_CNT, q.io.RM_T_CNT + ' of ' + q.io.EM_T_CNT);
  chk('sort-by-height: every short part rides on to the outfeed', q.io.EM_S_CNT - q.io.RM_S_CNT <= 1, q.io.RM_S_CNT + ' of ' + q.io.EM_S_CNT);
  chk('sort-by-height: parts balance and none are lost', pev('spawn') === pev('remove') + q.parts.size && pev('lost') === 0,
    pev('spawn') + ' in, ' + pev('remove') + ' out, ' + q.parts.size + ' inside, ' + pev('lost') + ' lost');
  chk('sort-by-height: no warnings', !q.events.some(e => e.k === 'warn'), q.events.filter(e => e.k === 'warn').map(e => e.msg).slice(0, 3).join(' | '));
  const q2 = await run();
  chk('sort-by-height: two runs give identical event logs', JSON.stringify(q2.events) === JSON.stringify(q.events), q.events.length + ' events');
  await q.close(); await q2.close();
}

// ---------------------------------------------------------------- scene assembler (index table)
{
  const asm = JSON.parse(fs.readFileSync(path.join(ROOT, 'scenes', 'assembler.json'), 'utf8'));
  const { create: createASM } = await import('../scenes/assembler.ctl.js');
  const run = async () => {
    const q = await createPlant(asm, { controller: createASM() });
    q.run(200); powerUp(q);
    q.run(60000);
    return q;
  };
  const q = await run();
  const pev = ev => q.events.filter(e => e.k === 'part' && e.ev === ev).length;
  chk('assembler: the table indexes a station per cycle', q.io.CYCLE_CNT >= 12, 'CYCLE_CNT ' + q.io.CYCLE_CNT + ', station ' + q.io.TBL_STATION);
  chk('assembler: nests seat every base squarely on the pocket floor (720 mm)',
    [...q.parts.values()].filter(p => p.tpl === 'wpBase' && p.held).every(p => Math.abs(p.body.translation().z / SK - 720) < 0.1),
    [...q.parts.values()].filter(p => p.tpl === 'wpBase').map(p => (p.body.translation().z / SK).toFixed(1)).join(' '));
  // The lid is only held by friction: a faster index throws it off the table (measured).
  const parts = [...q.parts.values()].map(p => { const t = p.body.translation(); return { tpl: p.tpl, x: t.x / SK, y: t.y / SK, z: t.z / SK }; });
  const lids = parts.filter(p => p.tpl === 'wpLid'), bases = parts.filter(p => p.tpl === 'wpBase');
  const riding = lids.filter(l => bases.some(b => Math.hypot(l.x - b.x, l.y - b.y) < 25 && Math.abs(l.z - (b.z + 20)) < 3));
  chk('assembler: every lid rides its base round the table', lids.length >= 1 && riding.length === lids.length, riding.length + '/' + lids.length);
  chk('assembler: parts balance and none are lost off the table', pev('spawn') === pev('remove') + q.parts.size && pev('lost') === 0,
    pev('spawn') + ' in, ' + pev('remove') + ' out, ' + q.parts.size + ' inside, ' + pev('lost') + ' lost');
  chk('assembler: no warnings', !q.events.some(e => e.k === 'warn'), q.events.filter(e => e.k === 'warn').map(e => e.msg).slice(0, 3).join(' | '));
  const q2 = await run();
  chk('assembler: two runs give identical event logs', JSON.stringify(q2.events) === JSON.stringify(q.events), q.events.length + ' events');
  await q.close(); await q2.close();
}

// ---------------------------------------------------------------- the 2-finger gripper
{
  // The gripper hangs from a lift cylinder: Lb = 165, so its frame is at 1157 - 192 - x, and at
  // full stroke (120) the grip zone centre (0.6 * 50 below the frame) sits at the part's centre.
  const grab = {
    format: 'mio-scene/1', name: 'griptest',
    components: [
      { id: 'base', type: 'frame', params: { size: [1200, 600, 800] } },
      { id: 'p1', type: 'workpiece', parent: 'base', socket: 'top', at: [0, 0, 0], params: { dynamic: true, size: [60, 40, 30] } },
      { id: 'lift', type: 'cylinder', parent: 'base', socket: 'top', at: [0, 0, 357], rot: [180, 0, 0],
        params: { bore: 25, stroke: 120, valve: '5/2-double' }, io: { solExt: 'SOL_DN', solRet: 'SOL_UP', 'sw.ext': 'AS_DN', 'sw.ret': 'AS_UP' } },
      { id: 'grip1', type: 'gripper', parent: 'lift', socket: 'rodEnd', at: [0, 0, 0],
        params: { span: 60, fingerLen: 50, fingerW: 10 }, io: { close: 'GRIP_CLOSE', open: 'GRIP_OPEN', closed: 'GRIP_CLOSED' } },
      // a second gripper over nothing: closing on air must reach the `closed` switch
      { id: 'grip2', type: 'gripper', parent: 'base', socket: 'top', at: [400, 0, 200], rot: [180, 0, 0],
        params: { span: 60, fingerLen: 50, fingerW: 10 }, io: { close: 'GRIP2_CLOSE', closed: 'GRIP2_CLOSED' } },
    ],
  };
  const g = await createPlant(grab, {});
  g.run(300);
  chk('the gripper starts open (the open switch, not the closed one)', g.io.GRIP_OPEN === true && g.io.GRIP_CLOSED === false && Math.abs(g.dof.grip1 - 30) < 1e-9);
  g.force('SOL_DN', true);
  g.run(700);
  chk('the lift puts the part between the fingers', g.io.AS_DN === true);
  g.force('GRIP_CLOSE', true);
  g.run(500);
  chk('the fingers stop at the part\'s half width (20 mm for a 40 mm part)', Math.abs(g.dof.grip1 - 20) < 0.2, g.dof.grip1.toFixed(3) + ' mm');
  chk('a gripped part never trips the closed switch: a missed grip stays visible', g.io.GRIP_CLOSED === false && g.io.GRIP_OPEN === false);
  chk('the part is held by the gripper', g.parts.get('p1').held?.id === 'grip1');
  g.force('SOL_DN', false); g.force('SOL_UP', true);
  g.run(700);
  const zUp = g.parts.get('p1').body.translation().z / SK;
  chk('the part rides up with the fingers', zUp > 900, zUp.toFixed(1) + ' mm');
  g.force('GRIP_CLOSE', false);
  g.run(800);
  chk('opening drops the part back on the frame', Math.abs(g.parts.get('p1').body.translation().z / SK - 800) < 1 && !g.parts.get('p1').held,
    (g.parts.get('p1').body.translation().z / SK).toFixed(2) + ' mm');
  g.force('GRIP2_CLOSE', true);
  g.run(500);
  chk('closing on nothing reaches the closed switch (the missed-grip signal)', g.io.GRIP2_CLOSED === true && g.dof.grip2 === 0);
  await g.close();
}

// ---------------------------------------------------------------- scene pick-place with its internal controller
{
  const pp = JSON.parse(fs.readFileSync(path.join(ROOT, 'scenes', 'pick-place.json'), 'utf8'));
  const { create: createPP } = await import('../scenes/pick-place.ctl.js');
  const run = async () => {
    const q = await createPlant(pp, { controller: createPP() });
    q.run(200); powerUp(q);
    q.run(60000);
    return q;
  };
  const q = await run();
  const pev = ev => q.events.filter(e => e.k === 'part' && e.ev === ev).length;
  chk('pick-place: the cycle repeats', q.io.CYCLE_CNT >= 3, 'CYCLE_CNT ' + q.io.CYCLE_CNT + ', step ' + q.io.ST1_STEP);
  chk('pick-place: the nest hands each part to the cup (one holder at a time)', pev('hold') === 2 * pev('release') || pev('hold') >= 2 * q.io.CYCLE_CNT,
    pev('hold') + ' holds, ' + pev('release') + ' releases');
  chk('pick-place: every part placed on the belt reaches the unloader', pev('spawn') === pev('remove') + q.parts.size && pev('lost') === 0,
    pev('spawn') + ' in, ' + pev('remove') + ' out, ' + q.parts.size + ' inside, ' + pev('lost') + ' lost');
  chk('pick-place: no warnings (servo replies are not stretched)', !q.events.some(e => e.k === 'warn'), q.events.filter(e => e.k === 'warn').map(e => e.msg).slice(0, 3).join(' | '));
  const q2 = await run();
  chk('pick-place: two runs give identical event logs', JSON.stringify(q2.events) === JSON.stringify(q.events), q.events.length + ' events');
  await q.close(); await q2.close();
}

// ---------------------------------------------------------------- scene stopper-pusher with its internal controller
{
  const sp = JSON.parse(fs.readFileSync(path.join(ROOT, 'scenes', 'stopper-pusher.json'), 'utf8'));
  const { create: createSP } = await import('../scenes/stopper-pusher.ctl.js');
  const run = async () => {
    const q = await createPlant(sp, { controller: createSP() });
    q.run(200); powerUp(q);
    q.run(60000);
    return q;
  };
  const q = await run();
  const pev = ev => q.events.filter(e => e.k === 'part' && e.ev === ev).length;
  chk('stopper-pusher: one cycle per part fed (2.5 s)', q.io.CYCLE_CNT >= 20, 'CYCLE_CNT ' + q.io.CYCLE_CNT);
  chk('stopper-pusher: odd parts slide down the chute into the bin, even parts reach the outfeed', q.io.RM_NG_CNT >= 10 && Math.abs(q.io.RM_NG_CNT - q.io.RM_OK_CNT) <= 1,
    'bin ' + q.io.RM_NG_CNT + ', outfeed ' + q.io.RM_OK_CNT);
  chk('stopper-pusher: in = out + inside, none lost', pev('spawn') === pev('remove') + q.parts.size && pev('lost') === 0,
    pev('spawn') + ' in, ' + pev('remove') + ' out, ' + q.parts.size + ' inside, ' + pev('lost') + ' lost');
  chk('stopper-pusher: no warnings', !q.events.some(e => e.k === 'warn'), q.events.filter(e => e.k === 'warn').map(e => e.msg).slice(0, 3).join(' | '));
  const q2 = await run();
  chk('stopper-pusher: two runs give identical event logs', JSON.stringify(q2.events) === JSON.stringify(q.events), q.events.length + ' events');
  await q.close(); await q2.close();
}

// ---------------------------------------------------------------- scene sort-by-material (inductive sensor, latched upstream)
{
  const sbm = JSON.parse(fs.readFileSync(path.join(ROOT, 'scenes', 'sort-by-material.json'), 'utf8'));
  const { create: createSBM } = await import('../scenes/sort-by-material.ctl.js');
  const run = async () => {
    const q = await createPlant(sbm, { controller: createSBM() });
    q.run(200); powerUp(q);
    q.run(60000);
    return q;
  };
  const q = await run();
  const pev = ev => q.events.filter(e => e.k === 'part' && e.ev === ev).length;
  chk('sort-by-material: the cycle repeats, feeding steel and plastic by turns', q.io.CYCLE_CNT >= 6 && Math.abs(q.io.EM_M_CNT - q.io.EM_P_CNT) <= 1,
    'CYCLE_CNT ' + q.io.CYCLE_CNT + ', steel ' + q.io.EM_M_CNT + ' plastic ' + q.io.EM_P_CNT);
  // The discriminating measurement: the inductive sensor pulses for steel only. Every steel part in
  // the bin passed it, and there are never more pulses than steel parts (plastic is invisible to it).
  const px = edges(q, 'PX_METAL', true).length;
  chk('sort-by-material: the inductive sensor sees every steel part and never a plastic one', px >= q.io.RM_M_CNT && px <= q.io.EM_M_CNT,
    px + ' pulses, ' + q.io.EM_M_CNT + ' steel fed, ' + q.io.RM_M_CNT + ' in the bin');
  chk('sort-by-material: steel ends in the bin, plastic on the outfeed', q.io.EM_M_CNT - q.io.RM_M_CNT <= 1 && q.io.EM_P_CNT - q.io.RM_P_CNT <= 1,
    'steel ' + q.io.RM_M_CNT + '/' + q.io.EM_M_CNT + ', plastic ' + q.io.RM_P_CNT + '/' + q.io.EM_P_CNT);
  chk('sort-by-material: parts balance and none are lost', pev('spawn') === pev('remove') + q.parts.size && pev('lost') === 0,
    pev('spawn') + ' in, ' + pev('remove') + ' out, ' + q.parts.size + ' inside, ' + pev('lost') + ' lost');
  chk('sort-by-material: no warnings', !q.events.some(e => e.k === 'warn'), q.events.filter(e => e.k === 'warn').map(e => e.msg).slice(0, 3).join(' | '));
  const q2 = await run();
  chk('sort-by-material: two runs give identical event logs', JSON.stringify(q2.events) === JSON.stringify(q.events), q.events.length + ' events');
  await q.close(); await q2.close();
}

// ---------------------------------------------------------------- scene gripper-transfer (pneumatic XZ with a 2-finger gripper)
{
  const gt = JSON.parse(fs.readFileSync(path.join(ROOT, 'scenes', 'gripper-transfer.json'), 'utf8'));
  const { create: createGT } = await import('../scenes/gripper-transfer.ctl.js');
  const run = async () => {
    const q = await createPlant(gt, { controller: createGT() });
    q.run(200); powerUp(q);
    q.run(60000);
    return q;
  };
  const q = await run();
  const pev = ev => q.events.filter(e => e.k === 'part' && e.ev === ev).length;
  chk('gripper-transfer: the cycle repeats', q.io.CYCLE_CNT >= 4, 'CYCLE_CNT ' + q.io.CYCLE_CNT + ', step ' + q.io.ST1_STEP);
  // A gripper has no "gripped" switch: CLOSED means it closed on nothing. It never did.
  chk('gripper-transfer: the fingers stop on the part every time (the closed switch never rises)', edges(q, 'GRIP_CLOSED', true).length === 0 && q.io.ST1_STEP !== 900,
    edges(q, 'GRIP_CLOSED', true).length + ' closed edges, step ' + q.io.ST1_STEP);
  chk('gripper-transfer: every pick is one hold and one release by the gripper', pev('hold') >= q.io.CYCLE_CNT && pev('hold') - pev('release') <= 1,
    pev('hold') + ' holds, ' + pev('release') + ' releases');
  chk('gripper-transfer: every part set down on the outfeed belt reaches the unloader', pev('spawn') === pev('remove') + q.parts.size && pev('lost') === 0,
    pev('spawn') + ' in, ' + pev('remove') + ' out, ' + q.parts.size + ' inside, ' + pev('lost') + ' lost');
  chk('gripper-transfer: no warnings', !q.events.some(e => e.k === 'warn'), q.events.filter(e => e.k === 'warn').map(e => e.msg).slice(0, 3).join(' | '));
  const q2 = await run();
  chk('gripper-transfer: two runs give identical event logs', JSON.stringify(q2.events) === JSON.stringify(q.events), q.events.length + ' events');
  await q.close(); await q2.close();
}

// ---------------------------------------------------------------- scene press-station (nest, clamp, press, ejector)
{
  const ps = JSON.parse(fs.readFileSync(path.join(ROOT, 'scenes', 'press-station.json'), 'utf8'));
  const { create: createPS } = await import('../scenes/press-station.ctl.js');
  // The rule (CLAUDE.md): a kinematic tool never closes ONTO a held part. The press stroke ends
  // 3 mm above the part's top, from the scene numbers alone.
  const pr = ps.components.find(c => c.id === 'press'), wp = ps.components.find(c => c.id === 'wp');
  const face = worldPoses(ps, { press: pr.params.stroke }).press.rod.p[2] - 27 - pr.params.headSize[2];
  const top = 800 + wp.params.size[2];
  chk('press-station: the press stops 3 mm above the part, never on it', Math.abs(face - top - 3) < 1e-6, 'face ' + face + ', part top ' + top);
  const run = async () => {
    const q = await createPlant(ps, { controller: createPS() });
    q.run(200); powerUp(q);
    q.run(60000);
    return q;
  };
  const q = await run();
  const pev = ev => q.events.filter(e => e.k === 'part' && e.ev === ev).length;
  chk('press-station: the cycle repeats (feed, clamp, press, eject)', q.io.CYCLE_CNT >= 15, 'CYCLE_CNT ' + q.io.CYCLE_CNT + ', step ' + q.io.ST1_STEP);
  chk('press-station: the clamped nest seats every part square on the pocket floor (800 mm)',
    [...q.parts.values()].filter(p => p.held).every(p => Math.abs(p.body.translation().z / SK - 800) < 0.1),
    [...q.parts.values()].map(p => (p.body.translation().z / SK).toFixed(2) + (p.held ? ' held' : '')).join(' '));
  chk('press-station: every ejected part slides down the chute into the bin', pev('spawn') === pev('remove') + q.parts.size && pev('lost') === 0 && q.io.EM_CNT - q.io.RM_CNT <= 1,
    pev('spawn') + ' in, ' + pev('remove') + ' out, ' + q.parts.size + ' inside, ' + pev('lost') + ' lost');
  chk('press-station: no warnings', !q.events.some(e => e.k === 'warn'), q.events.filter(e => e.k === 'warn').map(e => e.msg).slice(0, 3).join(' | '));
  const q2 = await run();
  chk('press-station: two runs give identical event logs', JSON.stringify(q2.events) === JSON.stringify(q.events), q.events.length + ' events');
  await q.close(); await q2.close();
}

// ---------------------------------------------------------------- the operator panel
// The cell panel (rb4axis): a selector AUTO / INDIVIDUAL, MASTER ON, a latching E-STOP mushroom,
// START, CYCLE STOP, HOME POS, and one button per actuator. The start-up order is the machine's:
// energise, home, start. The browser only sends button edges; the PLC program enforces all of it.
{
  const ab = JSON.parse(fs.readFileSync(path.join(ROOT, 'scenes', 'a-to-b.json'), 'utf8'));
  const { create: createAB } = await import('../scenes/a-to-b.ctl.js');
  const q = await createPlant(ab, { controller: createAB() });
  const tap = (id, key = 'pb') => { q.press(id, key, true); q.run(60); q.press(id, key, false); q.run(60); };
  const pev = ev => q.events.filter(e => e.k === 'part' && e.ev === ev).length;
  q.run(200);

  // -------------------------------------------------- start-up order
  tap('pbStart'); q.run(300);
  chk('panel: a machine that is not energised does not start', q.io.ST1_STEP === 0 && q.io.PL_MASTER === false, 'step ' + q.io.ST1_STEP);
  tap('pbMaster');
  chk('panel: MASTER ON energises the machine and lights its lamp', q.io.PL_MASTER === true);
  tap('pbStart'); q.run(300);
  chk('panel: energised but not homed, START is still refused', q.io.ST1_STEP === 0 && q.io.PL_HOME === false, 'step ' + q.io.ST1_STEP);
  tap('pbHome'); q.run(600);
  chk('panel: HOME POS homes the machine and lights the home lamp', q.io.PL_HOME === true && q.io.ST1_STEP === 0, 'step ' + q.io.ST1_STEP);
  tap('pbStart'); q.run(400);
  chk('panel: START then runs the sequence', q.io.AUTO_RUN === true && q.io.ST1_STEP >= 10 && q.io.ST1_STEP < 900, 'step ' + q.io.ST1_STEP);

  // -------------------------------------------------- cycle stop finishes the cycle
  tap('pbCstop');
  for (let i = 0; i < 300 && q.io.AUTO_RUN; i++) q.run(100);
  chk('panel: CYCLE STOP lets the cycle finish, then leaves the machine idle and still homed',
    q.io.ST1_STEP === 0 && q.io.AUTO_RUN === false && q.io.CYCLE_CNT >= 1 && q.io.PL_HOME === true,
    'step ' + q.io.ST1_STEP + ', CYCLE_CNT ' + q.io.CYCLE_CNT);

  // -------------------------------------------------- E-STOP
  tap('pbStart'); q.run(600);
  const running = q.io.AUTO_RUN;
  tap('pbEstop');                                          // a latching mushroom: one press latches it
  q.run(100);
  chk('panel: E-STOP stops the machine at once, drops the master and de-energises the outputs',
    running && q.io.ST1_STEP === 910 && q.io.AUTO_RUN === false && q.io.PL_MASTER === false && q.io.CV1_RUN === false && q.io.EM1_EMIT === false,
    'step ' + q.io.ST1_STEP + ', belt ' + q.io.CV1_RUN);
  chk('panel: an E-STOP also loses the home position', q.io.PL_HOME === false);
  tap('pbMaster'); q.run(100);
  chk('panel: while the mushroom is latched, MASTER ON does nothing', q.io.ST1_STEP === 910 && q.io.PL_MASTER === false);
  tap('pbEstop'); q.run(100);                              // twist to release
  chk('panel: releasing the mushroom alone does not energise the machine', q.io.ST1_STEP === 910 && q.io.PL_MASTER === false);
  tap('pbMaster'); q.run(100);
  chk('panel: MASTER ON after the release brings the machine back to idle', q.io.ST1_STEP === 0 && q.io.PL_MASTER === true);
  tap('pbStart'); q.run(300);
  chk('panel: it refuses to start until it has been homed again', q.io.ST1_STEP === 0, 'step ' + q.io.ST1_STEP);
  tap('pbHome'); q.run(600);
  tap('pbStart'); q.run(400);
  chk('panel: homed again, it runs again', q.io.PL_HOME === true && q.io.AUTO_RUN === true, 'step ' + q.io.ST1_STEP);

  // -------------------------------------------------- selector and the individual buttons
  tap('sel', 'sel'); q.run(100);
  chk('panel: turning the selector while running is a FAULT with the outputs off',
    q.io.SEL_AUTO === false && q.io.ST1_STEP === 900 && q.io.AUTO_RUN === false && q.io.CV1_RUN === false,
    'step ' + q.io.ST1_STEP + ', belt ' + q.io.CV1_RUN);
  const before = pev('spawn');
  tap('pbIndCv'); q.run(200);
  chk('panel: on INDIVIDUAL the belt button drives the belt on its own', q.io.CV1_RUN === true && q.dof.cv1 > 0);
  tap('pbIndCv'); q.run(200);
  const off = q.io.CV1_RUN;
  tap('pbIndCv'); q.run(200);
  chk('panel: pressing it again toggles the belt off, and again on', off === false && q.io.CV1_RUN === true);
  tap('pbIndFeed'); q.run(300);
  chk('panel: the individual feed button loads exactly one part', pev('spawn') === before + 1, (pev('spawn') - before) + ' loaded');
  tap('pbEstop'); q.run(100);
  chk('panel: E-STOP kills the individual outputs too', q.io.CV1_RUN === false && q.io.ST1_STEP === 910);
  tap('pbEstop'); tap('pbMaster'); q.run(100);
  tap('pbIndCv'); q.run(200);
  chk('panel: after the reset the individual buttons work again', q.io.CV1_RUN === true);
  tap('sel', 'sel'); q.run(100);
  chk('panel: back on AUTO the toggle memory is cleared: the belt stops although its button was left on',
    q.io.SEL_AUTO === true && q.io.CV1_RUN === false, 'belt ' + q.io.CV1_RUN);
  tap('pbStart'); q.run(200);
  chk('panel: the earlier E-STOP is still remembered, so START is refused until HOME POS is pressed again',
    q.io.ST1_STEP === 0 && q.io.AUTO_RUN === false && q.io.PL_HOME === false, 'step ' + q.io.ST1_STEP + ', homed ' + q.io.PL_HOME);
  tap('pbHome'); q.run(600);
  tap('pbStart'); q.run(400);
  chk('panel: homed once more, it runs again', q.io.PL_HOME === true && q.io.AUTO_RUN === true && q.io.ST1_STEP >= 10, 'step ' + q.io.ST1_STEP);
  await q.close();
}

// ---------------------------------------------------------------- scene palletizing (100 plugs, 5-up gantry, jig carrier)
// The biggest scene: a 10 x 10 pallet of spark plugs loaded by its own pallet loader, a five-up
// vacuum gantry on two servo axes, a rotary carrier with five-slot jigs, and an unload head
// feeding the next process. An empty tray calls for a fresh pallet (pinned in tests/ctl.test.js,
// where twenty cycles can be driven in a moment).
{
  const pl = JSON.parse(fs.readFileSync(path.join(ROOT, 'scenes', 'palletizing.json'), 'utf8'));
  const { create: createPL } = await import('../scenes/palletizing.ctl.js');
  const run = async () => {
    const q = await createPlant(pl, { controller: createPL() });
    q.run(200); powerUp(q);
    q.run(70000);
    return q;
  };
  const q = await run();
  const pev = ev => q.events.filter(e => e.k === 'part' && e.ev === ev).length;
  chk('palletizing: the machine loads its own pallet before it picks anything, 100 plugs in 100 holes',
    pev('spawn') === 100 && q.io.EM_PAL_CNT === 100, pev('spawn') + ' plugs');
  chk('palletizing: the cycle repeats: pick five, set five in the jig, index, unload five', q.io.CYCLE_CNT >= 4 && q.io.ST1_STEP < 900,
    'CYCLE_CNT ' + q.io.CYCLE_CNT + ', step ' + q.io.ST1_STEP);
  // The discriminating measurement: the next process receives whole groups of five, never a gap.
  chk('palletizing: the next process receives whole groups of five', q.io.RM_CNT > 0 && q.io.RM_CNT % 5 === 0, q.io.RM_CNT + ' plugs delivered');
  chk('palletizing: every plug delivered came out of the pallet, and none was dropped',
    pev('spawn') === pev('remove') + q.parts.size && pev('lost') === 0,
    pev('spawn') + ' in, ' + pev('remove') + ' out, ' + q.parts.size + ' inside, ' + pev('lost') + ' lost');
  // Unclamping the whole tray to let five cups take five: the other 95 must stay standing.
  const inPallet = [...q.parts.values()].filter(p => p.held?.id?.startsWith('pal'));
  chk('palletizing: the plugs left in the pallet stand square in their pockets', inPallet.length >= 20
    && inPallet.every(p => Math.abs(p.body.translation().z / SK - 732) < 0.1),
    inPallet.length + ' in the pallet');
  chk('palletizing: no warnings', !q.events.some(e => e.k === 'warn'), q.events.filter(e => e.k === 'warn').map(e => e.msg).slice(0, 3).join(' | '));
  // The scene is the heaviest in the repo, so it is also the one that pins the step budget.
  chk('palletizing: a step costs well under its 4 ms budget', q.stepUs === 0 || q.stepUs < 2500, q.stepUs + ' us');
  const q2 = await run();
  chk('palletizing: two runs give identical event logs', JSON.stringify(q2.events) === JSON.stringify(q.events), q.events.length + ' events');
  await q.close(); await q2.close();
}

// ---------------------------------------------------------------- E-STOP in the middle of a pick
// The trays must KEEP their parts through an E-STOP. Measured live: the mushroom was hit while the
// pallet was unclamped for a pick, a hundred plugs were left loose in their pockets, and the plant
// went from 355 to over 2800 us a step - the machine came back stalling at 2x world speed.
{
  const pl = JSON.parse(fs.readFileSync(path.join(ROOT, 'scenes', 'palletizing.json'), 'utf8'));
  const { create: createPL } = await import('../scenes/palletizing.ctl.js');
  const q = await createPlant(pl, { controller: createPL() });
  const tap = (/** @type {string} */ id) => { q.press(id, 'pb', true); q.run(120); q.press(id, 'pb', false); q.run(120); };
  q.run(200); powerUp(q, 6000);
  for (let i = 0; i < 300 && q.io.PAL_CLAMP !== false; i++) q.run(100);      // wait for a pick
  chk('E-STOP: the pallet really does unclamp to let the cups take five', q.io.PAL_CLAMP === false, 'step ' + q.io.ST1_STEP);
  tap('pbEstop');
  q.run(500);
  const loose = [...q.parts.values()].filter(p => !p.held && !p.pin).length;
  chk('an E-STOP does not let go of a hundred plugs: the trays keep what they hold',
    q.io.PAL_CLAMP === true && q.io.JIG_CLAMP === true && loose <= 5, loose + ' loose, step ' + q.io.ST1_STEP);
  chk('E-STOP: every station stops, not just the one that was moving', q.io.ST1_STEP === 910 && q.io.ST2_STEP === 0 && q.io.ST3_STEP === 0,
    q.io.ST1_STEP + '/' + q.io.ST2_STEP + '/' + q.io.ST3_STEP);
  tap('pbEstop'); tap('pbMaster');
  tap('pbHome');
  for (let i = 0; i < 80 && !q.io.PL_HOME; i++) q.run(200);
  tap('pbStart');
  q.run(20000);
  chk('E-STOP: master, home and start bring the cell back, all three stations running',
    q.io.AUTO_RUN === true && q.io.ST1_STEP < 900 && q.io.ST2_STEP >= 10 && q.io.ST3_STEP >= 100,
    q.io.ST1_STEP + '/' + q.io.ST2_STEP + '/' + q.io.ST3_STEP);
  await q.close();
}

// ---------------------------------------------------------------- the three stations run together
// One long sequence made the gantry wait for the carrier and then for the unloader before it could
// pick again. Three stations that run at once cut the cycle from about 9.5 s to under 6.
{
  const pl = JSON.parse(fs.readFileSync(path.join(ROOT, 'scenes', 'palletizing.json'), 'utf8'));
  const { create: createPL } = await import('../scenes/palletizing.ctl.js');
  const q = await createPlant(pl, { controller: createPL() });
  q.run(200); powerUp(q, 6000);
  q.run(90000);
  const st = q.status().plant;
  chk('palletizing: the plant times the cycle the scene told it to count', st.cycleTag === 'CYCLE_CNT' && st.cycleMs > 0, JSON.stringify(st.cycleMs));
  chk('palletizing: the three stations together hold a cycle under 7 s', st.avgMs > 0 && st.avgMs < 7000,
    (st.avgMs / 1000).toFixed(2) + ' s over ' + st.cycles + ', CYCLE_CNT ' + q.io.CYCLE_CNT);
  // The proof they really overlap: the gantry is somewhere else while the unloader works.
  const overlap = q.events.filter(e => e.k === 'step' && e.st === 'ST3').length;
  chk('palletizing: the unloader runs its own steps while the gantry runs its own', overlap > 40 && q.io.CYCLE_CNT >= 12,
    overlap + ' unloader steps, ' + q.io.CYCLE_CNT + ' cycles');
  await q.close();
}

// ---------------------------------------------------------------- the speed override
// The percentage dial on the panel scales what a MOTOR does - servo axes, belts, the index cam -
// and never the pneumatics: a cylinder's speed is set by its flow regulator and no dial changes
// it. The dial is an operator INPUT (plant -> PLC); the PLC passes it on to the axes.
{
  const ab = JSON.parse(fs.readFileSync(path.join(ROOT, 'scenes', 'a-to-b.json'), 'utf8'));
  const belt = async pct => {
    const q = await createPlant(ab, {});
    q.dial('dialOvr', 'ovr', pct);
    q.run(50);
    chk('speed override: the dial tells the PLC what it is set to (' + pct + '%)', q.io.OVR_SET === pct, 'OVR_SET ' + q.io.OVR_SET);
    q.force('OVR', pct); q.force('CV1_RUN', true); q.run(2000);
    const mm = q.dof.cv1;
    await q.close();
    return mm;
  };
  const full = await belt(100), half = await belt(50);
  chk('speed override: 50% runs the belt at half speed', Math.abs(half / full - 0.5) < 0.02, full.toFixed(0) + ' -> ' + half.toFixed(0) + ' mm in 2 s');

  const move = async pct => {
    const q = await createPlant(scene, {});
    q.dial('dialOvr', 'ovr', pct);
    q.run(50);
    q.force('OVR', pct); q.force('SV1_TGT', 400); q.force('SV1_EXEC', true);
    let ms = 0;
    while (!q.io.SV1_DONE && ms < 20000) { q.run(2); ms += 2; }
    await q.close();
    return ms;
  };
  const m100 = await move(100), m50 = await move(50);
  // Not exactly twice: the override scales speed, not acceleration, as on a real control.
  chk('speed override: 50% makes a servo move take longer, but not twice as long (acceleration is not scaled)',
    m50 > m100 * 1.5 && m50 < m100 * 2, m100 + ' -> ' + m50 + ' ms');

  const d = await createPlant(ab, {});
  d.dial('dialOvr', 'ovr', 500);
  d.run(50);
  chk('speed override: the dial clamps to its own range', d.io.OVR_SET === 100, 'OVR_SET ' + d.io.OVR_SET);
  d.dial('dialOvr', 'ovr', 0);
  d.run(50);
  chk('speed override: and never below the minimum on the dial', d.io.OVR_SET === 10, 'OVR_SET ' + d.io.OVR_SET);
  chk('speed override: every setting is recorded, so a run replays with it', d.events.filter(e => e.k === 'dial').length === 2);
  chk('speed override: a scene without the dial runs at full speed', (await (async () => {
    const q = await createPlant(ab, {});
    q.force('CV1_RUN', true); q.run(1000);
    const mm = q.dof.cv1;
    await q.close();
    return mm;
  })()) > 250, 'unbound ovr means 100%');
  await d.close();
}

// ---------------------------------------------------------------- jogging an axis
// Every servo can be jogged from the panel: hold + or - and the axis creeps at the override
// speed. Jog is refused while a move is running, because two sources of motion for one axis is
// how a machine gets broken, and both buttons at once is a stop, as on a real pendant.
{
  const j = await createPlant(scene, {});
  j.run(200);
  chk('jog: the axis starts at zero', j.dof.slide1 === 0);
  j.force('SV1_JOG_P', true); j.run(400);
  const fwd = j.dof.slide1;
  // A pendant jogs at a FRACTION of the cycle rate (`jogPct`, 10% by default): slide1 runs at
  // 300 mm/s in a cycle, so it creeps at 30 and covers 12 mm in 400 ms. At the full rate it
  // crossed its 500 mm stroke in under two seconds, which cannot be placed by hand.
  chk('jog: holding + creeps the axis forward, at a tenth of the cycle rate', fwd > 8 && fwd < 20, fwd.toFixed(1) + ' mm in 400 ms');
  j.force('SV1_JOG_P', null); j.run(200);
  chk('jog: letting go stops it where it is', Math.abs(j.dof.slide1 - fwd) < 1e-9, j.dof.slide1.toFixed(1) + ' mm');
  j.force('SV1_JOG_N', true); j.run(200);
  chk('jog: holding - brings it back', j.dof.slide1 < fwd, j.dof.slide1.toFixed(1) + ' mm');
  j.force('SV1_JOG_P', true); j.run(200);
  const both = j.dof.slide1;
  j.run(200);
  chk('jog: both buttons at once is a stop', Math.abs(j.dof.slide1 - both) < 1e-9, j.dof.slide1.toFixed(1) + ' mm');
  j.force('SV1_JOG_P', null); j.force('SV1_JOG_N', null);
  // A move owns the axis: jog may not fight it.
  j.force('SV1_TGT', 400); j.force('SV1_EXEC', true); j.run(200);
  const moving = j.dof.slide1;
  j.force('SV1_JOG_N', true); j.run(200);
  chk('jog: it is refused while a move is running', j.dof.slide1 > moving, moving.toFixed(1) + ' -> ' + j.dof.slide1.toFixed(1) + ' mm');
  // The speed override scales the jog too, as on a real pendant.
  j.force('SV1_EXEC', null); j.force('SV1_JOG_N', null); j.force('SV1_JOG_P', null); j.run(100);
  const creep = async pct => {
    const q = await createPlant(scene, {});
    q.run(100);
    q.force('OVR', pct); q.force('SV1_JOG_P', true); q.run(400);
    const mm = q.dof.slide1;
    await q.close();
    return mm;
  };
  const c100 = await creep(100), c50 = await creep(50);
  chk('jog: the speed override scales the jog as well', Math.abs(c50 / c100 - 0.5) < 0.02, c100.toFixed(1) + ' -> ' + c50.toFixed(1) + ' mm');
  await j.close();

  // A jog STOPS at the axis limit. Held against the end for long enough to cross the stroke twice,
  // the axis must sit exactly on it - a pendant cannot drive an axis past its soft limit, and an
  // axis that creeps past one has left the machine the scene describes.
  {
    const k = await createPlant(scene, {});
    k.run(100);
    k.force('SV1_JOG_P', true); k.run(60000);
    chk('jog: it stops at the axis limit and does not creep past it', Math.abs(k.dof.slide1 - 500) < 1e-9, k.dof.slide1.toFixed(3) + ' mm of a 500 mm stroke');
    k.force('SV1_JOG_P', null); k.force('SV1_JOG_N', true); k.run(60000);
    chk('jog: and stops at the other end too', Math.abs(k.dof.slide1) < 1e-9, k.dof.slide1.toFixed(3) + ' mm');
    await k.close();
  }
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
b.force('PB_CSTOP', true); b.run(BLIP); b.force('PB_CSTOP', null); b.run(300);
const on = edges(b, 'PB_CSTOP', true), off = edges(b, 'PB_CSTOP', false);
chk(BLIP + ' ms blip is held for minPulseMs ' + MIN, BLIP > 0 && on.length === 1 && off.length === 1 && off[0] - on[0] === MIN, on + ' -> ' + off);
chk('the stretch is recorded as a warn', warns.some(m => m.includes('pulse stretched PB_CSTOP ' + BLIP + ' -> ' + MIN + ' ms')), warns.join(' | '));
chk('the warn is in the event log too', b.events.some(e => e.k === 'warn' && /PB_CSTOP/.test(e.msg)));

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

// ---------------------------------------------------------------- robot-pitch (LR Mate + cam head)
// A six-axis arm as a chain of joints, a cam-driven pitch-change head, and a 5 x 5 pallet: the
// discriminating measurement is that the bin receives whole rows of five, and that the cam is
// wide over the pallet and narrow over the jig - the pitch change is the point of the cell.
{
  const rp = JSON.parse(fs.readFileSync(path.join(ROOT, 'scenes', 'robot-pitch.json'), 'utf8'));
  const { create: createRP } = await import('../scenes/robot-pitch.ctl.js');
  const q = await createPlant(rp, { controller: createRP() });
  q.run(200); powerUp(q);
  const cam = { pallet: /** @type {number|null} */ (null), jig: /** @type {number|null} */ (null) };
  for (let i = 0; i < 600; i++) {
    q.run(100);
    if (q.io.ST1_STEP === 14 && cam.pallet == null) cam.pallet = q.dof.head;
    if (q.io.ST1_STEP === 21 && cam.jig == null) cam.jig = q.dof.head;
  }
  const pev = (/** @type {string} */ ev) => q.events.filter(e => e.k === 'part' && e.ev === ev).length;
  chk('robot-pitch: the loader fills the pallet, 25 plugs in 25 pockets, before the arm picks', pev('spawn') >= 25 && q.io.EM_PAL_CNT >= 25, pev('spawn') + ' plugs');
  chk('robot-pitch: rows go pallet -> jig -> bin, five at a time', q.io.CYCLE_CNT >= 5 && q.io.RM_BIN_CNT > 0 && q.io.RM_BIN_CNT % 5 === 0 && q.io.ST1_STEP < 900,
    'CYCLE_CNT ' + q.io.CYCLE_CNT + ', bin ' + q.io.RM_BIN_CNT + ', step ' + q.io.ST1_STEP);
  chk('robot-pitch: the cam is wide (0) while picking from the pallet and narrow (90) while setting into the jig',
    cam.pallet != null && Math.abs(cam.pallet) < 0.5 && cam.jig != null && Math.abs(cam.jig - 90) < 0.5, JSON.stringify(cam));
  chk('robot-pitch: every plug is accounted for and none was dropped', pev('spawn') === pev('remove') + q.parts.size && pev('lost') === 0,
    pev('spawn') + ' in, ' + pev('remove') + ' out, ' + q.parts.size + ' inside, ' + pev('lost') + ' lost');
  chk('robot-pitch: no warnings', !q.events.some(e => e.k === 'warn'), q.events.filter(e => e.k === 'warn').map(e => e.msg).slice(0, 3).join(' | '));
  chk('robot-pitch: a step costs well under its 2 ms budget', q.stepUs === 0 || q.stepUs < 1200, q.stepUs + ' us');
  await q.close();
}

// ---------------------------------------------------------------- lathe-line (VS-087 + 2 lathes)
// A hanging six-axis robot on a traverse, two lathes and ONE conveyor past their fronts, with a
// pop-up stop and a pin lift at each machine. The discriminating measurements: parts go OP10 then
// OP20 and leave off the end of the belt, and the jaw takes the part out of a chuck (or off a pin)
// that is still CLAMPED - the hand-over a tending robot lives on. A chuck that opens first drops
// the part on the floor of the machine.
{
  const ll = JSON.parse(fs.readFileSync(path.join(ROOT, 'scenes', 'lathe-line.json'), 'utf8'));
  const { create: createLL } = await import('../scenes/lathe-line.ctl.js');
  const q = await createPlant(ll, { controller: createLL() });
  q.run(200);
  powerUp(q, 6000);
  let clampedTake = 0, steps = new Set();
  for (let i = 0; i < 1600; i++) {
    q.run(100);
    steps.add(q.io.ST1_STEP);
    // the moment a jaw reports a grip, the pin or the chuck it took from is still holding on
    if ((q.io.ST1_STEP === 15 && !q.io.AS_A_OPEN && q.io.S1_CLAMP !== false) ||
        (q.io.ST1_STEP === 34 && !q.io.AS_B_OPEN && q.io.M1_CHUCK !== false)) clampedTake++;
  }
  const pev = (/** @type {string} */ ev) => q.events.filter(e => e.k === 'part' && e.ev === ev).length;
  chk('lathe-line: the cell keeps completing cycles', q.io.CYCLE_CNT >= 6 && q.io.ST1_STEP < 900, 'CYCLE_CNT ' + q.io.CYCLE_CNT + ', step ' + q.io.ST1_STEP);
  chk('lathe-line: parts go through both machines and leave off the end of the belt', q.io.RM_OUT_CNT >= 1 && q.io.ST5_STEP > 0,
    'discharged ' + q.io.RM_OUT_CNT + ' of ' + q.io.EM_IN_CNT + ' fed');
  chk('lathe-line: every part is accounted for and none was dropped', pev('spawn') === pev('remove') + q.parts.size && pev('lost') === 0,
    pev('spawn') + ' in, ' + pev('remove') + ' out, ' + q.parts.size + ' inside, ' + pev('lost') + ' lost');
  chk('lathe-line: a jaw takes the part while the pin or the chuck still holds it (the hand-over)', clampedTake > 0, clampedTake + ' scans');
  // hold events, part by part: a hand-over shows as one holder handing straight to the next with
  // no release between, which is only possible because the jaw may take out of a nest.
  const handover = q.events.filter(e => e.k === 'part' && (e.ev === 'hold' || e.ev === 'release'))
    .reduce((/** @type {any} */ acc, e) => {
      const prev = acc.last[e.uid];
      if (e.ev === 'hold' && prev && prev.startsWith('pin') && e.by.startsWith('jaw')) acc.n++;
      acc.last[e.uid] = e.ev === 'hold' ? e.by : null;
      return acc;
    }, { n: 0, last: {} }).n;
  chk('lathe-line: the pin hands a part straight to the jaw, never letting go first', handover > 0, handover + ' hand-overs');
  chk('lathe-line: the door, the stop and the lift raise no warnings', !q.events.some(e => e.k === 'warn'),
    q.events.filter(e => e.k === 'warn').map(e => e.msg).slice(0, 3).join(' | '));
  chk('lathe-line: a step costs well under its 4 ms budget', q.stepUs === 0 || q.stepUs < 2000, q.stepUs + ' us');
  await q.close();
}

// ---------------------------------------------------------------- carton-sorter (OIP numbers)
// A sortation line: two swing blades across a 1.524 m belt at 2 m/s, and a shift register that
// remembers what each carton is for between the scanner and the blade. The discriminating
// measurements: the blade deflects by DRIVING the carton along itself (the belt does the work,
// nothing pushes), the three destinations come out even because the tracking is a queue and not a
// timer, and nothing is lost off the ends - a carton leaves a 2 m/s belt as a projectile.
{
  const cs = JSON.parse(fs.readFileSync(path.join(ROOT, 'scenes', 'carton-sorter.json'), 'utf8'));
  const { create: createCS } = await import('../scenes/carton-sorter.ctl.js');
  const q = await createPlant(cs, { controller: createCS() });
  q.run(200);
  powerUp(q, 3000);
  for (let i = 0; i < 900; i++) q.run(100);
  const pev = (/** @type {string} */ ev) => q.events.filter(e => e.k === 'part' && e.ev === ev).length;
  const out = [q.io.RM_A_CNT, q.io.RM_B_CNT, q.io.RM_T_CNT];
  chk('carton-sorter: cartons keep coming and keep being sorted', q.io.CYCLE_CNT >= 30 && q.io.ST1_STEP === 10,
    'scanned ' + q.io.CYCLE_CNT + ', step ' + q.io.ST1_STEP);
  chk('carton-sorter: all three destinations get their share, within one carton', Math.max(...out) - Math.min(...out) <= 1 && Math.min(...out) > 5,
    'A ' + out[0] + ', B ' + out[1] + ', through ' + out[2]);
  chk('carton-sorter: every carton is accounted for and none flew past a discharge', pev('spawn') === pev('remove') + q.parts.size && pev('lost') === 0,
    pev('spawn') + ' in, ' + pev('remove') + ' out, ' + q.parts.size + ' on the line, ' + pev('lost') + ' lost');
  chk('carton-sorter: no warnings', !q.events.some(e => e.k === 'warn'), q.events.filter(e => e.k === 'warn').map(e => e.msg).slice(0, 3).join(' | '));
  chk('carton-sorter: a step costs well under its 4 ms budget', q.stepUs === 0 || q.stepUs < 2000, q.stepUs + ' us');
  await q.close();
}

// ---------------------------------------------------------------- mps-sorting (Festo MPS)
// Festo's MPS Sorting station: three sensors at one stop tell three workpieces apart, and each
// goes to its own chute. The discriminating measurement is that the RETRO-REFLECTIVE sensor is
// blind to the matt black workpiece while the through-beam sees it - that difference IS how the
// real station tells black from red - and that each colour ends in its own bin.
{
  const ms = JSON.parse(fs.readFileSync(path.join(ROOT, 'scenes', 'mps-sorting.json'), 'utf8'));
  const { create: createMS } = await import('../scenes/mps-sorting.ctl.js');
  const q = await createPlant(ms, { controller: createMS() });
  q.run(200);
  powerUp(q, 2000);
  /** what the three sensors said while each colour stood at the stop */
  const read = { black: null, red: null, metal: null };
  for (let i = 0; i < 1500; i++) {
    q.run(100);
    if (q.io.ST1_STEP !== 30) continue;
    const held = [...q.parts.values()][0];
    if (!held) continue;
    const k = held.tpl === 'wpBlack' ? 'black' : held.tpl === 'wpRed' ? 'red' : 'metal';
    read[k] = [!!q.io.WP_DETECTED, !!q.io.WP_NOT_BLACK, !!q.io.WP_METALLIC];
  }
  const pev = (/** @type {string} */ ev) => q.events.filter(e => e.k === 'part' && e.ev === ev).length;
  const bins = [q.io.RM_1_CNT, q.io.RM_2_CNT, q.io.RM_3_CNT];
  chk('mps-sorting: the station keeps sorting', q.io.CYCLE_CNT >= 12 && q.io.ST1_STEP !== 900, 'CYCLE_CNT ' + q.io.CYCLE_CNT + ', step ' + q.io.ST1_STEP);
  chk('mps-sorting: red to chute 1, metallic to chute 2, black off the end to chute 3',
    bins.every(n => n >= 4) && Math.max(...bins) - Math.min(...bins) <= 1, 'bins ' + bins.join('/'));
  chk('mps-sorting: the through-beam sees the BLACK workpiece and the retro-reflective one does not',
    read.black && read.black[0] === true && read.black[1] === false && read.black[2] === false, JSON.stringify(read.black));
  chk('mps-sorting: red reads as not-black and not metal', read.red && read.red[0] && read.red[1] && !read.red[2], JSON.stringify(read.red));
  chk('mps-sorting: the metallic one reads on the inductive sensor too', read.metal && read.metal[0] && read.metal[2], JSON.stringify(read.metal));
  chk('mps-sorting: every workpiece is accounted for and none was lost', pev('spawn') === pev('remove') + q.parts.size && pev('lost') === 0,
    pev('spawn') + ' in, ' + pev('remove') + ' out, ' + pev('lost') + ' lost');
  chk('mps-sorting: no warnings', !q.events.some(e => e.k === 'warn'), q.events.filter(e => e.k === 'warn').map(e => e.msg).slice(0, 3).join(' | '));
  await q.close();
}

// ---------------------------------------------------------------- what a part costs
// A scene carries hundreds of parts only because most of them are HELD. Measured on this PC with
// 200 identical plugs, loose on a running belt against sitting in nests with the clamp on:
// 2058 us a step (10.3 per part) against 80 us (0.40 per part), a factor of 26; at 400 it is 36.
// The cost is Rapier's solver on dynamic bodies - a CPU profile of 400 loose parts puts 87% of
// the time inside the WASM and 3% in step() - so it is not something the plant's own loops or
// the viewer's triangles can be tuned out of. It is a DESIGN rule: a machine that leaves a
// hundred parts loose at once is the expensive one, and a carousel whose pallets are held costs
// almost nothing. The ratio is pinned loosely (>= 4x) because a shared box makes the absolute
// numbers move, but the shape of the curve does not.
{
  const many = (/** @type {number} */ n, /** @type {boolean} */ held) => {
    const C = [{ id: 'cv', type: 'conveyor', params: { length: 4000, width: 400, height: 800, speed: 200, guides: 0 }, io: { run: 'CV_RUN' } },
               { id: 'rack', type: 'frame', at: [0, 900, 0], params: { size: [4000, 600, 800] } }];
    const cols = Math.ceil(Math.sqrt(n));
    for (let i = 0; i < n; i++) {
      const cx = -1900 + (i % cols) * (3800 / cols), cy = Math.floor(i / cols) * 60 - 200;
      if (held) C.push({ id: 'n' + i, type: 'nest', parent: 'rack', socket: 'top', at: [cx, cy, 0], params: { size: [24, 24, 60], wall: 4 }, io: { clamp: 'CLAMP' } });
      C.push({ id: 'p' + i, type: 'workpiece', at: [cx, held ? 900 + cy : 0, 806], params: { kind: 'cyl', size: [16, 16, 90], material: 'steel', dynamic: true } });
    }
    return { format: 'mio-scene/1', name: 'many', sim: { dtMs: 2 }, components: C };
  };
  const cost = async (/** @type {boolean} */ held) => {
    const q = await createPlant(many(200, held), {});
    q.force(held ? 'CLAMP' : 'CV_RUN', true);
    q.run(1500);
    const n = [...q.parts.values()].filter(p => p.held).length;
    const t0 = performance.now();
    for (let i = 0; i < 600; i++) q.run(2);
    const us = (performance.now() - t0) / 600 * 1000;
    await q.close();
    return { us, n };
  };
  const loose = await cost(false), kept = await cost(true);
  chk('200 parts held in nests are all held, and parked', kept.n === 200, kept.n + ' held');
  chk('a held part costs a fraction of a loose one (the solver never sees it move)',
    kept.us * 4 < loose.us, 'loose ' + loose.us.toFixed(0) + ' us/step, held ' + kept.us.toFixed(0) + ' us/step');
}

// Real-time pacing: a stalled second is capped at 50 steps and counted. Starting is not a stall:
// the gap before the first tick is setup and a major GC, not the plant falling behind.
let clk = 0;
const r = await createPlant(scene, { clock: () => clk });
r.start();
await new Promise(res => setTimeout(res, 30));
chk('the gap before the first tick is not counted as an overrun', r.t === 0 && r.overruns === 0, 't ' + r.t + ' ms, overruns ' + r.overruns);
clk = 1000;
await new Promise(res => setTimeout(res, 30));
r.stop();
chk('a 1 s stall runs 50 steps and counts the rest as overruns', r.t === 100 && r.overruns === 450, 't ' + r.t + ' ms, overruns ' + r.overruns);

// A HICCUP is not a stall. A GC, or the OS scheduling Sysmac Studio and Chrome ahead of Node,
// leaves the plant a few hundred ms behind; dropping those steps and warning "plant stalled"
// on every such hiccup is what made the simulation feel fragile (found jogging palletizing
// with the PLC on the same laptop). Below DEBT_MAX_MS the plant catches up within a wall budget
// per tick and carries the rest: no step dropped, nothing warned, sim time back level.
{
  let hclk = 0;
  const warned = [];
  const h = await createPlant(scene, { clock: () => hclk });
  h.warnListeners.push((/** @type {string} */ m) => warned.push(m));
  h.start();
  await new Promise(res => setTimeout(res, 30));
  hclk = 300;                                                // a 300 ms hiccup
  await new Promise(res => setTimeout(res, 120));            // a few ticks to repay it
  h.stop();
  chk('a 300 ms hiccup is caught up, not dropped: sim time is level again', h.t === 300 && h.overruns === 0, 't ' + h.t + ' ms, overruns ' + h.overruns);
  chk('and it never warned about it', warned.length === 0, warned.join(' | '));
  chk('the status says how far behind the plant is, and it is back to zero', h.behindMs === 0, h.behindMs + ' ms');
  await h.close();
}

// World speed: sim time runs at a multiple of wall time - slow motion to watch, or ahead to get
// through a cycle. It is
// a SIMULATOR control, not a machine one, and it is forced back to 1x whenever a PLC is connected,
// because Sysmac timers run on wall time and a slowed plant would lie to the program.
{
  let sclk = 0;
  const sp = await createPlant(scene, { clock: () => sclk });
  chk('slow motion: a plant starts at 1x', sp.scale === 1);
  sp.setScale(0.25);
  sp.start();
  await new Promise(res => setTimeout(res, 30));
  sclk = 400;
  await new Promise(res => setTimeout(res, 30));
  sp.stop();
  chk('slow motion: at 1/4x, 400 ms of wall time is 100 ms of plant', sp.t === 100 && sp.overruns === 0, sp.t + ' ms, overruns ' + sp.overruns);
  chk('slow motion: the setting is recorded, so a run replays with it', sp.events.some(e => e.k === 'scale' && e.v === 0.25));
  chk('world speed: it runs ahead of the clock too, up to 4x', sp.setScale(4) === 4 && sp.setScale(8) === 4);
  // The cap on one tick is a cap on SIM time, so it has to grow with the world speed. Left fixed
  // it warned "plant stalled" several times a second at 2x-4x on a plant that was keeping up.
  {
    let fclk = 0;
    const fast = await createPlant(scene, { clock: () => fclk });
    fast.setScale(4);
    fast.start();
    await new Promise(res => setTimeout(res, 30));
    fclk = 1000;
    await new Promise(res => setTimeout(res, 30));
    fast.stop();
    chk('world speed: at 4x one tick may run four times as many steps before it counts an overrun',
      fast.t === 400 && fast.overruns === 1800, fast.t + ' ms, overruns ' + fast.overruns);
    await fast.close();
  }
  await sp.close();

  const warned = [];
  const fake = { name: 'fake', ready: false, write: async () => {}, status: () => ({ driver: 'fake', ok: false }), close: async () => {} };
  const pl = await createPlant(scene, { driver: fake });
  pl.warnListeners.push((/** @type {string} */ m) => warned.push(m));
  chk('slow motion: with a PLC connected the plant refuses to slow down, and says why',
    pl.setScale(0.25) === 1 && pl.scale === 1 && warned.some(m => /time scale stays 1x/.test(m)), warned.join(' | '));
  await pl.close();
}

// Static: physics never writes to the PLC. driver.write appears once, inside exchange().
const src = fs.readFileSync(path.join(ROOT, 'server', 'plant.js'), 'utf8');
const calls = src.match(/driver\.write\(/g) || [];
const ex = src.slice(src.indexOf('function exchange('), src.indexOf('function fromPlc('));
chk('driver.write is called only from exchange()', calls.length === 1 && ex.includes('driver.write('));

for (const pl of [p, p2, q, k, b, x, r]) await pl.close();
process.exit(fail ? 1 : 0);
