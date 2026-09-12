// Phase 0 smoke test: the Rapier features the plant is built on, in Node, Z-up, metres.
// Kinematic bodies moving and pushing, castRay, intersectionsWithShape, determinism.
let fail = 0;
const chk = (l, c, x) => { if (!c) fail++; console.log((c ? '  OK  ' : '>>BAD ') + l + (x ? '   ' + x : '')); };

let R;
try {
  const mod = await import('@dimforge/rapier3d-deterministic-compat');
  R = mod.default ?? mod;
  await R.init();
} catch (e) {
  console.log('  SKIP  Rapier not loadable (' + String(e.message || e).split('\n')[0] + '): run npm install');
  process.exit(0);
}

const DT = 0.002;                               // docs/PLAN.md §5: 2 ms plant step

/** Floor, a resting ball, and a kinematic pusher sweeping +X through it. */
function scenario(steps, partCanSleep = false) {
  const w = new R.World({ x: 0, y: 0, z: -9.81 });
  w.timestep = DT;
  const floor = w.createRigidBody(R.RigidBodyDesc.fixed().setTranslation(0, 0, -0.05));
  w.createCollider(R.ColliderDesc.cuboid(2, 2, 0.05), floor);
  const ball = w.createRigidBody(R.RigidBodyDesc.dynamic().setTranslation(0, 0, 0.2).setCanSleep(partCanSleep));
  const ballCol = w.createCollider(R.ColliderDesc.ball(0.02).setFriction(0.6), ball);
  const pusher = w.createRigidBody(R.RigidBodyDesc.kinematicPositionBased().setTranslation(-0.3, 0, 0.03));
  w.createCollider(R.ColliderDesc.cuboid(0.01, 0.05, 0.03), pusher);
  const trace = [];
  for (let i = 0; i < steps; i++) {
    const t = i * DT;
    // stay still for 1 s so the ball settles, then sweep 0.6 m in 1 s
    const x = t < 1 ? -0.3 : Math.min(0.3, -0.3 + (t - 1) * 0.6);
    pusher.setNextKinematicTranslation({ x, y: 0, z: 0.03 });
    w.step();
    if (i === 499) trace.push(ball.translation().z);        // resting height after 1 s
  }
  return { w, ball, ballCol, pusher, trace };
}

const a = scenario(1000);
chk('ball rests on the floor, Z-up (z ~ radius 0.02)', Math.abs(a.trace[0] - 0.02) < 0.002, 'z=' + a.trace[0].toFixed(4));
chk('kinematic body reached its target', Math.abs(a.pusher.translation().x - (-0.3 + (2 - 1 - DT) * 0.6)) < 0.01,
  'x=' + a.pusher.translation().x.toFixed(4));

// The trap, measured: a part that fell asleep while resting is NOT woken by a kinematic
// pusher. The pusher passes through it, and the part stays put without any error. Waking
// the pusher does not help. Hence the rule: workpieces never sleep. If this check starts
// failing, Rapier fixed it and the rule can be revisited.
const trap = scenario(1500, true);
chk('trap: a SLEEPING part is not pushed by a kinematic body (Rapier 0.20)', trap.ball.translation().x === 0,
  'ball x=' + trap.ball.translation().x.toFixed(3));

const b = scenario(1500);
chk('rule: a part that cannot sleep IS pushed along +X', b.ball.translation().x > 0.1, 'ball x=' + b.ball.translation().x.toFixed(3));

const hit = b.w.castRay(new R.Ray({ x: 1.5, y: 1.5, z: 1 }, { x: 0, y: 0, z: -1 }), 10, true);
const toi = hit && (hit.timeOfImpact ?? hit.toi);
chk('castRay down hits the floor top at 1 m', hit && Math.abs(toi - 1) < 1e-4, 'toi=' + toi);

let found = false;
const p = b.ball.translation();
b.w.intersectionsWithShape(p, { x: 0, y: 0, z: 0, w: 1 }, new R.Ball(0.01), col => { if (col.handle === b.ballCol.handle) found = true; return true; });
chk('intersectionsWithShape finds the ball at its own position', found);

const c = scenario(1500);
const same = ['x', 'y', 'z'].every(k => b.ball.translation()[k] === c.ball.translation()[k]);
chk('two identical runs end bit-identical (deterministic build)', same,
  JSON.stringify(b.ball.translation()) + ' vs ' + JSON.stringify(c.ball.translation()));

// ---------------------------------------------------------------- conveyor drive (spike A0)
// A 3 m belt at 0.3 m/s, 60 x 40 x 30 mm alu parts, mu 0.5. The belt collider has friction 0
// (combine rule Min) so Rapier's own friction does not brake parts against the static belt:
// the drive model is the belt's only grip.
const { beltDv } = await import('../server/plant.js');
const V = 0.3, MU = 0.5, HALF = [0.03, 0.02, 0.015];
function belt(model, n, stopperX) {
  const w = new R.World({ x: 0, y: 0, z: -9.81 });
  w.timestep = DT;
  const bb = w.createRigidBody(R.RigidBodyDesc.fixed().setTranslation(1.5, 0, -0.01));
  const bc = w.createCollider(R.ColliderDesc.cuboid(1.5, 0.1, 0.01).setFriction(0).setFrictionCombineRule(R.CoefficientCombineRule.Min), bb);
  if (stopperX != null) w.createCollider(R.ColliderDesc.cuboid(0.005, 0.1, 0.02), w.createRigidBody(R.RigidBodyDesc.fixed().setTranslation(stopperX + 0.005, 0, 0.02)));
  const parts = [];
  for (let i = 0; i < n; i++) {
    const b = w.createRigidBody(R.RigidBodyDesc.dynamic().setTranslation(0.1 + i * 0.1, 0, HALF[2] + 0.001).setCanSleep(false).setCcdEnabled(true));
    parts.push({ b, c: w.createCollider(R.ColliderDesc.cuboid(...HALF).setDensity(2700).setFriction(MU), b) });
  }
  const step = () => {
    for (const p of parts) {
      let touch = false;
      w.contactPair(bc, p.c, m => { if (m.numContacts() > 0) touch = true; });
      if (!touch) continue;
      const v = p.b.linvel();
      if (model === 'override') { p.b.setLinvel({ x: V, y: v.y, z: v.z }, true); continue; }
      const dv = beltDv([v.x, v.y, v.z], [V, 0, 0], [0, 0, 1], MU, DT), m = p.b.mass();
      p.b.applyImpulse({ x: m * dv[0], y: m * dv[1], z: m * dv[2] }, true);
    }
    w.step();
  };
  return { parts, step };
}
{
  const one = belt('slip', 1);
  let tArr = null;
  for (let i = 1; i <= 6000 && tArr == null; i++) { one.step(); if (one.parts[0].b.translation().x >= 1.1) tArr = i * DT; }
  const ideal = 1 / V + V / (2 * MU * 9.81);            // 1 m at belt speed + the lag of accelerating at mu*g
  chk('slip drive: a part travels 1 m in distance/speed + v/(2 mu g), +-2 steps', tArr != null && Math.abs(tArr - ideal) <= 2 * DT, tArr?.toFixed(3) + ' s vs ' + ideal.toFixed(3));

  const run = model => {
    const q = belt(model, 5, 1.2), hist = q.parts.map(() => []);
    for (let i = 1; i <= 5000; i++) { q.step(); if (i > 4000) q.parts.forEach((p, k) => hist[k].push(p.b.translation().x)); }
    const xs = q.parts.map(p => p.b.translation().x).sort((a, b) => b - a);
    return { xs, gaps: xs.slice(1).map((x, i) => (xs[i] - x) * 1000), zmax: Math.max(...q.parts.map(p => p.b.translation().z)) * 1000,
             jitter: Math.max(...hist.map(h => Math.max(...h) - Math.min(...h))) * 1000 };
  };
  const s = run('slip');
  chk('slip drive: 5 parts queue against a stopper, 59-60.5 mm apart (60 mm parts)', s.gaps.every(g => g > 59 && g < 60.5), s.gaps.map(g => g.toFixed(2)).join(' / '));
  chk('slip drive: the queue is still (jitter < 0.05 mm over the last 2 s), nothing climbs', s.jitter < 0.05 && s.zmax < 15.5, 'jitter ' + s.jitter.toFixed(3) + ' mm, zmax ' + s.zmax.toFixed(2) + ' mm');
  chk('slip drive: the front part rests on the stopper face (1170 mm)', Math.abs(s.xs[0] * 1000 - 1170) < 0.5, (s.xs[0] * 1000).toFixed(2));
  chk('slip drive: two runs end bit-identical', run('slip').xs.every((x, i) => x === s.xs[i]));
  // Why not the velocity override PLAN first had: it keeps shoving blocked parts, and they climb on each other.
  const o = run('override');
  chk('trap: velocity override stacks a queue (a gap under 50 mm or a part lifted)', o.gaps.some(g => g < 50) || o.zmax > 20, 'gaps ' + o.gaps.map(g => g.toFixed(1)).join(' / ') + ', zmax ' + o.zmax.toFixed(1));
}

// ---------------------------------------------------------------- stale contact on a lifted stopper
// A part queued against a kinematic stopper (belt slip presses it on) keeps a BLOCKING contact
// after the stopper lifts clear: measured stuck even with the stopper 20 mm above the part,
// for a cylinder and a box alike, CCD on or off. Rule (server/plant.js): when a kinematic link
// comes to rest, its colliders sit out one step and the contacts are recomputed. With that, a
// box stopper releases at any clearance; a cylinder still needs about 12 mm (so the Stopper
// preset is a square block). If the trap check starts failing, Rapier fixed it.
{
  const Y_TO_Z = { x: Math.SQRT1_2, y: 0, z: 0, w: Math.SQRT1_2 };
  const lifted = ({ shape, clear, refresh }) => {
    const w = new R.World({ x: 0, y: 0, z: -9.81 });
    w.timestep = DT;
    const bc = w.createCollider(R.ColliderDesc.cuboid(1, 0.1, 0.01).setTranslation(0, 0, -0.01).setFriction(0).setFrictionCombineRule(R.CoefficientCombineRule.Min), w.createRigidBody(R.RigidBodyDesc.fixed()));
    const part = w.createRigidBody(R.RigidBodyDesc.dynamic().setTranslation(0.2, 0, 0.0151).setCanSleep(false).setCcdEnabled(true));
    const pc = w.createCollider(R.ColliderDesc.cuboid(0.03, 0.02, 0.015).setDensity(2700).setFriction(0.5), part);
    const down = 0.005 + 0.0125, up = 0.030 + clear / 1000 + 0.0125;       // 25 mm stopper, 5 mm above the belt when down
    const pin = w.createRigidBody(R.RigidBodyDesc.kinematicPositionBased().setTranslation(0.3, 0, down));
    const col = w.createCollider(shape === 'cyl' ? R.ColliderDesc.cylinder(0.0125, 0.006).setRotation(Y_TO_Z) : R.ColliderDesc.cuboid(0.006, 0.006, 0.0125), pin);
    const step = z => {
      pin.setNextKinematicTranslation({ x: 0.3, y: 0, z });
      let touch = false;
      w.contactPair(bc, pc, m => { if (m.numContacts() > 0) touch = true; });
      if (touch) { const v = part.linvel(), dv = beltDv([v.x, v.y, v.z], [V, 0, 0], [0, 0, 1], 0.6, DT), m = part.mass(); part.applyImpulse({ x: m * dv[0], y: m * dv[1], z: m * dv[2] }, true); }
      w.step();
    };
    for (let i = 0; i < 400; i++) step(down);                              // the part rides up and queues on the stopper
    for (let i = 1; i <= 60; i++) step(down + (up - down) * i / 60);       // the stopper lifts clear
    if (refresh) { col.setEnabled(false); step(up); col.setEnabled(true); }
    for (let i = 0; i < 300; i++) step(up);
    return part.translation().x * 1000;
  };
  const t1 = lifted({ shape: 'cyl', clear: 20, refresh: false }), t2 = lifted({ shape: 'box', clear: 20, refresh: false });
  chk('trap: a part pressed against a stopper stays stuck after it lifts 20 mm clear (cylinder and box)', t1 < 270 && t2 < 270, 'part x ' + t1.toFixed(1) + ' / ' + t2.toFixed(1) + ' mm');
  const r1 = lifted({ shape: 'box', clear: 5, refresh: true }), r2 = lifted({ shape: 'cyl', clear: 15, refresh: true });
  chk('rule: with the contact refresh, a box stopper 5 mm clear (and a cylinder 15 mm clear) releases the part', r1 > 330 && r2 > 330, 'part x ' + r1.toFixed(1) + ' / ' + r2.toFixed(1) + ' mm');
}

// ---------------------------------------------------------------- a part dropped into a pocket
// Measured: a 70 x 50 x 20 mm part dropped into a pocket with 25 mm walls HANGS on speculative
// contacts at the wall top edges (45° normals, 7 mm of reported distance) unless the clearance
// is generous: 19.9 mm above the floor at 5 mm per side, landing only from 12 mm. Walls shorter
// than the part need less. Hence the rule: a nest draws its walls but does not collide with
// them, and any guide a part must drop between leaves >= 12 mm per side.
{
  const drop = (gapMm, wallH) => {
    const w = new R.World({ x: 0, y: 0, z: -9.81 });
    w.timestep = DT;
    const floor = w.createRigidBody(R.RigidBodyDesc.fixed());
    w.createCollider(R.ColliderDesc.cuboid(0.1, 0.1, 0.003).setTranslation(0, 0, -0.003), floor);
    const g = gapMm / 1000, hx = 0.035 + g, hy = 0.025 + g;
    for (const ix of [-1, 1]) w.createCollider(R.ColliderDesc.cuboid(0.004, hy + 0.008, wallH / 2).setTranslation(ix * (hx + 0.004), 0, wallH / 2), floor);
    for (const iy of [-1, 1]) w.createCollider(R.ColliderDesc.cuboid(hx, 0.004, wallH / 2).setTranslation(0, iy * (hy + 0.004), wallH / 2), floor);
    const b = w.createRigidBody(R.RigidBodyDesc.dynamic().setTranslation(0, 0, 0.040).setCanSleep(false).setCcdEnabled(true));
    w.createCollider(R.ColliderDesc.cuboid(0.035, 0.025, 0.010).setDensity(2700).setFriction(0.5), b);
    for (let i = 0; i < 500; i++) w.step();
    return (b.translation().z - 0.010) * 1000;                      // above the pocket floor, mm
  };
  const tight = drop(5, 0.025), wide = drop(12, 0.025), low = drop(5, 0.008);
  chk('trap: a part dropped into a pocket with 5 mm clearance hangs above the floor', tight > 5, tight.toFixed(2) + ' mm up');
  chk('rule: 12 mm of clearance per side lands it (and walls below the part need only 5 mm)', wide < 0.5 && low < 0.5, wide.toFixed(2) + ' / ' + low.toFixed(2) + ' mm');
}

process.exit(fail ? 1 : 0);
